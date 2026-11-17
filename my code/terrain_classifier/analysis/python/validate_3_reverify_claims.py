"""
validate_3_reverify_claims.py
===============================================================================
검증 [3] 핵심 아키텍처 결론 재검증 — analyze_locomotion_bank.py 가 원본 6명
(AB06,07,08,09,10,12 — AB11 제외)으로 낸 결론들이, held-out 전원(AB11 제외,
신규 AB13~21/28/30 포함)을 더한 큰 그룹에서도 그대로 유지되는지 비교한다.

AB11 은 전극 문제 의심으로 두 그룹 모두에서 제외 — "작은 깨끗한 그룹"과
"큰 깨끗한 그룹"을 비교하는 것이 목적이라 나쁜 세션이 어느 한쪽에만 섞이면
비교가 오염된다.

재검증 대상 (Locomotion_Mode_Classifier.md / ref_curve_tables.h 에 박제된 주장들):
  (A) TURN vs WALK 는 41 feature 전부에서 d_subj <= 0.80 이다
      -> TURN 을 z-score 뱅크에서 빼고 hip IMU yaw-rate 로 따로 판정하는 근거
  (B) EMG greedy 1순위 = gastrocmed(총 gain 5.44), 2순위 = vastusmedialis(+0.72)
  (C) shank/foot 대체 손실 평균 1.7% (15쌍 기준, 12쌍은 0%, 최대 11.2%)
  (D) IMU_thigh_TILT 는 모드별 평균이 전부 -85~-87 deg 로 사실상 동일(flat)
===============================================================================
"""

import numpy as np
from itertools import combinations
from camargo_common import (
    MAT_PATH_ALL, EXCLUDED_SUBJECTS, ORIGINAL_SUBJECTS, NEW_SUBJECTS,
    load_population, curve, pooled_mean_std,
)

LOCO_MODES = ["WALK", "TURN", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]

IMU_AVAILABLE = ["IMU_thigh_TILT", "IMU_thigh_GYROMAG", "IMU_trunk_TILT", "IMU_trunk_GYROMAG"]
IMU_UNAVAILABLE = ["IMU_shank_TILT", "IMU_shank_GYROMAG", "IMU_foot_TILT", "IMU_foot_GYROMAG"]
MUSCLES = ["gastrocmed", "tibialisanterior", "soleus", "vastusmedialis",
           "vastuslateralis", "rectusfemoris", "bicepsfemoris", "semitendinosus",
           "gracilis", "gluteusmedius", "rightexternaloblique"]
EMG_METRICS = ["MAV", "RMS", "ZC"]
EPS = 1e-9


def emg_feats(muscles):
    return [f"EMG_{m}_{k}" for m in muscles for k in EMG_METRICS]


class Bank:
    """지정한 subject_list 로만 d_pop/d_subj 를 계산 (파일 전체가 아니라)."""

    def __init__(self, sub_struct, subject_list):
        self.sub = sub_struct
        self.subjects = subject_list
        self._pop_curve_cache = {}
        self._pop, self._subj = {}, {}

    def _pop_curve(self, mode, feat):
        key = (mode, feat)
        if key not in self._pop_curve_cache:
            stats = []
            for s in self.subjects:
                sc = getattr(self.sub, s, None)
                if sc is None:
                    continue
                c = curve(sc, mode, feat)
                if c is not None:
                    stats.append(c)
            if not stats:
                self._pop_curve_cache[key] = None
            else:
                mu, sd = pooled_mean_std(stats)
                n = sum(c[2] for c in stats)
                self._pop_curve_cache[key] = (mu, sd, n)
        return self._pop_curve_cache[key]

    def d_rms(self, cA, cB):
        muA, sdA, _ = cA
        muB, sdB, _ = cB
        pooled = np.maximum(np.sqrt((sdA ** 2 + sdB ** 2) / 2.0), EPS)
        return float(np.sqrt(np.mean(((muB - muA) / pooled) ** 2)))

    def d_pop(self, mA, mB, f):
        k = (mA, mB, f)
        if k not in self._pop:
            cA, cB = self._pop_curve(mA, f), self._pop_curve(mB, f)
            self._pop[k] = self.d_rms(cA, cB) if (cA and cB) else 0.0
        return self._pop[k]

    def d_subj(self, mA, mB, f):
        k = (mA, mB, f)
        if k not in self._subj:
            vals = []
            for s in self.subjects:
                sc = getattr(self.sub, s, None)
                if sc is None:
                    continue
                cA, cB = curve(sc, mA, f), curve(sc, mB, f)
                if cA and cB:
                    vals.append(self.d_rms(cA, cB))
            self._subj[k] = float(np.median(vals)) if vals else 0.0
        return self._subj[k]

    def best_over(self, mA, mB, feats, metric="subj"):
        fn = self.d_subj if metric == "subj" else self.d_pop
        best_f, best_v = None, -1.0
        for f in feats:
            v = fn(mA, mB, f)
            if v > best_v:
                best_v, best_f = v, f
        return best_f, best_v

    def greedy_muscles(self, n_slots=3):
        pairs = list(combinations(LOCO_MODES, 2))

        def score(muscles):
            pool = IMU_AVAILABLE + emg_feats(muscles)
            return sum(self.best_over(a, b, pool, "subj")[1] for a, b in pairs)

        base = score([])
        chosen, gains = [], []
        for _ in range(n_slots):
            best_m, best_s = None, -1
            for m in MUSCLES:
                if m in chosen:
                    continue
                s = score(chosen + [m])
                if s > best_s:
                    best_s, best_m = s, m
            chosen.append(best_m)
            gains.append(best_s - base)
            base = best_s
        return chosen, gains

    def shank_foot_loss(self):
        pairs = list(combinations(LOCO_MODES, 2))
        losses = []
        for a, b in pairs:
            _, va = self.best_over(a, b, IMU_AVAILABLE + IMU_UNAVAILABLE + emg_feats(MUSCLES), "subj")
            _, ve = self.best_over(a, b, IMU_AVAILABLE + emg_feats(MUSCLES), "subj")
            loss = (1 - ve / max(va, EPS)) * 100
            losses.append(loss)
        return losses


def main():
    print("=" * 100)
    print(" 검증[3] 핵심 아키텍처 결론 재검증 — 작은(원본6) 그룹 기준 결론이 큰(held-out포함) 그룹에서도 유지되는가")
    print("=" * 100)

    _, sub_all, _ = load_population(MAT_PATH_ALL)
    small_group = list(ORIGINAL_SUBJECTS)
    big_group = [s for s in ORIGINAL_SUBJECTS + NEW_SUBJECTS if s not in EXCLUDED_SUBJECTS]

    bank_small = Bank(sub_all, small_group)
    bank_big = Bank(sub_all, big_group)
    print(f"\n작은 그룹({len(small_group)}명): {small_group}")
    print(f"큰 그룹({len(big_group)}명):   {big_group}")
    print(f"(제외: {EXCLUDED_SUBJECTS})")

    groups = [("작은(원본6)", bank_small), ("큰(전체-AB11)", bank_big)]

    # ---------------- (A) TURN vs WALK ----------------
    print("\n\n###### (A) TURN vs WALK 판별력 — 원래 결론: 41 feature 전부 d_subj<=0.80 ######")
    all_feats = IMU_AVAILABLE + IMU_UNAVAILABLE + emg_feats(MUSCLES)
    for label, bank in groups:
        best_f, best_v = bank.best_over("TURN", "WALK", all_feats, "subj")
        verdict = "실제 분리기준(d>=2.0)에 한참 못미침 -> 판별 불가 유지" if best_v < 2.0 else "!! 분리 가능 수준, 재검토 필요"
        print(f"  [{label}] 최강 feature = {best_f:24s}  d_subj = {best_v:.3f}   -> {verdict}")

    # ---------------- (B) EMG greedy ----------------
    print("\n\n###### (B) EMG greedy 근육 선택 — 원래 결론: 1위 gastrocmed(+5.44), 2위 vastusmedialis(+0.72) ######")
    for label, bank in groups:
        chosen, gains = bank.greedy_muscles(3)
        print(f"  [{label}]")
        cum = 0.0
        for slot, (m, g) in enumerate(zip(chosen, gains), 1):
            cum += g
            print(f"      {slot}자리: {m:20s}  이번 슬롯 증가분=+{g:.2f}  (누적={cum:.2f})")

    # ---------------- (C) shank/foot 대체 손실 ----------------
    print("\n\n###### (C) shank/foot 대체 손실 — 원래 결론: 15쌍 평균 1.7% (최대 11.2%, 12쌍은 0%) ######")
    for label, bank in groups:
        losses = bank.shank_foot_loss()
        zero_pairs = sum(1 for l in losses if l < 1e-6)
        print(f"  [{label}] 평균={np.mean(losses):.2f}%  최대={np.max(losses):.2f}%  "
              f"손실0%인 쌍={zero_pairs}/{len(losses)}")

    # ---------------- (D) IMU_thigh_TILT flatness ----------------
    print("\n\n###### (D) IMU_thigh_TILT 모드별 평균 — 원래 결론: 전부 -85~-87 deg 로 동일(flat) ######")
    bank_modes_for_d = ["WALK", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]
    for label, bank in groups:
        print(f"  [{label}]")
        vals = []
        for mode in bank_modes_for_d:
            c = bank._pop_curve(mode, "IMU_thigh_TILT")
            if c is None:
                continue
            mu = float(np.mean(c[0]))
            vals.append(mu)
            print(f"      {mode:14s} 평균 = {mu:7.2f} deg")
        spread = max(vals) - min(vals)
        print(f"      -> 모드간 평균 범위(max-min) = {spread:.2f} deg  "
              f"({'여전히 flat' if spread < 5 else '!! flat 아님, 재검토 필요'})")

    print("\n" + "=" * 100)
    print(" 결론 요약은 위 각 항목의 '->' 표시를 확인. 작은/큰 그룹 결과가 방향과 크기 모두")
    print(" 비슷하면 원래 결론이 표본의 우연이 아니라 실제 구조적 특성이라는 뜻이다.")
    print("=" * 100)


if __name__ == "__main__":
    main()
