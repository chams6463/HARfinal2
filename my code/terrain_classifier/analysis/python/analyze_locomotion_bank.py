"""
analyze_locomotion_bank.py
===============================================================================
z-score 뱅크 설계를 위한 최종 분석.

배경 (diag_stand_curves2.m 으로 확인된 사실)
--------------------------------------------
Camargo 의 'idle' 라벨은 "정지"가 아니다:
    STAND median |w| = 0.923 rad/s (53 deg/s), p95 = 2.99 rad/s
    STAIRS_DOWN median |w| = 0.872 rad/s  <- STAND 가 계단내려가기보다 크다
    STAND run 길이 = 0.09 s ~ 9.51 s (100:1)  <- 균등분할 위상정렬 무의미
따라서 h_d^STAND(theta) 는 기준곡선으로 쓸 수 없다.

=> 설계 결정: STAND 는 z-score 뱅크에서 제외하고 forwardVelocity/각속도
   에너지 게이트로 판정한다 (브리핑 5장 Tier4 의 STAND_TO_WALK 결론과 동일).
   뱅크는 보행 6모드만 판별한다:
       WALK / TURN / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN

이 스크립트가 답하는 것
-----------------------
 (1) 보행 6모드 15개 쌍의 분리도 — IMU 가용 feature만으로 어디까지 되나
 (2) EMG 근육 자리를 몇 개, 어느 근육으로 해야 하나 (greedy, 데이터 기반)
 (3) C 로 export 할 feature 최종 선정 (모드x feature 개수 = 메모리 예산)
 (4) TURN 이 정말 최약체인가, 어떤 feature 가 그나마 살리나

지표: 위상분해 Cohen's d
    d(theta) = (mB(theta)-mA(theta)) / sqrt((sA(theta)^2+sB(theta)^2)/2)
    d_rms = sqrt(mean_theta d^2)
  * d_pop  : 7명 pooled (피험자간 분산 포함) = 캘리브레이션 없는 경우
  * d_subj : 피험자별 계산 후 |d| 중앙값     = BTN1 캘리브레이션 후 (설계 기준)

실행: python analyze_locomotion_bank.py
===============================================================================
"""

import numpy as np
import scipy.io as sio
from collections import defaultdict
from itertools import combinations

MAT_PATH = r"C:\Users\user\Desktop\onephai_ifac\Camargo_matlab\h_d_population.mat"

LOCO_MODES = ["WALK", "TURN", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]

IMU_AVAILABLE = [
    "IMU_thigh_TILT",      # <- leftThighAngle / rightThighAngle
    "IMU_thigh_GYROMAG",   # <- hip IMU 3축 자이로 크기 (또는 thigh angle 미분)
    "IMU_trunk_TILT",      # <- pelvicAngle
    "IMU_trunk_GYROMAG",   # <- hip IMU 3축 자이로 크기 (장착위치 모호성 있음)
]
IMU_UNAVAILABLE = ["IMU_shank_TILT", "IMU_shank_GYROMAG",
                   "IMU_foot_TILT", "IMU_foot_GYROMAG"]

MUSCLES = ["gastrocmed", "tibialisanterior", "soleus", "vastusmedialis",
           "vastuslateralis", "rectusfemoris", "bicepsfemoris", "semitendinosus",
           "gracilis", "gluteusmedius", "rightexternaloblique"]
EMG_METRICS = ["MAV", "RMS", "ZC"]

EPS = 1e-9


def load():
    d = sio.loadmat(MAT_PATH, struct_as_record=False, squeeze_me=True)
    return d["h_d_population"], d["h_d_subject"]


def curve(container, mode, feat):
    ms = getattr(container, mode, None)
    if ms is None:
        return None
    fs = getattr(ms, feat, None)
    if fs is None:
        return None
    mu = np.atleast_1d(np.asarray(fs.mean, float))
    sd = np.atleast_1d(np.asarray(fs.std, float))
    if mu.size != 101 or sd.size != 101:
        return None
    if not (np.all(np.isfinite(mu)) and np.all(np.isfinite(sd))):
        return None
    return mu, sd


def d_rms(cA, cB):
    muA, sdA = cA
    muB, sdB = cB
    pooled = np.maximum(np.sqrt((sdA ** 2 + sdB ** 2) / 2.0), EPS)
    dc = (muB - muA) / pooled
    return float(np.sqrt(np.mean(dc ** 2)))


class Bank:
    """feature별 d 계산 결과 캐시."""

    def __init__(self):
        self.pop, self.sub = load()
        self.subjects = list(self.sub._fieldnames)
        self._pop = {}
        self._subj = {}

    def d_pop(self, mA, mB, f):
        k = (mA, mB, f)
        if k not in self._pop:
            cA, cB = curve(self.pop, mA, f), curve(self.pop, mB, f)
            self._pop[k] = d_rms(cA, cB) if (cA and cB) else 0.0
        return self._pop[k]

    def d_subj(self, mA, mB, f):
        """피험자별 d_rms 의 중앙값 (캘리브레이션 후 기대 성능)."""
        k = (mA, mB, f)
        if k not in self._subj:
            vals = []
            for s in self.subjects:
                sc = getattr(self.sub, s, None)
                if sc is None:
                    continue
                cA, cB = curve(sc, mA, f), curve(sc, mB, f)
                if cA and cB:
                    vals.append(d_rms(cA, cB))
            self._subj[k] = float(np.median(vals)) if vals else 0.0
        return self._subj[k]


def best_over(bank, mA, mB, feats, metric):
    fn = bank.d_subj if metric == "subj" else bank.d_pop
    best_f, best_v = None, -1.0
    for f in feats:
        v = fn(mA, mB, f)
        if v > best_v:
            best_v, best_f = v, f
    return best_f, best_v


def emg_feats(muscles):
    return [f"EMG_{m}_{k}" for m in muscles for k in EMG_METRICS]


def main():
    bank = Bank()
    pairs = list(combinations(LOCO_MODES, 2))

    print("=" * 84)
    print(" 보행 6모드 z-score 뱅크 설계 분석  (STAND 제외 — idle 라벨 오염으로 사용 불가)")
    print(" d_subj = 피험자별 위상분해 Cohen's d 중앙값 (BTN1 캘리브레이션 후 기준)")
    print(" d_pop  = 7명 pooled (캘리브레이션 없을 때)")
    print("=" * 84)

    # ---------------- (1) 쌍별 분리도 매트릭스 ----------------
    for metric, label in [("subj", "d_subj (캘리브레이션 후)"), ("pop", "d_pop (무보정)")]:
        print(f"\n\n###### (1{'a' if metric=='subj' else 'b'}) 6모드 쌍별 최고 분리도 — IMU 가용 4개만 [{label}] ######")
        hdr = "%-14s" % "" + "".join("%13s" % m[:12] for m in LOCO_MODES)
        print(hdr)
        for m1 in LOCO_MODES:
            line = "%-14s" % m1[:13]
            for m2 in LOCO_MODES:
                if m1 == m2:
                    line += "%13s" % "-"
                else:
                    _, v = best_over(bank, m1, m2, IMU_AVAILABLE, metric)
                    line += "%13.2f" % v
            print(line)

    print("\n[판정 기준] d>=2.0 이면 실용적 분리 가능, 1.0~2.0 은 취약, <1.0 은 사실상 불가")

    # ---------------- (2) 쌍별 상세: 최강 feature ----------------
    print("\n\n###### (2) 쌍별 최강 feature — IMU만 vs IMU+EMG전체 vs 전체41 ######")
    print("%-26s %-24s %6s  %-26s %6s  %-24s %6s"
          % ("PAIR", "IMU가용 최강", "d_subj", "IMU+EMG 최강", "d_subj", "전체41 최강", "d_subj"))
    print("-" * 132)
    weak_pairs = []
    for (a, b) in pairs:
        fi, vi = best_over(bank, a, b, IMU_AVAILABLE, "subj")
        fe, ve = best_over(bank, a, b, IMU_AVAILABLE + emg_feats(MUSCLES), "subj")
        fa, va = best_over(bank, a, b, IMU_AVAILABLE + IMU_UNAVAILABLE + emg_feats(MUSCLES), "subj")
        print("%-26s %-24s %6.2f  %-26s %6.2f  %-24s %6.2f"
              % (f"{a}/{b}", fi, vi, fe, ve, fa, va))
        if ve < 2.0:
            weak_pairs.append((a, b, ve))
    print("-" * 132)
    if weak_pairs:
        print("취약 쌍 (IMU+EMG 최강 d_subj < 2.0):")
        for a, b, v in sorted(weak_pairs, key=lambda x: x[2]):
            print("   %-28s d_subj=%.2f" % (f"{a} vs {b}", v))

    # ---------------- (3) shank/foot 대체 손실 ----------------
    print("\n\n###### (3) shank/foot 복원불가로 인한 손실 (쌍별) ######")
    print("%-26s %8s %8s %8s   %s" % ("PAIR", "전체41", "H10가용", "손실%", "1위가 shank/foot?"))
    print("-" * 92)
    tot_loss = []
    for (a, b) in pairs:
        fa, va = best_over(bank, a, b, IMU_AVAILABLE + IMU_UNAVAILABLE + emg_feats(MUSCLES), "subj")
        fe, ve = best_over(bank, a, b, IMU_AVAILABLE + emg_feats(MUSCLES), "subj")
        loss = (1 - ve / max(va, EPS)) * 100
        tot_loss.append(loss)
        print("%-26s %8.2f %8.2f %7.1f%%   %s"
              % (f"{a}/{b}", va, ve, loss, "YES  (" + fa + ")" if fa in IMU_UNAVAILABLE else "no"))
    print("-" * 92)
    print("평균 손실 = %.1f%%" % np.mean(tot_loss))

    # ---------------- (4) EMG 근육 자리 greedy 선택 ----------------
    print("\n\n###### (4) EMG 근육 자리 선택 (greedy, 6모드 15쌍 기준) ######")
    print("목적함수: sum over 15 pairs of (IMU+선택근육 최강 d_subj)")

    def score(muscles):
        pool = IMU_AVAILABLE + emg_feats(muscles)
        return sum(best_over(bank, a, b, pool, "subj")[1] for (a, b) in pairs)

    def min_pair(muscles):
        pool = IMU_AVAILABLE + emg_feats(muscles)
        return min(best_over(bank, a, b, pool, "subj")[1] for (a, b) in pairs)

    base = score([])
    print("\n  0자리 (IMU only)          합계=%7.2f   최약쌍 d=%.2f" % (base, min_pair([])))
    chosen = []
    for slot in range(1, 4):
        best_m, best_s = None, -1
        for m in MUSCLES:
            if m in chosen:
                continue
            s = score(chosen + [m])
            if s > best_s:
                best_s, best_m = s, m
        chosen.append(best_m)
        print("  %d자리 + %-22s 합계=%7.2f   최약쌍 d=%.2f   (증가 +%.2f)"
              % (slot, best_m, best_s, min_pair(chosen), best_s - base))
        base = best_s

    print("\n  [참고] 최약쌍 기준 greedy (합계 대신 최소값 최대화):")
    chosen2 = []
    for slot in range(1, 4):
        best_m, best_s = None, -1
        for m in MUSCLES:
            if m in chosen2:
                continue
            s = min_pair(chosen2 + [m])
            if s > best_s:
                best_s, best_m = s, m
        chosen2.append(best_m)
        print("     %d자리 + %-22s 최약쌍 d=%.2f" % (slot, best_m, best_s))

    # ---------------- (5) 근육별 단독 기여 ----------------
    print("\n\n###### (5) 근육별 단독 기여도 (IMU 대비 개선) ######")
    gains = []
    for m in MUSCLES:
        g = score([m]) - score([])
        wins = sum(1 for (a, b) in pairs
                   if best_over(bank, a, b, IMU_AVAILABLE + emg_feats([m]), "subj")[1]
                   > best_over(bank, a, b, IMU_AVAILABLE, "subj")[1] + 1e-6)
        gains.append((m, g, wins))
    print("%-24s %10s %10s" % ("MUSCLE", "총 gain", "이긴 쌍/15"))
    print("-" * 48)
    for m, g, w in sorted(gains, key=lambda x: -x[1]):
        print("%-24s %10.2f %10d" % (m, g, w))

    # ---------------- (6) TURN 집중 분석 ----------------
    print("\n\n###### (6) TURN 집중 분석 (브리핑상 최약체) ######")
    for other in ["WALK", "RAMP_UP", "RAMP_DOWN", "STAIRS_UP", "STAIRS_DOWN"]:
        allf = IMU_AVAILABLE + IMU_UNAVAILABLE + emg_feats(MUSCLES)
        rows = sorted(((f, bank.d_subj("TURN", other, f), bank.d_pop("TURN", other, f))
                       for f in allf), key=lambda r: -r[1])
        print("\n  TURN vs %s — 상위 6" % other)
        for f, ds, dp in rows[:6]:
            mark = "  [HW없음]" if f in IMU_UNAVAILABLE else ""
            print("      %-34s d_subj=%5.2f  d_pop=%5.2f%s" % (f, ds, dp, mark))

    # ---------------- (7) 최종 export feature 선정 ----------------
    print("\n\n###### (7) C export 대상 feature 선정 ######")
    print("각 쌍의 상위 2개 feature를 모아 사용빈도로 정렬 (IMU + 선택근육 %s)" % chosen[:2])
    pool = IMU_AVAILABLE + emg_feats(chosen[:2])
    votes = defaultdict(float)
    for (a, b) in pairs:
        rows = sorted(((f, bank.d_subj(a, b, f)) for f in pool), key=lambda r: -r[1])
        for f, v in rows[:2]:
            votes[f] += v
    print("\n%-34s %10s" % ("FEATURE", "누적 d"))
    print("-" * 46)
    sel = sorted(votes.items(), key=lambda x: -x[1])
    for f, v in sel:
        print("%-34s %10.2f" % (f, v))
    print("-" * 46)
    n_feat = len(sel)
    print("\n선정 feature 수 = %d" % n_feat)
    print("메모리: 6모드 x %d feature x 101점 x (mean+std) x 4B = %.1f KB"
          % (n_feat, 6 * n_feat * 101 * 2 * 4 / 1024))


if __name__ == "__main__":
    main()
