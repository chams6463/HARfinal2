"""
analyze_muscle_ranking.py
===============================================================================
EMG 근육 11개 판별력 전체 랭킹 — AB11 제외 21명(배포용 뱅크와 동일 모집단).

analyze_locomotion_bank.py 의 (4)(5) 절을 최신 데이터로 다시 돌리되,
그때 없던 두 가지를 추가한다:
  - 근육별 최적 지표(MAV/RMS/ZC) 가 무엇인지
  - 1위 근육이 "어느 모드쌍에서" 실제로 이득을 만드는지

지표: 위상분해 Cohen's d 를 피험자별로 계산한 뒤 중앙값(d_subj).
      캘리브레이션으로 개인차를 흡수한 뒤의 기대 판별력에 해당한다.
목적함수: 보행 6모드 15개 쌍 각각에서 "IMU 가용 4개 + 선택근육"으로 얻는
      최고 d_subj 의 합계. 근육 추가로 이 합계가 얼마나 오르는지가 gain.
===============================================================================
"""

import numpy as np
from itertools import combinations
from camargo_common import load_population, curve, pooled_mean_std

MAT_PATH = r"D:\ifac_wearable_contest\Camargo_matlab\h_d_population_clean.mat"

LOCO_MODES = ["WALK", "TURN", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]
IMU_AVAILABLE = ["IMU_thigh_TILT", "IMU_thigh_GYROMAG",
                 "IMU_trunk_TILT", "IMU_trunk_GYROMAG"]
MUSCLES = ["gastrocmed", "tibialisanterior", "soleus", "vastusmedialis",
           "vastuslateralis", "rectusfemoris", "bicepsfemoris", "semitendinosus",
           "gracilis", "gluteusmedius", "rightexternaloblique"]
MUSCLE_KR = {
    "gastrocmed": "비복근 내측두",
    "tibialisanterior": "전경골근",
    "soleus": "가자미근",
    "vastusmedialis": "내측광근",
    "vastuslateralis": "외측광근",
    "rectusfemoris": "대퇴직근",
    "bicepsfemoris": "대퇴이두근",
    "semitendinosus": "반건양근",
    "gracilis": "박근",
    "gluteusmedius": "중둔근",
    "rightexternaloblique": "외복사근(우)",
}
EMG_METRICS = ["MAV", "RMS", "ZC"]
EPS = 1e-9


class Bank:
    def __init__(self, mat_path, subjects=None):
        _, self.sub, self.meta = load_population(mat_path)
        available = list(self.sub._fieldnames)
        self.subjects = subjects or available
        self._subj = {}

    def _d_rms(self, cA, cB):
        muA, sdA, _ = cA
        muB, sdB, _ = cB
        pooled = np.maximum(np.sqrt((sdA ** 2 + sdB ** 2) / 2.0), EPS)
        return float(np.sqrt(np.mean(((muB - muA) / pooled) ** 2)))

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
                    vals.append(self._d_rms(cA, cB))
            self._subj[k] = float(np.median(vals)) if vals else 0.0
        return self._subj[k]

    def best_over(self, mA, mB, feats):
        best_f, best_v = None, -1.0
        for f in feats:
            v = self.d_subj(mA, mB, f)
            if v > best_v:
                best_v, best_f = v, f
        return best_f, best_v


def emg_feats(muscles):
    return [f"EMG_{m}_{k}" for m in muscles for k in EMG_METRICS]


def main():
    bank = Bank(MAT_PATH)
    pairs = list(combinations(LOCO_MODES, 2))
    print("=" * 88)
    print(" EMG 근육 판별력 랭킹 — %d명 (AB11 제외), 보행 6모드 %d쌍" %
          (len(bank.subjects), len(pairs)))
    print(" 지표: 위상분해 Cohen's d 피험자별 중앙값 (캘리브레이션 후 기대치)")
    print("=" * 88)

    def score(muscles):
        pool = IMU_AVAILABLE + emg_feats(muscles)
        return sum(bank.best_over(a, b, pool)[1] for a, b in pairs)

    base = score([])
    print(f"\nIMU 4개만 사용했을 때 기준선(15쌍 d_subj 합계) = {base:.2f}")

    # ---------------- (1) 근육별 단독 기여 ----------------
    print("\n\n###### (1) 근육 하나만 추가했을 때의 단독 기여 ######")
    print("%-22s %-14s %9s %10s %12s" %
          ("MUSCLE", "한글", "총 gain", "이긴쌍/15", "최적지표"))
    print("-" * 88)
    rows = []
    for m in MUSCLES:
        g = score([m]) - base
        wins = 0
        metric_votes = {}
        for a, b in pairs:
            imu_best = bank.best_over(a, b, IMU_AVAILABLE)[1]
            f, v = bank.best_over(a, b, IMU_AVAILABLE + emg_feats([m]))
            if v > imu_best + 1e-6:
                wins += 1
                metric = f.rsplit("_", 1)[-1]
                metric_votes[metric] = metric_votes.get(metric, 0) + 1
        top_metric = max(metric_votes, key=metric_votes.get) if metric_votes else "—"
        rows.append((m, g, wins, top_metric))
    for m, g, wins, top_metric in sorted(rows, key=lambda r: -r[1]):
        print("%-22s %-14s %9.2f %10d %12s" %
              (m, MUSCLE_KR[m], g, wins, top_metric))

    # ---------------- (2) greedy 순차 선택 ----------------
    print("\n\n###### (2) greedy 순차 선택 (자리를 늘려가며) ######")
    print("이미 고른 근육들과 함께 썼을 때 합계를 가장 많이 올리는 근육을 순차 선택")
    print("%-6s %-22s %-14s %10s %10s" %
          ("자리", "MUSCLE", "한글", "이번 증가", "누적합계"))
    print("-" * 88)
    chosen, running = [], base
    for slot in range(1, 6):
        best_m, best_s = None, -1e9
        for m in MUSCLES:
            if m in chosen:
                continue
            s = score(chosen + [m])
            if s > best_s:
                best_s, best_m = s, m
        gain = best_s - running
        chosen.append(best_m)
        running = best_s
        print("%-6d %-22s %-14s %+10.2f %10.2f" %
              (slot, best_m, MUSCLE_KR[best_m], gain, running))

    # ---------------- (3) 1위 근육의 쌍별 활약처 ----------------
    top_muscle = sorted(rows, key=lambda r: -r[1])[0][0]
    print(f"\n\n###### (3) 1위 근육({top_muscle}) 은 어느 모드쌍에서 이기는가 ######")
    print("%-28s %10s %10s %9s   %s" %
          ("PAIR", "IMU만", "IMU+근육", "개선", "채택 feature"))
    print("-" * 88)
    detail = []
    for a, b in pairs:
        imu_f, imu_v = bank.best_over(a, b, IMU_AVAILABLE)
        f, v = bank.best_over(a, b, IMU_AVAILABLE + emg_feats([top_muscle]))
        detail.append((f"{a}/{b}", imu_v, v, v - imu_v, f))
    for name, iv, v, d, f in sorted(detail, key=lambda r: -r[3]):
        mark = f if d > 1e-6 else "(IMU 유지)"
        print("%-28s %10.2f %10.2f %+9.2f   %s" % (name, iv, v, d, mark))

    # ---------------- (4) 현재 채택된 2자리 재확인 ----------------
    print("\n\n###### (4) 현재 펌웨어가 쓰는 2자리와 비교 ######")
    current = ["gastrocmed", "vastusmedialis"]
    print(f"현재 채택: {current}  -> 합계 {score(current):.2f}")
    print(f"greedy 상위 2: {chosen[:2]}  -> 합계 {score(chosen[:2]):.2f}")
    if chosen[:2] != current:
        print("   ** greedy 결과가 현재 채택과 다르다 — 재검토 필요")
    else:
        print("   현재 채택과 일치")
    print(f"\n참고: 3자리까지 늘리면 {chosen[:3]} -> 합계 {score(chosen[:3]):.2f}")
    print(f"      (2자리 대비 +{score(chosen[:3]) - score(chosen[:2]):.2f})")


if __name__ == "__main__":
    main()
