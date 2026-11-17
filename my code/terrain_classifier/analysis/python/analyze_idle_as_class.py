"""
analyze_idle_as_class.py
===============================================================================
질문: Camargo 의 'idle' 을 "정지(STAND)"가 아니라 "전이/재배치(IDLE)" 라는
      별도 클래스로 재정의해서 살릴 수 있는가?

배경: diag_stand_curves2.m 결과 idle 은 median |w| = 0.923 rad/s 로 정지가
      아니었고, run 길이가 0.09~9.51 s (100:1) 라 위상정렬이 무의미했다.
      => "STAND 기준곡선"으로는 못 쓴다는 결론이었다.
      그러나 idle 을 "보행도 정지도 아닌 중간 상태"로 재정의하면 얘기가 다르다.

이 스크립트가 확인하는 것:
 (1) h_d^STAND(theta) 가 실제로 위상 구조가 없는가(flat 한가)?
     flatness = 위상간 분산 / 전체 분산.
     WALK 등 보행모드는 크고, idle 은 작아야 "위상 없는 클래스"로 다뤄도
     잃는 정보가 없다는 뜻이 된다.
 (2) 위상을 버리고 '주변분포(marginal)' 로만 봤을 때 idle 이 5개 보행모드와
     구분되는가? (전체분산 = 위상내 분산 + 위상간 분산, 총분산의 법칙)
 (3) idle 을 6번째 클래스로 넣으면 기존 5모드 판별을 방해하는가?
     (idle 주변분포가 보행모드들과 얼마나 겹치는지)

실행: python analyze_idle_as_class.py
===============================================================================
"""

import numpy as np
import scipy.io as sio
from itertools import combinations

MAT_PATH = r"C:\Users\user\Desktop\onephai_ifac\Camargo_matlab\h_d_population.mat"

LOCO = ["WALK", "TURN", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]
ALL_MODES = ["STAND"] + LOCO

# 펌웨어가 실제로 쓰는 4 feature (h_d_tables.h 와 동일)
FEATS = [
    "IMU_trunk_TILT",
    "IMU_thigh_GYROMAG",
    "EMG_gastrocmed_RMS",
    "EMG_vastusmedialis_RMS",
]

EPS = 1e-12


def load():
    d = sio.loadmat(MAT_PATH, struct_as_record=False, squeeze_me=True)
    return d["h_d_population"], d["h_d_subject"]


def curve(container, mode, feat):
    o = getattr(getattr(container, mode), feat)
    return (np.asarray(o.mean, float), np.asarray(o.std, float))


def marginal(mu, sd):
    """위상을 적분해 없앤 주변분포의 (평균, 표준편차).

    총분산의 법칙:  Var = E_theta[ Var(theta) ] + Var_theta[ E(theta) ]
                        = 위상내 분산 평균  +  위상간(파형) 분산
    """
    m = float(np.mean(mu))
    within = float(np.mean(sd ** 2))
    between = float(np.mean((mu - m) ** 2))
    return m, np.sqrt(within + between), within, between


def cohens_d(m1, s1, m2, s2):
    p = np.sqrt((s1 ** 2 + s2 ** 2) / 2.0)
    return (m2 - m1) / max(p, EPS)


def main():
    pop, sub = load()
    subs = list(sub._fieldnames)

    print("=" * 78)
    print(" idle 을 'STAND' 대신 'IDLE(전이/재배치)' 클래스로 살릴 수 있는가")
    print("=" * 78)

    # ---------------- (1) 위상 구조 유무 ----------------
    print("\n###### (1) 위상 구조가 있는가 — flatness = 위상간분산 / 전체분산 ######")
    print("높을수록 파형(위상) 정보가 많다는 뜻. idle 이 낮으면 위상 없는 클래스로")
    print("다뤄도 잃는 게 없다 -> 101점 테이블 대신 스칼라 1쌍(mean,std)이면 충분.")
    print("\n%-14s" % "MODE" + "".join("%22s" % f.replace("IMU_", "").replace("EMG_", "")
                                       for f in FEATS))
    print("-" * 102)
    for m in ALL_MODES:
        line = "%-14s" % m
        for f in FEATS:
            mu, sd = curve(pop, m, f)
            _, _, within, between = marginal(mu, sd)
            frac = between / max(within + between, EPS)
            line += "%21.1f%%" % (frac * 100)
        print(line)
    print("-" * 102)

    # ---------------- (2) 주변분포 기반 분리도 ----------------
    print("\n\n###### (2) 위상 버리고 주변분포로만 봤을 때 idle vs 보행모드 ######")
    print("d_pop = 7명 pooled, d_subj = 피험자별 중앙값(캘리브레이션 후 기준)")

    def marg_pop(m, f):
        return marginal(*curve(pop, m, f))[:2]

    def marg_subj(s, m, f):
        sc = getattr(sub, s)
        return marginal(*curve(sc, m, f))[:2]

    print("\n%-38s %9s %9s" % ("PAIR (feature)", "d_pop", "d_subj"))
    print("-" * 60)
    best_per_pair = {}
    for other in LOCO:
        rows = []
        for f in FEATS:
            m1, s1 = marg_pop("STAND", f)
            m2, s2 = marg_pop(other, f)
            dp = abs(cohens_d(m1, s1, m2, s2))

            ds = []
            for s in subs:
                a1, b1 = marg_subj(s, "STAND", f)
                a2, b2 = marg_subj(s, other, f)
                ds.append(abs(cohens_d(a1, b1, a2, b2)))
            rows.append((f, dp, float(np.median(ds))))
        rows.sort(key=lambda r: -r[2])
        best_per_pair[other] = rows[0]
        for f, dp, ds in rows:
            tag = "  <= 최강" if (f, dp, ds) == rows[0] else ""
            print("%-38s %9.2f %9.2f%s" % ("IDLE/%s (%s)" % (other, f.replace("IMU_", "").replace("EMG_", "")), dp, ds, tag))
        print()

    print("-" * 60)
    print("\n[요약] IDLE vs 각 보행모드 최강 feature")
    print("%-16s %-26s %9s %9s" % ("상대 모드", "최강 feature", "d_pop", "d_subj"))
    print("-" * 64)
    worst = 1e9
    for other in LOCO:
        f, dp, ds = best_per_pair[other]
        worst = min(worst, ds)
        print("%-16s %-26s %9.2f %9.2f" % (other, f.replace("IMU_", "").replace("EMG_", ""), dp, ds))
    print("-" * 64)
    print("최약 쌍 d_subj = %.2f  (2.0 이상이면 실용, 1.0 미만이면 사실상 불가)" % worst)

    # ---------------- (3) 기존 5모드를 방해하는가 ----------------
    print("\n\n###### (3) IDLE 을 추가하면 기존 5모드 판별을 방해하는가 ######")
    print("각 보행모드 쌍의 분리도(주변분포 기준)와 비교해서, IDLE 이 끼어들 때")
    print("더 가까운 이웃이 되는지 본다.")

    BANK5 = ["WALK", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]
    print("\n%-30s %9s" % ("PAIR", "최강 d_subj"))
    print("-" * 42)
    loco_pairs = []
    for a, b in combinations(BANK5, 2):
        best = 0.0
        for f in FEATS:
            ds = []
            for s in subs:
                a1, b1 = marg_subj(s, a, f)
                a2, b2 = marg_subj(s, b, f)
                ds.append(abs(cohens_d(a1, b1, a2, b2)))
            best = max(best, float(np.median(ds)))
        loco_pairs.append(((a, b), best))
        print("%-30s %9.2f" % ("%s / %s" % (a, b), best))
    print("-" * 42)
    print("보행쌍 최소 = %.2f, 중앙값 = %.2f"
          % (min(v for _, v in loco_pairs), float(np.median([v for _, v in loco_pairs]))))

    idle_ds = [best_per_pair[o][2] for o in BANK5]
    print("IDLE-보행쌍 최소 = %.2f, 중앙값 = %.2f"
          % (min(idle_ds), float(np.median(idle_ds))))
    print("\n[판정] IDLE-보행 분리도가 보행-보행 분리도와 비슷하거나 크면,")
    print("       IDLE 을 6번째 클래스로 넣어도 기존 판별을 방해하지 않는다.")


if __name__ == "__main__":
    main()
