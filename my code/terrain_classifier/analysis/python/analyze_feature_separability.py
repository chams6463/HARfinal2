"""
analyze_feature_separability.py
===============================================================================
h_d_population.mat (7 subjects, AB06~AB12) 기반 "위상분해(phase-resolved)
Cohen's d" 분석.

목적
-----
브리핑 4장의 Tier 표는 "전환 경계 전후 300ms 평균 비교"(3명, AB06/07/08)로
얻은 1차 스크리닝 결과다. 이 스크립트는 같은 질문을 (a) 7명 전원으로,
(b) 실제 아키텍처(z-score 뱅크는 각 theta에서 비교한다)와 동일한
위상분해 방식으로 다시 계산한다.

특히 shank/foot IMU feature(H10에 없음)를 thigh/trunk/EMG로 "대체"했을 때
판별력이 실제로 얼마나 떨어지는지를 숫자로 확인하는 것이 핵심 목적이다.

지표 정의
---------
mode A vs mode B, feature f, 위상 theta_i (i=0..100):

    d(theta_i) = (mean_B(theta_i) - mean_A(theta_i))
                 / sqrt( (std_A(theta_i)^2 + std_B(theta_i)^2) / 2 )

    d_rms = sqrt( mean_i d(theta_i)^2 )   <- 사이클 전체 평균 분리도 (주 지표)
    d_max = max_i |d(theta_i)|            <- 가장 잘 갈리는 위상에서의 분리도

두 가지 수준에서 계산한다:
  * d_pop  : h_d_population (7명 pooled). std에 피험자간 분산이 포함되므로
             "subject-independent 분류기가 실제로 겪는 난이도"에 해당. 보수적.
  * d_subj : h_d_subject 로 피험자별 계산 후 중앙값. std가 피험자내 분산이라
             값이 크게 나온다. 브리핑 Tier 표(d=7~12)와 같은 스케일.
  * n_agree: 7명 중 |d_rms|>=2.0 이고 부호가 중앙값과 같은 피험자 수.
             (Tier1 "subject-independent" 여부를 7명으로 재판정)

주의: A->B 와 B->A 는 |d|가 동일하고 부호만 뒤집힌다. 따라서 12개 전환이
아니라 "고유 모드쌍" 단위로 계산한다.

실행: python analyze_feature_separability.py
===============================================================================
"""

import numpy as np
import scipy.io as sio
from collections import defaultdict

MAT_PATH = r"C:\Users\user\Desktop\onephai_ifac\Camargo_matlab\h_d_population.mat"

# --- H10 하드웨어에서 복원 가능한 IMU feature ---
# thigh : leftThighAngle/rightThighAngle (CM이 이미 융합 제공)
# trunk : pelvicAngle
# GYROMAG : hip IMU 3축 자이로 크기 sqrt(Gx^2+Gy^2+Gz^2) (6축 전부 PDO 등록 확인)
#           또는 각도 수치미분. 둘 다 가능.
IMU_AVAILABLE = [
    "IMU_thigh_TILT",
    "IMU_thigh_GYROMAG",
    "IMU_trunk_TILT",
    "IMU_trunk_GYROMAG",
]
# 무릎/발목 엔코더가 없어 복원 불가 — 대체 대상
IMU_UNAVAILABLE = [
    "IMU_shank_TILT",
    "IMU_shank_GYROMAG",
    "IMU_foot_TILT",
    "IMU_foot_GYROMAG",
]

MUSCLES = [
    "gastrocmed", "tibialisanterior", "soleus", "vastusmedialis",
    "vastuslateralis", "rectusfemoris", "bicepsfemoris", "semitendinosus",
    "gracilis", "gluteusmedius", "rightexternaloblique",
]
EMG_METRICS = ["MAV", "RMS", "ZC"]
EMG_ALL = [f"EMG_{m}_{k}" for m in MUSCLES for k in EMG_METRICS]

MODES = ["STAND", "WALK", "TURN", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]

# 브리핑 5장 12개 전환 -> 고유 모드쌍
TRANSITIONS = [
    ("STAND_TO_STAIRUP",    "STAND", "STAIRS_UP"),
    ("STAIRUP_TO_STAND",    "STAIRS_UP", "STAND"),
    ("STAND_TO_STAIRDOWN",  "STAND", "STAIRS_DOWN"),
    ("STAIRDOWN_TO_STAND",  "STAIRS_DOWN", "STAND"),
    ("STAND_TO_RAMPUP",     "STAND", "RAMP_UP"),
    ("RAMPUP_TO_STAND",     "RAMP_UP", "STAND"),
    ("STAND_TO_RAMPDOWN",   "STAND", "RAMP_DOWN"),
    ("RAMPDOWN_TO_STAND",   "RAMP_DOWN", "STAND"),
    ("STAND_TO_WALK",       "STAND", "WALK"),
    ("WALK_TO_STAND",       "WALK", "STAND"),
    ("WALK_TO_TURN",        "WALK", "TURN"),
    ("TURN_TO_WALK",        "TURN", "WALK"),
]

EPS = 1e-9


def load():
    d = sio.loadmat(MAT_PATH, struct_as_record=False, squeeze_me=True)
    return d["h_d_population"], d["h_d_subject"], d["h_d_meta"]


def curve(container, mode, feat):
    """(mean[101], std[101]) 반환. 없으면 None."""
    ms = getattr(container, mode, None)
    if ms is None:
        return None
    fs = getattr(ms, feat, None)
    if fs is None:
        return None
    mu = np.atleast_1d(np.asarray(fs.mean, dtype=float))
    sd = np.atleast_1d(np.asarray(fs.std, dtype=float))
    if mu.size != 101 or sd.size != 101:
        return None
    if not (np.all(np.isfinite(mu)) and np.all(np.isfinite(sd))):
        return None
    return mu, sd


def cohens_d_curve(cA, cB):
    """위상별 Cohen's d 곡선 [101]. A->B 방향(증가면 +)."""
    muA, sdA = cA
    muB, sdB = cB
    pooled = np.sqrt((sdA ** 2 + sdB ** 2) / 2.0)
    pooled = np.maximum(pooled, EPS)
    return (muB - muA) / pooled


def summarize(dcurve):
    d_rms = float(np.sqrt(np.mean(dcurve ** 2)))
    imax = int(np.argmax(np.abs(dcurve)))
    d_max = float(dcurve[imax])
    # 부호: 사이클 평균 d의 부호 (증가/감소 방향)
    sign = 1.0 if float(np.mean(dcurve)) >= 0 else -1.0
    return d_rms, d_max, imax, sign


def analyze_pair(pop, sub, subjects, modeA, modeB, feats):
    """feature별 결과 dict 리스트 반환, d_pop_rms 내림차순."""
    rows = []
    for f in feats:
        cA = curve(pop, modeA, f)
        cB = curve(pop, modeB, f)
        if cA is None or cB is None:
            continue
        dc = cohens_d_curve(cA, cB)
        d_pop_rms, d_pop_max, imax, sign = summarize(dc)

        per_subj = []
        for s in subjects:
            sc = getattr(sub, s, None)
            if sc is None:
                continue
            sA = curve(sc, modeA, f)
            sB = curve(sc, modeB, f)
            if sA is None or sB is None:
                continue
            sdc = cohens_d_curve(sA, sB)
            r, _, _, sg = summarize(sdc)
            per_subj.append(r * sg)

        if per_subj:
            arr = np.array(per_subj)
            d_subj_med = float(np.median(np.abs(arr)))
            med_sign = 1.0 if np.median(arr) >= 0 else -1.0
            n_agree = int(np.sum((np.abs(arr) >= 2.0) & (np.sign(arr) == med_sign)))
        else:
            d_subj_med, n_agree = float("nan"), 0

        rows.append(dict(
            feat=f, d_pop_rms=d_pop_rms, d_pop_max=d_pop_max,
            theta_max=imax / 100.0, sign=sign,
            d_subj_med=d_subj_med, n_agree=n_agree,
            n_subj=len(per_subj),
        ))
    rows.sort(key=lambda r: -r["d_pop_rms"])
    return rows


def fmt_rows(rows, k=8, mark=None):
    out = []
    for r in rows[:k]:
        tag = ""
        if mark and r["feat"] in mark:
            tag = "  <== " + mark[r["feat"]]
        out.append(
            "    %-34s d_pop=%6.2f  d_max=%7.2f @th=%.2f  d_subj=%6.2f  agree=%d/%d  %s%s"
            % (r["feat"], r["d_pop_rms"], r["d_pop_max"], r["theta_max"],
               r["d_subj_med"], r["n_agree"], r["n_subj"],
               "UP" if r["sign"] > 0 else "DN", tag)
        )
    return "\n".join(out)


def main():
    pop, sub, meta = load()
    subjects = list(sub._fieldnames)

    print("=" * 79)
    print("위상분해 Cohen's d 분석 — 7 subjects:", ", ".join(subjects))
    print("d_pop = 7명 pooled(피험자간 분산 포함, 보수적) | d_subj = 피험자별 중앙값")
    print("agree = 7명 중 |d_rms|>=2 이고 부호 일치하는 수 (subject-independence)")
    print("=" * 79)

    all_feats = IMU_AVAILABLE + IMU_UNAVAILABLE + EMG_ALL

    # ---------------------------------------------------------------
    # (1) 전환별: 전체 랭킹 vs 하드웨어 가용 feature만의 랭킹
    # ---------------------------------------------------------------
    print("\n\n############ (1) 전환별 feature 랭킹 — 대체 손실 정량화 ############")
    loss_summary = []
    for name, mA, mB in TRANSITIONS:
        rows_all = analyze_pair(pop, sub, subjects, mA, mB, all_feats)
        avail = IMU_AVAILABLE + EMG_ALL
        rows_av = [r for r in rows_all if r["feat"] in avail]
        rows_imu_only = [r for r in rows_all if r["feat"] in IMU_AVAILABLE]

        best_all = rows_all[0] if rows_all else None
        best_av = rows_av[0] if rows_av else None
        best_imu = rows_imu_only[0] if rows_imu_only else None

        print("\n--- %s  (%s -> %s) ---" % (name, mA, mB))
        print("  [전체 41 feature 상위]")
        print(fmt_rows(rows_all, 5))
        print("  [H10 가용 feature만 (shank/foot 제외) 상위]")
        print(fmt_rows(rows_av, 5))
        if best_all and best_av:
            unavailable_won = best_all["feat"] in IMU_UNAVAILABLE
            loss = (1.0 - best_av["d_pop_rms"] / max(best_all["d_pop_rms"], EPS)) * 100.0
            print("  >> 최강=%s(%.2f) / 가용최강=%s(%.2f) / 손실=%.1f%%  %s"
                  % (best_all["feat"], best_all["d_pop_rms"],
                     best_av["feat"], best_av["d_pop_rms"], loss,
                     "[shank/foot가 1위 -> 대체 필요]" if unavailable_won else "[대체 불필요]"))
            loss_summary.append((name, best_all["feat"], best_all["d_pop_rms"],
                                 best_av["feat"], best_av["d_pop_rms"], loss,
                                 unavailable_won,
                                 best_imu["feat"] if best_imu else "-",
                                 best_imu["d_pop_rms"] if best_imu else float("nan")))

    print("\n\n############ (2) 대체 손실 요약표 ############")
    print("%-20s %-22s %7s  %-24s %7s %7s %s"
          % ("TRANSITION", "BEST(all41)", "d", "BEST(H10가용)", "d", "손실%", "IMU-only최강"))
    print("-" * 118)
    for (name, bf, bd, af, ad, loss, unav, imf, imd) in loss_summary:
        flag = "*" if unav else " "
        print("%-20s%s%-22s %7.2f  %-24s %7.2f %6.1f%%  %s(%.2f)"
              % (name, flag, bf, bd, af, ad, loss, imf, imd))
    print("-" * 118)
    print("* = shank/foot(복원불가)가 1위였던 전환 = 브리핑이 '대체 필요'로 지목한 항목")

    # ---------------------------------------------------------------
    # (3) EMG 근육 자리 선택 — 근육별 기여도 집계
    # ---------------------------------------------------------------
    print("\n\n############ (3) EMG 근육 자리 선택 (데이터 기반) ############")
    print("각 전환에서 'IMU 가용 feature만으로 얻는 최고 d' 대비, 그 근육을 추가했을 때")
    print("얻는 개선폭(gain)을 근육별로 집계. gain>0 인 전환 수와 총 gain을 본다.")

    muscle_gain = defaultdict(float)
    muscle_wins = defaultdict(int)
    muscle_detail = defaultdict(list)

    for name, mA, mB in TRANSITIONS:
        rows_all = analyze_pair(pop, sub, subjects, mA, mB, all_feats)
        by_feat = {r["feat"]: r for r in rows_all}
        imu_best = max([by_feat[f]["d_pop_rms"] for f in IMU_AVAILABLE if f in by_feat],
                       default=0.0)
        for mus in MUSCLES:
            feats = [f"EMG_{mus}_{k}" for k in EMG_METRICS]
            best = max([by_feat[f]["d_pop_rms"] for f in feats if f in by_feat], default=0.0)
            gain = best - imu_best
            if gain > 0:
                muscle_gain[mus] += gain
                muscle_wins[mus] += 1
                muscle_detail[mus].append((name, gain, best))

    print("\n%-22s %8s %8s   %s" % ("MUSCLE", "총gain", "이긴전환", "주요 기여 전환"))
    print("-" * 100)
    for mus in sorted(MUSCLES, key=lambda m: -muscle_gain[m]):
        det = sorted(muscle_detail[mus], key=lambda x: -x[1])[:3]
        ds = ", ".join("%s(+%.1f)" % (n, g) for n, g, _ in det)
        print("%-22s %8.2f %8d   %s" % (mus, muscle_gain[mus], muscle_wins[mus], ds))
    print("-" * 100)

    # 누적 자리 선택 (greedy): 1~3자리
    print("\n[Greedy 자리 선택] — 이미 고른 근육 + IMU 대비 추가 개선이 가장 큰 근육을 순차 선택")
    chosen = []
    for slot in range(1, 4):
        best_mus, best_total = None, -1.0
        for mus in MUSCLES:
            if mus in chosen:
                continue
            cand = chosen + [mus]
            total = 0.0
            for name, mA, mB in TRANSITIONS:
                rows_all = analyze_pair(pop, sub, subjects, mA, mB, all_feats)
                by_feat = {r["feat"]: r for r in rows_all}
                pool = list(IMU_AVAILABLE) + [f"EMG_{m}_{k}" for m in cand for k in EMG_METRICS]
                total += max([by_feat[f]["d_pop_rms"] for f in pool if f in by_feat], default=0.0)
            if total > best_total:
                best_total, best_mus = total, mus
        chosen.append(best_mus)
        print("  %d자리: + %-20s  (12전환 d 합계 = %.2f)" % (slot, best_mus, best_total))

    # IMU only baseline
    total_imu = 0.0
    for name, mA, mB in TRANSITIONS:
        rows_all = analyze_pair(pop, sub, subjects, mA, mB, all_feats)
        by_feat = {r["feat"]: r for r in rows_all}
        total_imu += max([by_feat[f]["d_pop_rms"] for f in IMU_AVAILABLE if f in by_feat],
                         default=0.0)
    print("  0자리(IMU only) 기준선                (12전환 d 합계 = %.2f)" % total_imu)

    # ---------------------------------------------------------------
    # (4) trunk vs thigh GYROMAG — hip IMU 장착 위치 모호성이 문제되는가
    # ---------------------------------------------------------------
    print("\n\n############ (4) trunk_GYROMAG vs thigh_GYROMAG 판별력 비교 ############")
    print("H10 hip IMU가 골반 고정인지 허벅지 연동인지 코드만으론 확정 불가.")
    print("두 feature의 판별력이 비슷하면 이 모호성은 실질적 위험이 아니다.")
    print("\n%-20s %10s %10s %8s" % ("TRANSITION", "trunk_GM", "thigh_GM", "차이"))
    print("-" * 52)
    for name, mA, mB in TRANSITIONS:
        rows = analyze_pair(pop, sub, subjects, mA, mB,
                            ["IMU_trunk_GYROMAG", "IMU_thigh_GYROMAG"])
        by = {r["feat"]: r["d_pop_rms"] for r in rows}
        t = by.get("IMU_trunk_GYROMAG", float("nan"))
        h = by.get("IMU_thigh_GYROMAG", float("nan"))
        print("%-20s %10.2f %10.2f %8.2f" % (name, t, h, h - t))
    print("-" * 52)

    # ---------------------------------------------------------------
    # (5) 7모드 전체 쌍별 분리도 (IMU 가용 feature만) — 뱅크 구조의 실제 난이도
    # ---------------------------------------------------------------
    print("\n\n############ (5) 7모드 쌍별 분리도 (H10 IMU 4 feature 중 최고 d_pop) ############")
    print("z-score 뱅크는 7모드를 동시에 비교하므로, 모든 쌍이 갈려야 한다.")
    hdr = "%-13s" % "" + "".join("%12s" % m[:11] for m in MODES)
    print(hdr)
    for i, m1 in enumerate(MODES):
        line = "%-13s" % m1[:12]
        for j, m2 in enumerate(MODES):
            if i == j:
                line += "%12s" % "-"
                continue
            rows = analyze_pair(pop, sub, subjects, m1, m2, IMU_AVAILABLE)
            best = rows[0]["d_pop_rms"] if rows else float("nan")
            line += "%12.2f" % best
        print(line)
    print("\n(값이 낮은 쌍 = IMU만으로는 구분 불가 -> EMG 또는 문맥/전이제약 필요)")


if __name__ == "__main__":
    main()
