"""
validate_2_template_stability.py
===============================================================================
검증 [2] 6명(AB11 제외 원본) 템플릿 vs 전체 held-out 포함 템플릿 비교
    "표본을 6명에서 훨씬 많은 인원으로 늘리면 kRcMean/kRcStd 가 얼마나
     흔들리는가? 늘려도 안 흔들리면(수렴) 지금 뱅크가 이미 안정적이라는 뜻이고,
     많이 흔들리면 표본이 더 필요하다는 뜻이다."

AB11 은 전극 문제 의심으로 두 그룹 모두에서 제외한다(뱅크 비교 자체가
"깨끗한 데이터로 만든 작은 템플릿 vs 깨끗한 데이터로 만든 큰 템플릿"이어야
공정하기 때문).

지표 (모드x feature 20칸 각각에 대해)
-----------------------------------
  shift_rms  : sqrt(mean_theta (mean_big-mean_small)^2) / rms(std_small)
  std_ratio  : rms(std_big) / rms(std_small)
  nStride    : 작은 그룹 -> 큰 그룹

실행: python validate_2_template_stability.py
===============================================================================
"""

import numpy as np
from camargo_common import (
    MAT_PATH_ALL, EXCLUDED_SUBJECTS, ORIGINAL_SUBJECTS, NEW_SUBJECTS,
    BANK_MODES, BANK_FEATURES,
    load_population, curve, build_bank_from_subjects,
)


def main():
    print("=" * 100)
    print(" 검증[2] 템플릿 안정성 — 원본 %d명 vs 전체 %d명 (AB11 제외 공통)" %
          (len(ORIGINAL_SUBJECTS), len(ORIGINAL_SUBJECTS) + len(NEW_SUBJECTS) - len(EXCLUDED_SUBJECTS)))
    print("=" * 100)

    _, sub_all, meta_all = load_population(MAT_PATH_ALL)

    small_group = list(ORIGINAL_SUBJECTS)
    full_group = [s for s in ORIGINAL_SUBJECTS + NEW_SUBJECTS if s not in EXCLUDED_SUBJECTS]

    print(f"\n작은 그룹({len(small_group)}명): {small_group}")
    print(f"큰 그룹({len(full_group)}명):   {full_group}")
    print(f"(제외: {EXCLUDED_SUBJECTS})")

    bank_small = build_bank_from_subjects(sub_all, small_group)
    bank_full = build_bank_from_subjects(sub_all, full_group)

    print("\n%-14s %-16s %10s %10s %10s %12s" %
          ("MODE", "FEATURE", "shift_rms", "std_small", "std_big", "std_ratio"))
    print("-" * 90)

    shifts, ratios = [], []
    for mode in BANK_MODES:
        for _src_name, tag, _w in BANK_FEATURES:
            mu_s, sd_s = bank_small[mode][tag]
            mu_b, sd_b = bank_full[mode][tag]

            shift_rms = float(np.sqrt(np.mean((mu_b - mu_s) ** 2)))
            std_s_rms = float(np.sqrt(np.mean(sd_s ** 2)))
            std_b_rms = float(np.sqrt(np.mean(sd_b ** 2)))
            norm_shift = shift_rms / max(std_s_rms, 1e-9)
            std_ratio = std_b_rms / max(std_s_rms, 1e-9)

            shifts.append(norm_shift)
            ratios.append(std_ratio)

            print("%-14s %-16s %10.3f %10.4f %10.4f %11.3fx" %
                  (mode, tag, norm_shift, std_s_rms, std_b_rms, std_ratio))

    print("-" * 90)
    print("\n[해석 기준]")
    print("  shift_rms (평균곡선 이동량 / 작은그룹 std) : <0.3=거의 안 움직임(수렴) / 0.3~0.7=다소 이동 / >0.7=많이 이동")
    print("  std_ratio (표본 늘렸을 때 분산 변화)         : ~1.0=안정 / >1.2=새 인원이 더 다양한 패턴 추가 / <0.8=오히려 좁아짐")

    print(f"\n요약: 평균 정규화 이동량 = {np.mean(shifts):.3f} (중앙값 {np.median(shifts):.3f}), "
          f"평균 std 비율 = {np.mean(ratios):.3f} (중앙값 {np.median(ratios):.3f})")
    if np.mean(shifts) < 0.3 and 0.85 < np.mean(ratios) < 1.3:
        print(f"=> 표본을 {len(small_group)}->{len(full_group)}명으로 늘려도 템플릿이 크게 안 변함. "
              "이미 꽤 수렴된 상태로 보임.")
    else:
        print("=> 신규 인원이 기존 템플릿과 다른 패턴을 유의미하게 가져옴. 표본을 늘리는 게 실제로 도움이 될 수 있음.")

    print("\n\n" + "=" * 60)
    print(" 참고: 모드별 총 stride 수 (작은 그룹 vs 큰 그룹)")
    print("=" * 60)
    print("%-14s %14s %14s" % ("MODE", "nStride(작음)", "nStride(큼)"))
    for mode in BANK_MODES:
        def total_n(group):
            n = 0
            for sid in group:
                sc = getattr(sub_all, sid, None)
                if sc is None:
                    continue
                c = curve(sc, mode, "IMU_thigh_GYROMAG")
                if c:
                    n += c[2]
            return n
        print("%-14s %14d %14d" % (mode, total_n(small_group), total_n(full_group)))


if __name__ == "__main__":
    main()
