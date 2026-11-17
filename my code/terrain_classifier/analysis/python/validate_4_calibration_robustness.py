"""
validate_4_calibration_robustness.py
===============================================================================
검증 [4] BTN1 캘리브레이션(_FinishCalibration) 강건성 스트레스 테스트.

"새로운 사람이 왔을 때, 그 사람 WALK 데이터로 추정한 sign/scale/offset/
phase_shift 가 튀는 값 없이 안정적으로 나오는가? 아니면 특정 체형/보행
패턴에서 캘리브레이션 자체가 이상하게 깨지는가?"

방법
----
뱅크는 원본 7명(AB06~12)으로 고정. 12명 전원(원본 7 + 신규 5) 각각의
WALK 평균곡선을 "그 사람의 BTN1 데이터"라고 보고 estimate_calibration()
(펌웨어 _FinishCalibration 재현)을 돌려 phase_shift/sign/scale/offset을
추정한다.

- 원본 7명 중 한 명을 넣을 때는 "자기 자신도 들어있는 뱅크와 비교"이므로
  당연히 잘 맞을 가능성이 높다(참고용 기준 분포).
- 신규 5명은 뱅크에 전혀 없던 사람이므로, 이 사람들의 계수가 원본 7명의
  계수 분포(평균 ± 2*std) 밖으로 튀는지가 핵심 관심사 — 튀면 "이 캘리브레이션
  알고리즘이 특정 체형/보행패턴에서 불안정할 수 있다"는 신호.

실행: python validate_4_calibration_robustness.py
===============================================================================
"""

import numpy as np
from camargo_common import (
    MAT_PATH_ALL, EXCLUDED_SUBJECTS, ORIGINAL_SUBJECTS, NEW_SUBJECTS, FEAT_TAGS,
    load_population, build_bank_from_subjects, own_walk_curves_for, estimate_calibration,
    circular_shift_distance,
)


def main():
    print("=" * 100)
    print(f" 검증[4] 캘리브레이션(sign/scale/offset/phase_shift) 강건성 — 전체 subject")
    print(f" (뱅크 학습: {ORIGINAL_SUBJECTS} / 제외: {EXCLUDED_SUBJECTS})")
    print("=" * 100)

    _, sub_all, _ = load_population(MAT_PATH_ALL)
    bank = build_bank_from_subjects(sub_all, ORIGINAL_SUBJECTS)

    rows = []  # (subj, is_new, phase_shift, {tag: (sign,scale,offset)})
    for subj_id in ORIGINAL_SUBJECTS + NEW_SUBJECTS:
        own_walk = own_walk_curves_for(sub_all, subj_id)
        if own_walk is None:
            print(f"  [SKIP] {subj_id}: WALK 데이터 없음")
            continue
        phase_shift, calib = estimate_calibration(own_walk, bank["WALK"])
        rows.append((subj_id, subj_id in NEW_SUBJECTS, phase_shift, calib))

    # ---- phase_shift 표 ----
    print("\n%-8s %-10s %12s" % ("SUBJ", "구분", "phase_shift"))
    print("-" * 34)
    for subj_id, is_new, ps, _ in rows:
        tag = "신규" if is_new else "원본(in-bank)"
        circ = circular_shift_distance(ps)
        flag = "" if circ <= 5 else "  !! 원형거리 큼 (위상원점 안 맞음 - 데이터 이상 의심)"
        print("%-8s %-10s %12d%s (원형거리=%dpt)" % (subj_id, tag, ps, flag, circ))

    # ---- feature별 sign/scale/offset 표 + 원본7 분포 대비 이상치 탐지 ----
    for tag in FEAT_TAGS:
        print(f"\n\n###### feature = {tag} ######")
        print("%-8s %-14s %8s %10s %12s" % ("SUBJ", "구분", "sign", "scale", "offset"))
        print("-" * 56)

        orig_scales, orig_offsets = [], []
        for subj_id, is_new, _ps, calib in rows:
            if not is_new:
                orig_scales.append(calib[tag]["scale"])
                orig_offsets.append(calib[tag]["offset"])

        scale_mu, scale_sd = np.mean(orig_scales), np.std(orig_scales, ddof=1)
        offset_mu, offset_sd = np.mean(orig_offsets), np.std(orig_offsets, ddof=1)

        for subj_id, is_new, _ps, calib in rows:
            c = calib[tag]
            if not c["enabled"]:
                print("%-8s %-14s   [비활성 — 신호 평탄]" % (subj_id, "신규" if is_new else "원본"))
                continue
            z_scale = (c["scale"] - scale_mu) / max(scale_sd, 1e-9)
            z_offset = (c["offset"] - offset_mu) / max(offset_sd, 1e-9)
            flag = ""
            if is_new and (abs(z_scale) > 2 or abs(z_offset) > 2):
                flag = "  !! 원본7 분포 밖 (|z|>2) - 이상치"
            label = "신규" if is_new else "원본(in-bank)"
            print("%-8s %-14s %+8.0f %10.4f %12.4f%s" %
                  (subj_id, label, c["sign"], c["scale"], c["offset"], flag))

        print(f"  (원본7 기준: scale = {scale_mu:.4f} ± {scale_sd:.4f},  "
              f"offset = {offset_mu:.4f} ± {offset_sd:.4f})")

    print("\n" + "=" * 100)
    print(" 결론 판단: phase_shift 는 전원 원형거리 0에 가까워야 정상(Camargo 데이터는")
    print(" 원래 위상원점이 전부 gcRight 로 통일돼 있으므로). '!! 이상치' 표시가 신규 그룹")
    print(" 다수/특정 feature 에 몰려 있으면, 그 feature 의 캘리브레이션이 체형/개인차에")
    print(f" 민감하다는 뜻. (참고: {EXCLUDED_SUBJECTS} 는 뱅크에서 의도적으로 제외된 세션이라")
    print(" 여기서 이상치로 잡혀도 예상된 결과 — 오히려 안 잡히면 배제 근거를 재검토할 것.)")
    print("=" * 100)


if __name__ == "__main__":
    main()
