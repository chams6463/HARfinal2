"""
validate_1_holdout_accuracy.py
===============================================================================
검증 [1] Held-out 정확도 재현 — 가장 직접적인 질문:
    "AB11 을 제외한 6명(AB06,07,08,09,10,12)으로 만든 뱅크가, 한 번도 본 적
     없는 나머지 사람들(AB11 포함 — 배제 효과 확인용 + AB13~21/28/30)의
     개별 stride를 얼마나 잘 분류하는가?"

AB11 은 gastrocmed(EMG_SITE0) 전극 접촉 불량이 의심되어 뱅크 학습에서
제외하기로 결정됨(AB11_AB28_outlier_analysis.md 참고). 학습에서는 빼되
held-out 풀에는 남겨서 "제외 결정이 실제로 타당했는지"를 계속 확인한다.

방법
----
1) 뱅크는 원본 6명(h_d_population_all.mat, AB11 제외)으로만 만든다.
2) 각 held-out subject 에 대해:
     a) 그 사람의 WALK 평균곡선을 "BTN1 캘리브레이션 데이터"로 간주 ->
        estimate_calibration() 으로 sign/scale/offset(+phase_shift) 추정
        (펌웨어 _FinishCalibration 재현)
     b) stride_bank_all.mat 에 있는 그 사람의 개별 stride(원시, 평균 안 낸 것)
        각각에 calibration을 적용하고 classify_curves()로 5모드 중 하나를
        예측 (펌웨어 _ClassifyStride 재현)
     c) confusion matrix / 정확도 집계 + 인구통계(나이/성별/키/체중) 병기
        -> 정확도가 특정 인구층에 쏠리는지 눈으로 바로 보기 위함
3) 비교 기준(상한선, in-sample sanity check): 원본 6명 각자에 대해서도
   똑같은 과정을 (자기 자신이 포함된 뱅크로) 돌려서, "완전히 알고 있는 사람"
   에서 나오는 낙관적 정확도를 참고치로 같이 보여준다.

주의: Camargo 데이터는 학습 subject/held-out subject 모두 동일 단위·동일
theta 원점(gcRight)이므로, H10 하드웨어 단위변환(rad/s vs deg/s 등)이나
MHE-vs-heelstrike 위상원점 불일치는 여기 없다. 즉 이 스크립트는 "기준곡선
자체가 새로운 사람에게 통계적으로 일반화되는가"만 순수하게 측정한다 —
"H10 실측 신호에서 캘리브레이션이 잘 되는가"는 검증[4]가 별도로 다룬다.
===============================================================================
"""

import numpy as np
from camargo_common import (
    MAT_PATH_ALL, STRIDE_BANK_ALL, EXCLUDED_SUBJECTS,
    ORIGINAL_SUBJECTS, NEW_SUBJECTS, BANK_MODES,
    load_population, load_stride_bank, load_subject_info,
    build_bank_from_subjects, own_walk_curves_for, estimate_calibration,
    classify_curves, confusion_matrix, print_confusion_matrix, circular_shift_distance,
)


def demo_str(info, sid):
    d = info.get(sid)
    if d is None:
        return "(인구통계 없음)"
    return f"{d['age']:.0f}세/{d['gender']}/{d['height']:.2f}m/{d['weight']:.1f}kg"


def evaluate_subject(subj_id, bank, h_d_subject, stride_bank, label):
    own_walk = own_walk_curves_for(h_d_subject, subj_id)
    if own_walk is None:
        print(f"  [SKIP] {subj_id}: WALK 캘리브레이션 데이터 없음")
        return None

    phase_shift, calib = estimate_calibration(own_walk, bank["WALK"])
    n_enabled = sum(1 for c in calib.values() if c["enabled"])
    circ = circular_shift_distance(phase_shift)
    print(f"  {subj_id} ({label}): phase_shift={phase_shift} (원형거리={circ}pt, "
          f"{'정상' if circ <= 5 else '!! 위상원점 불일치 의심'}), 활성 feature={n_enabled}/4")
    for tag, c in calib.items():
        print(f"      {tag:16s} sign={c['sign']:+.0f} scale={c['scale']:.4f} offset={c['offset']:.4f}")

    if subj_id not in stride_bank:
        print(f"  [SKIP] {subj_id}: stride_bank_all 에 데이터 없음")
        return None

    true_labels, pred_labels = [], []
    for mode in BANK_MODES:
        if mode not in stride_bank[subj_id]:
            continue
        feats = stride_bank[subj_id][mode]
        if not all(tag in feats for tag in calib.keys()):
            continue
        n_strides = feats[list(calib.keys())[0]].shape[0]
        for i in range(n_strides):
            x_dict = {tag: feats[tag][i] for tag in calib.keys()}
            _, pred = classify_curves(x_dict, bank, phase_shift, calib)
            true_labels.append(mode)
            pred_labels.append(pred)

    if not true_labels:
        print(f"  [SKIP] {subj_id}: 뱅크 5모드에 해당하는 stride 없음")
        return None

    return true_labels, pred_labels


def main():
    print("=" * 90)
    print(f" 검증[1] Held-out 정확도 — 뱅크({','.join(ORIGINAL_SUBJECTS)}) vs "
          f"held-out({','.join(NEW_SUBJECTS)})")
    print(f" (제외: {EXCLUDED_SUBJECTS} — 전극 문제 의심, 학습에서만 뺌)")
    print("=" * 90)

    pop_all, sub_all, meta_all = load_population(MAT_PATH_ALL)
    info = load_subject_info()
    print(f"\n전체 발견 subject: {list(meta_all.subjects)}")
    bank = build_bank_from_subjects(sub_all, ORIGINAL_SUBJECTS)
    stride_bank = load_stride_bank(STRIDE_BANK_ALL)

    print("\n" + "-" * 90)
    print(" [상한선/sanity check] 원본 6명 self-test (자신이 포함된 뱅크로 자신을 분류)")
    print("-" * 90)
    all_true, all_pred = [], []
    for sid in ORIGINAL_SUBJECTS:
        r = evaluate_subject(sid, bank, sub_all, stride_bank, "원본, in-sample")
        if r:
            all_true += r[0]
            all_pred += r[1]
    if all_true:
        cm = confusion_matrix(all_true, all_pred)
        print_confusion_matrix(cm, title="원본 6명 합산 (in-sample 상한선)")

    print("\n" + "-" * 90)
    print(" [진짜 held-out] 뱅크 학습에 전혀 안 쓰인 사람 (AB11 포함)")
    print("-" * 90)
    new_true, new_pred = [], []
    per_subject_acc = []
    for sid in NEW_SUBJECTS:
        label = "제외됨(전극의심), held-out으로 재평가" if sid in EXCLUDED_SUBJECTS else "신규 held-out"
        r = evaluate_subject(sid, bank, sub_all, stride_bank, label)
        if r:
            t, p = r
            new_true += t
            new_pred += p
            cm_s = confusion_matrix(t, p)
            acc_s = np.trace(cm_s) / max(cm_s.sum(), 1) * 100
            per_subject_acc.append((sid, acc_s, cm_s.sum()))

    print("\n피험자별 정확도 (인구통계 병기):")
    for sid, acc, n in sorted(per_subject_acc, key=lambda r: r[1]):
        flag = "  <- 제외된 세션(참고용)" if sid in EXCLUDED_SUBJECTS else ""
        print(f"  {sid}: {acc:5.1f}%  (n={n} strides)  {demo_str(info, sid)}{flag}")

    if new_true:
        cm = confusion_matrix(new_true, new_pred)
        overall = print_confusion_matrix(cm, title="held-out 전체 합산 (AB11 포함)")

        # AB11 제외한 "진짜 신규" 그룹만 따로도 집계 (더 공정한 일반화 지표)
        clean_new = [s for s in NEW_SUBJECTS if s not in EXCLUDED_SUBJECTS]
        # per_subject_acc 로는 true/pred 개별 리스트를 못 뽑으므로 다시 계산
        clean_true, clean_pred = [], []
        for sid in clean_new:
            r = evaluate_subject(sid, bank, sub_all, stride_bank, "재집계")
            if r:
                clean_true += r[0]
                clean_pred += r[1]

        print("\n" + "-" * 90)
        cm_clean = confusion_matrix(clean_true, clean_pred)
        overall_clean = print_confusion_matrix(cm_clean, title="held-out (AB11 제외한 '진짜 신규'만)")

        print("\n" + "=" * 90)
        in_sample_overall = np.trace(confusion_matrix(all_true, all_pred)) / max(len(all_true), 1) * 100
        print(f" 요약: in-sample 상한선 = {in_sample_overall:.1f}%   "
              f"held-out(AB11포함) = {overall:.1f}%   held-out(AB11제외) = {overall_clean:.1f}%")
        gap = in_sample_overall - overall_clean
        print(f"       gap(AB11제외 기준) = {gap:.1f}%p", end="  ")
        if gap < 10:
            print("-> 일반화 잘 됨. 뱅크가 특정 6명에 과적합되지 않았다.")
        elif gap < 25:
            print("-> 어느 정도 일반화되나, 신규 인원에서 성능 저하가 관찰됨.")
        else:
            print("-> 상당한 과적합 신호. 6명 템플릿이 새 사람에게 잘 안 맞을 수 있음.")
        print("=" * 90)
    else:
        print("\n[경고] held-out 그룹에서 평가 가능한 stride를 하나도 못 찾음.")


if __name__ == "__main__":
    main()
