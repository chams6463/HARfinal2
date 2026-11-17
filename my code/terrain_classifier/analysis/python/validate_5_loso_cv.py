"""
validate_5_loso_cv.py
===============================================================================
검증 [5] Leave-One-Subject-Out(LOSO) 교차검증.

검증[1]은 "원본 6명으로 만든 뱅크 -> held-out 전원에서 테스트"라는 표본
1개짜리 결과였다. 이건 통계적으로 약하다(그 사람들이 우연히 쉬웠을 수도,
어려웠을 수도 있음). LOSO는 "깨끗한" subject 각각을 돌아가며 "그 사람만
빼고 나머지로 뱅크를 만들고, 빠진 사람으로 테스트"를 반복해 평균/분산을
낸다 — 표준적인 소규모 데이터셋 일반화 성능 추정 방법이다.

AB11(전극 문제 의심)은 어느 fold의 학습에도 절대 쓰지 않는다 — 대신
"전체 깨끗한 pool로 만든 뱅크로 AB11 을 분류하면 어떻게 되는가"를 별도의
참고용 fold로 덧붙인다(제외 결정이 여전히 타당한지 계속 확인하기 위함).

각 fold:
  1) held-out 1명을 뺀 나머지 깨끗한 subject로 뱅크 구축
  2) held-out 사람의 WALK 평균곡선으로 캘리브레이션 추정(_FinishCalibration 재현)
  3) held-out 사람의 개별 stride(stride_bank_all)를 그 뱅크로 분류
  4) fold별 confusion matrix / 정확도 기록

실행: python validate_5_loso_cv.py  (fold마다 뱅크를 새로 짜야 해서 검증[1]보다 느림)
===============================================================================
"""

import numpy as np
from camargo_common import (
    MAT_PATH_ALL, STRIDE_BANK_ALL, ORIGINAL_SUBJECTS, NEW_SUBJECTS, EXCLUDED_SUBJECTS,
    BANK_MODES,
    load_population, load_stride_bank,
    build_bank_from_subjects, own_walk_curves_for, estimate_calibration,
    classify_curves, confusion_matrix, print_confusion_matrix,
)


def run_fold(sub_all, stride_bank, train_subjects, held_out):
    bank = build_bank_from_subjects(sub_all, train_subjects)
    own_walk = own_walk_curves_for(sub_all, held_out)
    if own_walk is None:
        print(f"  [SKIP fold] {held_out}: WALK 캘리브레이션 데이터 없음")
        return None
    phase_shift, calib = estimate_calibration(own_walk, bank["WALK"])

    if held_out not in stride_bank:
        print(f"  [SKIP fold] {held_out}: stride_bank_all 에 없음")
        return None

    true_labels, pred_labels = [], []
    for mode in BANK_MODES:
        if mode not in stride_bank[held_out]:
            continue
        feats = stride_bank[held_out][mode]
        tags = list(calib.keys())
        if not all(t in feats for t in tags):
            continue
        n_strides = feats[tags[0]].shape[0]
        for i in range(n_strides):
            x_dict = {t: feats[t][i] for t in tags}
            _, pred = classify_curves(x_dict, bank, phase_shift, calib)
            true_labels.append(mode)
            pred_labels.append(pred)

    if not true_labels:
        print(f"  [SKIP fold] {held_out}: 평가 가능한 stride 없음")
        return None

    cm = confusion_matrix(true_labels, pred_labels)
    acc = np.trace(cm) / max(cm.sum(), 1) * 100
    return true_labels, pred_labels, cm, acc, phase_shift


def main():
    clean_pool = [s for s in ORIGINAL_SUBJECTS + NEW_SUBJECTS if s not in EXCLUDED_SUBJECTS]

    print("=" * 100)
    print(f" 검증[5] LOSO 교차검증 — 깨끗한 {len(clean_pool)}명 대상 {len(clean_pool)}-fold "
          f"(+ 제외된 {EXCLUDED_SUBJECTS} 참고용 fold)")
    print("=" * 100)

    _, sub_all, _ = load_population(MAT_PATH_ALL)
    stride_bank = load_stride_bank(STRIDE_BANK_ALL)

    fold_results = []  # (subj, acc, n_strides, cm)
    pooled_true, pooled_pred = [], []

    for held_out in clean_pool:
        train_subjects = [s for s in clean_pool if s != held_out]
        r = run_fold(sub_all, stride_bank, train_subjects, held_out)
        if r is None:
            continue
        true_labels, pred_labels, cm, acc, phase_shift = r
        fold_results.append((held_out, acc, cm.sum(), cm))
        pooled_true += true_labels
        pooled_pred += pred_labels
        print(f"  fold [{held_out} held out, train on other {len(train_subjects)}]: "
              f"정확도 = {acc:5.1f}%  (n={cm.sum()} strides, phase_shift={phase_shift})")

    print("\n" + "-" * 100)
    print(" fold별 정확도 요약")
    print("-" * 100)
    accs = [r[1] for r in fold_results]
    print("%-8s %10s %10s" % ("SUBJ", "정확도", "n_stride"))
    for subj, acc, n, _ in sorted(fold_results, key=lambda r: r[1]):
        print("%-8s %9.1f%% %10d" % (subj, acc, n))
    print("-" * 100)
    print(f"{len(fold_results)}-fold 평균 정확도 = {np.mean(accs):.2f}%  "
          f"(표준편차 = {np.std(accs, ddof=1):.2f}%p, 최저 {np.min(accs):.1f}% / 최고 {np.max(accs):.1f}%)")

    print("\n" + "-" * 100)
    print(" 전체 pooled confusion matrix (깨끗한 그룹 모든 fold 합산)")
    print("-" * 100)
    cm_all = confusion_matrix(pooled_true, pooled_pred)
    overall = print_confusion_matrix(cm_all, title=f"LOSO 전체 ({len(fold_results)}-fold pooled, 깨끗한 그룹만)")

    print("\n" + "-" * 100)
    print(f" [참고 fold] 제외된 세션 {EXCLUDED_SUBJECTS} — 전체 깨끗한 pool로 만든 뱅크로 분류하면?")
    print("-" * 100)
    for excl in EXCLUDED_SUBJECTS:
        r = run_fold(sub_all, stride_bank, clean_pool, excl)
        if r is None:
            continue
        true_labels, pred_labels, cm, acc, phase_shift = r
        print_confusion_matrix(cm, title=f"{excl} (제외 세션 참고용)")
        print(f"  -> {excl} 정확도 = {acc:.1f}% (n={cm.sum()}) — 깨끗한 그룹 평균({np.mean(accs):.1f}%) 대비 "
              f"{'현저히 낮음: 제외 결정 재확인됨' if acc < np.mean(accs) - 15 else '큰 차이 없음: 제외 결정 재검토 필요'}")

    print("\n" + "=" * 100)
    print(f" 최종: LOSO 정확도 = {overall:.2f}% (fold 평균 {np.mean(accs):.2f}% ± {np.std(accs, ddof=1):.2f}%p)")
    print(" 검증[1](단일 분할)과 비교해서 fold간 표준편차가 크면,")
    print(" 검증[1] 결과가 우연히 좋거나 나빴을 가능성을 의심해야 한다.")
    print("=" * 100)


if __name__ == "__main__":
    main()
