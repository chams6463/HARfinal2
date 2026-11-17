"""
extend_C_anthropometric_normalization.py
===============================================================================
확장 [C] 개인차(체형) 정규화 — "지금 kRcStd 는 7명 pooled라 피험자간 분산까지
포함해서 보수적"이라고 ref_curve_tables.h 에 명시돼 있다. 그 피험자간 분산 중
일부가 키(Height)/체중(Weight) 같은 체형 차이에서 온다면, 사용자 키를 알고
(BTN 설정 등으로 입력받아) feature를 정규화해서 pooled std를 줄이고 z-score를
더 민감하게(=더 잘 구분하게) 만들 수 있을지 데이터로 검증한다.

절차
----
1) SubjectInfo.mat (Height/Weight, 12명 전원) 과 h_d_subject_12.mat 의 WALK
   평균곡선으로 "그 사람 WALK RMS 진폭"을 4개 feature 각각에 대해 계산.
2) 진폭 vs Height, 진폭 vs Weight 의 Pearson r 계산 (12명).
3) |r_height| 가 임계치(0.4) 이상인 feature에 대해서만: 원본 7명(AB06~12)
   으로 "진폭 = a + b*Height" 선형회귀를 적합 (신규 5명은 애초에 설계 시점에
   존재하지 않았다고 가정 — 회귀는 반드시 원본 7명 데이터만으로 학습해야
   "새 유저 키를 넣으면 예측 가능"이라는 실제 사용 시나리오와 맞음).
4) 이 회귀선으로 모든 12명 각자의 정규화 계수 k_subj = 예측진폭(Height_subj) /
   예측진폭(Height_평균) 을 구하고, 그 사람의 모든 곡선(원시 stride 포함)을
   x_norm = x / k_subj 로 나눈다.
5) 정규화 적용 뱅크 vs 미적용(기존) 뱅크를 각각 원본 7명으로 만들고, 신규
   5명 held-out stride 분류 정확도를 검증[1]과 동일한 방식으로 비교한다.
   -> 실제로 정확도가 오르는지가 "이 확장이 구현할 가치가 있는가"의 최종 근거.

주의: firmware 에 이걸 넣으려면 BTN 설정 단계에서 사용자 키 입력을 받는
UI/저장공간이 추가로 필요하다(현재 설계엔 없음) — 그 비용 대비 정확도 개선이
작으면 안 하는 게 맞다. 이 스크립트의 결론이 그 판단 근거가 된다.

실행: python extend_C_anthropometric_normalization.py
===============================================================================
"""

import numpy as np
from camargo_common import (
    MAT_PATH_7, MAT_PATH_12, STRIDE_BANK_12,
    ORIGINAL_SUBJECTS, NEW_SUBJECTS, BANK_MODES, FEAT_TAGS, BANK_FEATURES,
    load_population, load_stride_bank, load_subject_info,
    curve, pooled_mean_std, own_walk_curves_for, estimate_calibration,
    classify_curves, confusion_matrix, print_confusion_matrix,
)

R_THRESHOLD = 0.40


def rms_amplitude(curve_vals):
    return float(np.sqrt(np.mean(np.asarray(curve_vals, dtype=float) ** 2)))


def pearson_r(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    return float(np.corrcoef(x, y)[0, 1])


def build_bank_normalized(h_d_subject, subject_ids, scale_map, norm_feat_tags, modes=BANK_MODES,
                           std_floor_frac=0.01):
    """build_bank_from_subjects()와 동일하되, norm_feat_tags 에 속한 feature는
    각 subject 곡선을 scale_map[subj] 로 나눈 뒤 pooled."""
    bank = {}
    for mode in modes:
        bank[mode] = {}
        for src_name, tag, _w in BANK_FEATURES:
            stats = []
            for sid in subject_ids:
                sc = getattr(h_d_subject, sid, None)
                if sc is None:
                    continue
                c = curve(sc, mode, src_name)
                if c is None:
                    continue
                mu, sd, n = c
                if tag in norm_feat_tags:
                    k = scale_map[sid]
                    mu, sd = mu / k, sd / k
                stats.append((mu, sd, n))
            mu_p, sd_p = pooled_mean_std(stats)
            floor_val = max(std_floor_frac * np.max(np.abs(mu_p)), 1e-6)
            sd_p = np.maximum(sd_p, floor_val)
            bank[mode][tag] = (mu_p, sd_p)
    return bank


def held_out_accuracy(bank, sub12, stride_bank, scale_map, norm_feat_tags, subjects):
    """검증[1]과 동일한 절차. scale_map/norm_feat_tags 가 주어지면 own WALK curve와
    개별 stride 모두 같은 배수로 나눠 정규화한 뒤 분류한다."""
    all_true, all_pred = [], []
    for subj_id in subjects:
        own_walk = own_walk_curves_for(sub12, subj_id)
        if own_walk is None:
            continue
        if scale_map is not None:
            k = scale_map[subj_id]
            own_walk = {t: (v / k if t in norm_feat_tags else v) for t, v in own_walk.items()}

        phase_shift, calib = estimate_calibration(own_walk, bank["WALK"])
        if subj_id not in stride_bank:
            continue
        for mode in BANK_MODES:
            if mode not in stride_bank[subj_id]:
                continue
            feats = stride_bank[subj_id][mode]
            tags = list(calib.keys())
            if not all(t in feats for t in tags):
                continue
            n_strides = feats[tags[0]].shape[0]
            for i in range(n_strides):
                x_dict = {}
                for t in tags:
                    v = feats[t][i]
                    if scale_map is not None and t in norm_feat_tags:
                        v = v / scale_map[subj_id]
                    x_dict[t] = v
                _, pred = classify_curves(x_dict, bank, phase_shift, calib)
                all_true.append(mode)
                all_pred.append(pred)
    return all_true, all_pred


def main():
    print("=" * 100)
    print(" 확장[C] 체형(키/체중) 정규화 — pooled std를 줄여 z-score 민감도를 올릴 수 있는가")
    print("=" * 100)

    info = load_subject_info()
    _, sub12, _ = load_population(MAT_PATH_12)

    # ---- 1) 진폭 vs Height/Weight 상관 ----
    print("\n###### (1) feature별 WALK RMS 진폭 vs Height/Weight 상관 (12명) ######")
    heights = np.array([info[s]["height"] for s in ORIGINAL_SUBJECTS + NEW_SUBJECTS])
    weights = np.array([info[s]["weight"] for s in ORIGINAL_SUBJECTS + NEW_SUBJECTS])

    amp_by_feat = {}
    for src_name, tag, _w in BANK_FEATURES:
        amps = []
        for sid in ORIGINAL_SUBJECTS + NEW_SUBJECTS:
            sc = getattr(sub12, sid)
            c = curve(sc, "WALK", src_name)
            amps.append(rms_amplitude(c[0]) if c else np.nan)
        amp_by_feat[tag] = np.array(amps)

    print("%-16s %10s %10s" % ("FEATURE", "r(Height)", "r(Weight)"))
    print("-" * 40)
    r_height = {}
    for tag in FEAT_TAGS:
        rh = pearson_r(heights, amp_by_feat[tag])
        rw = pearson_r(weights, amp_by_feat[tag])
        r_height[tag] = rh
        flag = "  <- 정규화 후보" if abs(rh) >= R_THRESHOLD else ""
        print("%-16s %10.3f %10.3f%s" % (tag, rh, rw, flag))

    norm_feats = [t for t in FEAT_TAGS if abs(r_height[t]) >= R_THRESHOLD]
    if not norm_feats:
        print(f"\n|r(Height)| >= {R_THRESHOLD} 인 feature 없음 -> 키 정규화로 얻을 게 거의 없음. "
              "여기서 결론 내려도 됨: 확장C는 비용 대비 효과 없음.")
        return
    print(f"\n정규화 대상 feature: {norm_feats}")

    # ---- 2) 원본 7명으로만 회귀선 적합 ----
    print("\n###### (2) 원본 7명으로 진폭~Height 선형회귀 적합 ######")
    orig_heights = np.array([info[s]["height"] for s in ORIGINAL_SUBJECTS])
    scale_map = {}
    fits = {}
    for tag in norm_feats:
        orig_amps = np.array([amp_by_feat[tag][i] for i, s in enumerate(ORIGINAL_SUBJECTS + NEW_SUBJECTS)
                               if s in ORIGINAL_SUBJECTS])
        b, a = np.polyfit(orig_heights, orig_amps, 1)  # amp = a + b*height
        fits[tag] = (a, b)
        mean_h = float(np.mean(orig_heights))
        pred_mean = a + b * mean_h
        print(f"  {tag}: amp = {a:.5f} + {b:.5f}*height   (7명 기준 평균키={mean_h:.3f}m, "
              f"예측진폭={pred_mean:.5f})")

    for i, sid in enumerate(ORIGINAL_SUBJECTS + NEW_SUBJECTS):
        h = heights[i]
        # feature가 여러 개면 첫 정규화 feature 기준으로 통일된 스케일 하나만 쓰지 않고
        # feature별로 다른 k가 필요하므로, subject당 feature별 k를 별도 dict에 저장.
        pass

    # feature별 subject별 k 계산 (scale_map[tag][subj])
    scale_map_by_feat = {}
    for tag in norm_feats:
        a, b = fits[tag]
        mean_h = float(np.mean(orig_heights))
        pred_mean = a + b * mean_h
        d = {}
        for i, sid in enumerate(ORIGINAL_SUBJECTS + NEW_SUBJECTS):
            pred_h = a + b * heights[i]
            d[sid] = max(pred_h, 1e-6) / max(pred_mean, 1e-6)
        scale_map_by_feat[tag] = d

    print("\n  신규 5명의 정규화 계수 k_subj (1.0에서 멀수록 보정量 큼):")
    for tag in norm_feats:
        print(f"    [{tag}] " + ", ".join(f"{s}={scale_map_by_feat[tag][s]:.3f}" for s in NEW_SUBJECTS))

    # 단일 feature 케이스로 단순화 (여러 개면 첫 feature만 적용 + 경고)
    if len(norm_feats) > 1:
        print(f"\n  [참고] 정규화 후보가 {len(norm_feats)}개 -> 이 스크립트는 첫 번째({norm_feats[0]})만 "
              f"적용해 비교한다 (동시 다중 정규화는 상호작용 검증이 더 필요해 스코프 밖).")
    target_tag = norm_feats[0]
    scale_map = scale_map_by_feat[target_tag]

    # ---- 3) held-out 정확도 비교: 정규화 적용 vs 미적용 ----
    print(f"\n###### (3) held-out(신규 5명) 정확도 비교 — '{target_tag}' 정규화 적용 vs 미적용 ######")

    _, sub7, _ = load_population(MAT_PATH_7)
    stride_bank = load_stride_bank(STRIDE_BANK_12)

    bank_baseline = build_bank_normalized(sub7, ORIGINAL_SUBJECTS, scale_map={s: 1.0 for s in ORIGINAL_SUBJECTS},
                                           norm_feat_tags=[])
    bank_norm = build_bank_normalized(sub7, ORIGINAL_SUBJECTS, scale_map=scale_map, norm_feat_tags=[target_tag])

    t_base, p_base = held_out_accuracy(bank_baseline, sub12, stride_bank, None, [], NEW_SUBJECTS)
    t_norm, p_norm = held_out_accuracy(bank_norm, sub12, stride_bank, scale_map, [target_tag], NEW_SUBJECTS)

    print_confusion_matrix(confusion_matrix(t_base, p_base), title="[미적용/baseline] 신규 5명 held-out")
    print_confusion_matrix(confusion_matrix(t_norm, p_norm), title=f"[{target_tag} 키정규화 적용] 신규 5명 held-out")

    acc_base = np.trace(confusion_matrix(t_base, p_base)) / max(len(t_base), 1) * 100
    acc_norm = np.trace(confusion_matrix(t_norm, p_norm)) / max(len(t_norm), 1) * 100

    print("\n" + "=" * 100)
    print(f" 결론: baseline held-out 정확도 = {acc_base:.2f}%   "
          f"키정규화 적용 후 = {acc_norm:.2f}%   (변화 {acc_norm - acc_base:+.2f}%p)")
    if acc_norm - acc_base >= 2.0:
        print(" -> 의미 있는 개선. BTN 설정에 키 입력을 추가할 가치가 있어 보임 (UI/저장공간 비용 감수 가능).")
    elif acc_norm - acc_base <= -2.0:
        print(" -> 오히려 나빠짐. 12명 표본에서 회귀선이 신규 5명에 잘 안 맞았을 가능성 -> 채택 보류.")
    else:
        print(" -> 유의미한 차이 없음. 키 입력 UI/저장공간 비용을 들일 근거가 부족함 -> 채택 보류 권장.")
    print("=" * 100)


if __name__ == "__main__":
    main()
