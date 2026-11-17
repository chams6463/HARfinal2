"""
camargo_common.py
===============================================================================
5개 검증 스크립트(validate_1~5)와 확장(extend_C)이 공유하는 유틸리티.

펌웨어(Locomotion_Mode_Classifier.c)의 다음 세 함수를 Python으로 그대로
재현한다 — 숫자가 다르면 "우리가 뭘 검증했는지"가 의미를 잃으므로, 계산식은
1:1로 맞춘다:
    _CurveStats()       -> curve_stats()
    _FinishCalibration() -> estimate_calibration()
    _ClassifyStride()   -> classify_curves()

주의: Camargo 원본 데이터(h_d_population*.mat, stride_bank_12.mat)는 학습에
쓰인 7명과 held-out 5명이 전부 "같은 좌표계/단위/theta 원점(gcRight
heel-strike)"으로 뽑혀 있다. 따라서 실제 H10 런타임과 달리 위상 origin
불일치(MHE vs heel-strike)는 애초에 존재하지 않는다 — phase_shift는 그래도
펌웨어와 동일하게 계산해서 실제로 ~0 이 나오는지 확인하는 용도로만 쓴다
(0이 아니면 데이터 파이프라인에 뭔가 문제가 있다는 신호).
===============================================================================
"""

import numpy as np
import scipy.io as sio

REPO_ROOT = r"D:\ifac_wearable_contest"
CAMARGO_DIR = r"D:\ifac_wearable_contest\Camargo_matlab"

MAT_PATH_7 = rf"{CAMARGO_DIR}\h_d_population.mat"        # (역사적 참고용) AB11 포함 원본 7명
MAT_PATH_12 = rf"{CAMARGO_DIR}\h_d_population_12.mat"    # (역사적 참고용) AB11 포함 12명
STRIDE_BANK_12 = rf"{CAMARGO_DIR}\stride_bank_12.mat"    # (역사적 참고용) AB11 포함 12명

MAT_PATH_ALL = rf"{CAMARGO_DIR}\h_d_population_all.mat"  # 현재 다운로드된 전체 subject
STRIDE_BANK_ALL = rf"{CAMARGO_DIR}\stride_bank_all.mat"  # (extract_all_subjects.m 이 자동 탐색해 생성)

SUBJECT_INFO = rf"{CAMARGO_DIR}\subject_info_export.mat"  # SubjectInfo.mat(MATLAB table)의
# scipy-호환 평문 배열 버전. MATLAB table 객체는 scipy.io 가 못 읽어서
# (MatlabOpaque) 아래 커맨드로 한 번 export 해둔 것 — 원본 SubjectInfo.mat 이
# 갱신되면 다시 실행해야 함:
#   matlab -batch "S=load('SubjectInfo.mat'); T=S.data; subjectId=T.Subject; \
#     age=double(T.Age); gender=T.Gender; height=double(T.Height); weight=double(T.Weight); \
#     save('subject_info_export.mat','subjectId','age','gender','height','weight');"

# AB11 은 뱅크 학습 대상에서 제외 — gastrocmed(EMG_SITE0) 전극 접촉 불량 의심
# (AB11_AB28_outlier_analysis.md 참고: 다른 3 feature는 정상인데 EMG_SITE0 만
# RAMP_DOWN/STAIRS_DOWN 어느 템플릿과도 안 맞을 만큼 절대거리가 크고, 캘리브레이션
# scale도 2.09x/2.34x로 원본 7명 중 가장 높은 축). 학습에는 안 쓰되, "제외했더니
# 실제로 어떻게 분류되는지"를 계속 보기 위해 held-out 풀(NEW_SUBJECTS)에는 남겨둔다.
EXCLUDED_SUBJECTS = ["AB11"]

ORIGINAL_SUBJECTS = ["AB06", "AB07", "AB08", "AB09", "AB10", "AB12"]  # 뱅크 학습(6명, AB11 제외)
NEW_SUBJECTS = ["AB11", "AB13", "AB14", "AB15", "AB16", "AB17", "AB18", "AB19", "AB20", "AB21",
                "AB28", "AB30"]  # held-out 테스트 풀 (AB11 포함 — 배제 효과 확인용)
ALL_12_SUBJECTS = ORIGINAL_SUBJECTS + NEW_SUBJECTS   # 이름은 남기지만 이제 12명이 아님(하위호환)
ALL_SUBJECTS = ALL_12_SUBJECTS

BANK_MODES = ["WALK", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"]
# 뱅크 feature: (h_d_population.mat 41-feature 이름, RcFeature_t 접미사, 펌웨어 가중치)
BANK_FEATURES = [
    ("IMU_trunk_TILT",         "TRUNK_TILT",     1.0),
    ("IMU_thigh_GYROMAG",      "THIGH_GYROMAG",  1.0),
    ("EMG_gastrocmed_RMS",     "EMG_SITE0_RMS",  0.8),
    ("EMG_vastusmedialis_RMS", "EMG_SITE1_RMS",  0.8),
]
FEAT_TAGS = [f[1] for f in BANK_FEATURES]
FEAT_WEIGHTS = {f[1]: f[2] for f in BANK_FEATURES}
SRC_TO_TAG = {f[0]: f[1] for f in BANK_FEATURES}

N_THETA = 101
EPS = 1e-9


# ============================================================================
# 로딩
# ============================================================================

def load_population(mat_path):
    """h_d_population.mat / h_d_population_12.mat 로드.
    반환: pop(struct), sub(struct, 필드명=subjectId), meta(struct)
    """
    d = sio.loadmat(mat_path, struct_as_record=False, squeeze_me=True)
    return d["h_d_population"], d["h_d_subject"], d["h_d_meta"]


def load_stride_bank(mat_path=STRIDE_BANK_12):
    """stride_bank_12.mat 로드.
    반환: dict[subjId][modeName][featTag] = np.ndarray [nStride x 101]
    """
    d = sio.loadmat(mat_path, struct_as_record=False, squeeze_me=True)
    raw = d["strideBank"]
    out = {}
    for subjId in raw._fieldnames:
        subjS = getattr(raw, subjId)
        out[subjId] = {}
        for modeName in subjS._fieldnames:
            modeS = getattr(subjS, modeName)
            out[subjId][modeName] = {}
            for featTag in modeS._fieldnames:
                arr = np.atleast_2d(np.asarray(getattr(modeS, featTag), dtype=float))
                out[subjId][modeName][featTag] = arr
    return out


def load_subject_info():
    """subject_info_export.mat (Height[m]/Weight[kg]/Age/Gender) -> dict[subjId] = {...}"""
    d = sio.loadmat(SUBJECT_INFO, struct_as_record=False, squeeze_me=True)
    subjects = np.atleast_1d(d["subjectId"])
    heights = np.atleast_1d(d["height"])
    weights = np.atleast_1d(d["weight"])
    ages = np.atleast_1d(d["age"])
    genders = np.atleast_1d(d["gender"])
    out = {}
    for i, s in enumerate(subjects):
        out[str(s)] = dict(height=float(heights[i]), weight=float(weights[i]),
                            age=float(ages[i]), gender=str(genders[i]))
    return out


def curve(container, mode, feat):
    """(mean[101], std[101], nStride) 반환. 없으면 None. (analyze_*.py 와 동일 규약)"""
    ms = getattr(container, mode, None)
    if ms is None:
        return None
    fs = getattr(ms, feat, None)
    if fs is None:
        return None
    mu = np.atleast_1d(np.asarray(fs.mean, dtype=float))
    sd = np.atleast_1d(np.asarray(fs.std, dtype=float))
    if mu.size != N_THETA or sd.size != N_THETA:
        return None
    if not (np.all(np.isfinite(mu)) and np.all(np.isfinite(sd))):
        return None
    n = int(np.asarray(fs.nStride).reshape(-1)[0]) if hasattr(fs, "nStride") else None
    return mu, sd, n


# ============================================================================
# 펌웨어 재현: _CurveStats / _FinishCalibration / _ClassifyStride
# ============================================================================

def curve_stats(v):
    """_CurveStats() — 곡선 101점 자체의 mean/std (모집단 std, N분모). 위상축 상의
    '전체 레벨/전체 스프레드'를 뽑아 캘리브레이션 정렬에 쓴다 (theta별 stride간
    분산인 kRcStd와는 다른 개념)."""
    v = np.asarray(v, dtype=float)
    m = float(np.mean(v))
    var = float(np.mean((v - m) ** 2))
    return m, np.sqrt(var)


def estimate_calibration(own_curves, bank_walk):
    """_FinishCalibration() 재현.
    own_curves: dict[featTag] -> own WALK 평균곡선 [101] (그 사람 자신의 calibration 데이터)
    bank_walk: bank["WALK"] 그대로 — dict[featTag] -> (mean[101], std[101]) 튜플.
               (평균곡선만 필요하므로 내부에서 [0]으로 꺼내 쓴다 — 호출부에서
               실수로 (mean,std) 튜플째 넘겨 버그가 났던 적이 있어 아예 이 형태를
               표준 입력으로 받는다.)
    반환: phase_shift(int), calib: dict[featTag] -> dict(sign, scale, offset, enabled)
    """
    ref = bank_walk["THIGH_GYROMAG"][0]
    ref_mean, ref_std = curve_stats(ref)
    own_mean, own_std = curve_stats(own_curves["THIGH_GYROMAG"])

    best_shift, best_corr = 0, -np.inf
    own_tg = own_curves["THIGH_GYROMAG"]
    for k in range(N_THETA):
        shifted_ref = np.roll(ref, -k)  # ref[(i+k)%101] for i=0..100
        acc = float(np.sum((own_tg - own_mean) * (shifted_ref - ref_mean)))
        if acc > best_corr:
            best_corr, best_shift = acc, k

    calib = {}
    for tag in FEAT_TAGS:
        rc = bank_walk[tag][0]
        own = own_curves[tag]
        rc_mean, rc_std = curve_stats(rc)
        ow_mean, ow_std = curve_stats(own)
        if rc_std <= 1e-9 or ow_std <= 1e-9:
            calib[tag] = dict(sign=1.0, scale=1.0, offset=0.0, enabled=False)
            continue
        rc_shifted = np.roll(rc, -best_shift)
        acc = float(np.sum((own - ow_mean) * (rc_shifted - rc_mean)))
        sign = 1.0 if acc >= 0.0 else -1.0
        scale = rc_std / ow_std
        offset = rc_mean - sign * scale * ow_mean
        calib[tag] = dict(sign=sign, scale=scale, offset=offset, enabled=True)

    return best_shift, calib


def apply_calibration(x_curve, calib_f):
    return calib_f["sign"] * calib_f["scale"] * np.asarray(x_curve, dtype=float) + calib_f["offset"]


def classify_curves(x_dict, bank, phase_shift, calib):
    """_ClassifyStride() 재현.
    x_dict: dict[featTag] -> 실측 stride 곡선 [101] (보정 전, 원단위)
    bank: dict[mode][featTag] = (mean[101], std[101])
    phase_shift: estimate_calibration()이 반환한 정수
    calib: estimate_calibration()이 반환한 feature별 sign/scale/offset/enabled
    반환: z (dict[mode] -> float), pred_mode (str)
    """
    z = {}
    for mode in BANK_MODES:
        acc, wsum = 0.0, 0.0
        for tag in FEAT_TAGS:
            if not calib[tag]["enabled"]:
                continue
            w = FEAT_WEIGHTS[tag]
            xc = apply_calibration(x_dict[tag], calib[tag])
            rc_mean, rc_std = bank[mode][tag]
            # kRcMean[m][f][(i+shift)%101] indexed by i == np.roll(rc, -shift)[i]
            rc_mean_shifted = np.roll(rc_mean, -phase_shift)
            rc_std_shifted = np.roll(rc_std, -phase_shift)
            d = (xc - rc_mean_shifted) / rc_std_shifted
            acc += w * float(np.mean(d ** 2))
            wsum += w
        z[mode] = np.sqrt(acc / wsum) if wsum > 0 else np.inf
    pred_mode = min(z, key=z.get)
    return z, pred_mode


def circular_shift_distance(shift, n=N_THETA):
    """phase_shift 를 '0으로부터 원형 거리'로 정규화. 101(=n)에 대해 100/99/97 은
    사실 -1/-2/-4 와 같은 값이므로(원형), 절대크기 min(shift, n-shift) 로 봐야
    '위상원점이 실제로 얼마나 안 맞는지' 를 올바르게 판단할 수 있다."""
    shift = shift % n
    return min(shift, n - shift)


# ============================================================================
# 뱅크 구축 (임의 subject 부분집합으로부터)
# ============================================================================

def pooled_mean_std(subj_stats):
    """총분산의 법칙으로 여러 subject의 (mean,std,nStride)를 population 레벨로 합성.
    subj_stats: list of (mean[101], std[101], nStride)
    각 subject 내부는 '피험자내 stride간 분산'(std), subject간 평균 차이는
    '피험자간 분산'으로 별도 반영 — export_ref_curve_tables.m 의 IDLE 스칼라
    집계와 동일한 원리를 theta별로 적용한 것.
    """
    means = np.stack([s[0] for s in subj_stats])       # [nSubj x 101]
    stds = np.stack([s[1] for s in subj_stats])         # [nSubj x 101]
    ns = np.array([s[2] for s in subj_stats], dtype=float)
    w = ns / ns.sum()

    grand_mean = np.sum(w[:, None] * means, axis=0)                      # [101]
    within = np.sum(w[:, None] * (stds ** 2), axis=0)                    # E[std^2]
    between = np.sum(w[:, None] * (means - grand_mean[None, :]) ** 2, axis=0)  # Var[mean]
    grand_std = np.sqrt(within + between)
    return grand_mean, grand_std


def build_bank_from_subjects(h_d_subject, subject_ids, modes=BANK_MODES, std_floor_frac=0.01):
    """h_d_subject(어느 population.mat이든)에서 지정한 subject_ids만 골라
    export_ref_curve_tables.m 과 동일한 규약(std 하한 클램프 포함)으로 뱅크를 만든다.
    반환: bank[mode][featTag] = (mean[101], std[101])
    """
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
                if c is not None:
                    stats.append(c)
            if not stats:
                raise ValueError(f"뱅크 구축 실패: {mode}/{tag} 에 데이터 있는 subject 없음")
            mu, sd = pooled_mean_std(stats)
            floor_val = max(std_floor_frac * np.max(np.abs(mu)), 1e-6)
            sd = np.maximum(sd, floor_val)
            bank[mode][tag] = (mu, sd)
    return bank


def own_walk_curves_for(h_d_subject, subj_id):
    """그 사람 자신의 WALK 평균곡선 (BTN1 캘리브레이션 데이터의 역할)."""
    sc = getattr(h_d_subject, subj_id)
    out = {}
    for src_name, tag, _w in BANK_FEATURES:
        c = curve(sc, "WALK", src_name)
        if c is None:
            return None
        out[tag] = c[0]
    return out


def confusion_matrix(true_labels, pred_labels, labels=BANK_MODES):
    idx = {m: i for i, m in enumerate(labels)}
    cm = np.zeros((len(labels), len(labels)), dtype=int)
    for t, p in zip(true_labels, pred_labels):
        cm[idx[t], idx[p]] += 1
    return cm


def print_confusion_matrix(cm, labels=BANK_MODES, title=""):
    if title:
        print(f"\n{title}")
    print("%-14s" % "true\\pred" + "".join("%13s" % m[:12] for m in labels) + "%10s" % "acc")
    for i, m in enumerate(labels):
        row = cm[i]
        total = row.sum()
        acc = row[i] / total * 100 if total > 0 else float("nan")
        print("%-14s" % m[:13] + "".join("%13d" % v for v in row) + "%9.1f%%" % acc)
    overall = np.trace(cm) / max(cm.sum(), 1) * 100
    print("-" * (14 + 13 * len(labels) + 10))
    print("전체 정확도 = %.2f%%  (n=%d strides)" % (overall, cm.sum()))
    return overall
