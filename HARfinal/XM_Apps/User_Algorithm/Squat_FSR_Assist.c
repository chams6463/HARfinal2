/**
 ******************************************************************************
 * @file    Squat_FSR_Assist.c
 * @brief   EMG-proportional squat assist with THREE-SENSOR FUSION phase detection.
 *
 *          Phase判定를 encoder 각도 단독에서 아래 3가지 신호 퓨전으로 교체:
 *            1) Encoder 고관절 각도 (기하학적 기준)
 *            2) EMG 활성도 (근육 의도 신호)
 *            3) FSR Heel-Toe 압력 분포 비율 (발 하중 분포)
 *
 *          Base 파일: Squat_EMG_Assist.c  (원본 수정 없음)
 *          변경 범위:
 *            - 상수: FSR_MIN_TOTAL_LOAD, FUSION_* 기본값 추가
 *            - static 변수: s_fsr_heel_ratio, s_fsr_total_load, s_avg_emg
 *            - 공개 변수: fsr_heel_ratio, fsr_total_load, avg_emg_norm +
 *                        퓨전 튜닝 파라미터 7개
 *            - _UpdateSquatFsm(): 3중 퓨전 조건으로 전면 교체
 *            - _UpdatePublicSignals(): 신규 신호 복사 추가
 *            - CDC 스트림: fsr_heel_ratio, avg_emg_norm 채널 추가 (총 16채널)
 *
 * ============================================================================
 * Pin mapping  (Squat_EMG_Assist.c 와 동일)
 * ============================================================================
 *   PF3 = EMG right hip   (XM_EXT_DIO_1 -> XM_EXT_ADC_5)
 *   PF4 = EMG left hip    (XM_EXT_DIO_2 -> XM_EXT_ADC_6)
 *   PF5 = FSR left toe    (XM_EXT_DIO_3 -> XM_EXT_ADC_7)
 *   PF6 = FSR left heel   (XM_EXT_DIO_4 -> XM_EXT_ADC_8)
 *   PF7 = FSR right toe   (XM_EXT_DIO_5 -> XM_EXT_ADC_9)
 *   PF8 = FSR right heel  (XM_EXT_DIO_6 -> XM_EXT_ADC_10)
 *
 * ============================================================================
 * Operating procedure  (Squat_EMG_Assist.c 와 동일)
 * ============================================================================
 *  1) Switch H10 to ASSIST mode.
 *  2) Stand unloaded, press BTN1 (PRESSED) -> FSR zero cal, hold still 1 s.
 *  3) Apply full body weight, press BTN2 (PRESSED) -> FSR load cal, hold 1 s.
 *  4) Relax muscles, press BTN1 (CLICK) -> EMG rest cal, stay relaxed 3 s.
 *  5) Contract target muscles, press BTN2 (CLICK) -> EMG effort cal, 3 s.
 *  6) Confirm fsr_cal_ready==1 and emg_cal_done==1 in Live Expressions.
 *  7) Set squat_control_ON=1 to enable assistance.
 *  BTN3 PRESSED resets FSR cal.  BTN3 CLICK resets EMG cal.
 *
 * ============================================================================
 * Safety gate  (변경 없음)
 * ============================================================================
 *  squat_control_ON == 1  AND  fsr_cal_ready == 1  AND  H10 mode == ASSIST
 *
 * ============================================================================
 * Fusion phase detection — 추가된 Live Expressions 튜닝 변수
 * ============================================================================
 * Write from debugger:
 *   fsr_heel_thresh        heel 우세 판정 임계값 (기본 0.58, 0=toe, 1=heel)
 *   fsr_toe_thresh         toe  우세 판정 임계값 (기본 0.42)
 *   fsr_min_total_load     FSR 총 하중이 이 미만이면 heel ratio 계산 무효 (기본 0.05)
 *   emg_phase_thresh       EMG 활성 판정 임계값 (기본 0.10, 0~1)
 *   phase_w_enc            퓨전 가중치 — encoder (기본 0.50)
 *   phase_w_fsr            퓨전 가중치 — FSR     (기본 0.30)
 *   phase_w_emg            퓨전 가중치 — EMG     (기본 0.20)
 *   phase_fusion_thresh    전환 확정 퓨전 점수 임계값 (기본 0.50)
 *
 * Observe only (추가):
 *   fsr_heel_ratio         실시간 heel/(heel+toe) 비율 0~1
 *   fsr_total_load         실시간 4채널 합산 하중
 *   avg_emg_norm           좌우 EMG norm 평균 0~1
 *
 * ============================================================================
 * CDC stream (Module ID 0xF0, 16 float channels, 100 Hz)
 * ============================================================================
 *  1) EMG RH Env   5) FSR LT     9) Squat Ph    13) FSR Cal
 *  2) EMG LH Env   6) FSR LH    10) L Torque    14) Control
 *  3) EMG RH Nrm   7) FSR RT    11) R Torque    15) Heel Ratio
 *  4) EMG LH Nrm   8) FSR RH   12) Avg EMG     16) Assist
 ******************************************************************************
 */

#include "xm_api.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * CONSTANTS
 * ============================================================================ */

#define FSR_COUNT                   4U
#define EMG_COUNT                   2U
#define CONTROL_DT_S                0.001f
#define USB_MODULE_ID               0xF1U    /* 0xF0=EMG_Assist와 충돌 방지 — 별도 디코더 ID */
#define HARD_MAX_ASSIST_TORQUE_NM   2.5f

/* FSR */
#define FSR_LPF_CUTOFF_HZ           8.0f
#define FSR_CAL_DURATION_MS         1000U
#define FSR_MINIMUM_SPAN_V          0.05f

/*
 * FSR_MIN_TOTAL_LOAD (0.05):
 *   4채널 합산 하중이 이 미만이면 FSR 신호를 퓨전에서 제외 (unloaded 상태).
 *   에러: 서 있는데도 heel_ratio = 0.5 고정으로 나옴
 *     → total_load < 이 값 → FSR 캘이 안 된 것. FSR 캘 먼저 수행.
 *   에러: 공중에 뜬 발에서도 heel_ratio 이 요동침
 *     → 이 값을 0.10~0.15 로 올릴 것.
 */
#define FSR_MIN_TOTAL_LOAD_DEFAULT  0.05f

/* EMG */
#define EMG_RAW_LPF_CUTOFF_HZ       80.0f
#define EMG_ENV_LPF_CUTOFF_HZ       5.0f
#define EMG_DEADBAND_V              0.020f
#define EMG_DEFAULT_BIAS_V          1.65f
#define EMG_DEFAULT_FULL_SCALE_V    1.000f
#define EMG_MIN_FULL_SCALE_V        0.050f
#define EMG_CAL_DURATION_MS         3000U

/* Encoder / compensation */
#define DEG_TO_RAD                  0.01745329252f
#define TWO_PI                      6.28318530718f
#define COMP_VEL_LPF_HZ             5.0f

/* Squat FSM debounce */
#define SQUAT_DEBOUNCE_MS           150U

/*
 * Fusion default weights  (합이 1.0 이 되도록 맞출 필요 없음 — phase_fusion_thresh 로 조정)
 *
 * 설계 의도:
 *   encoder(0.50) : 기하학적 각도가 가장 신뢰할 수 있는 primary 기준.
 *   FSR    (0.30) : 발 하중 이동은 스쿼트 동작 패턴을 잘 반영.
 *   EMG    (0.20) : 근육 활성도는 의도 검출 보조. 캘 미완료 시 신뢰도 낮음.
 *
 * 퓨전 통과 기준(phase_fusion_thresh = 0.50):
 *   encoder만 조건 만족: 0.50 → 겨우 통과 (encoder 단독)
 *   encoder + FSR    : 0.80 → 통과
 *   encoder + EMG    : 0.70 → 통과
 *   FSR + EMG 만     : 0.50 → 통과 (encoder 조건 미달이지만 FSR+EMG 확신 강할 때)
 *
 * 에러: 상태 전환이 너무 쉽게 일어남 → phase_fusion_thresh 를 0.60~0.70 으로 올릴 것.
 * 에러: 상태 전환이 전혀 안 일어남 → phase_fusion_thresh 를 0.40 으로 낮추거나
 *         FSR/EMG 캘 완료 여부 먼저 확인.
 */
#define FUSION_W_ENC_DEFAULT        0.50f
#define FUSION_W_FSR_DEFAULT        0.30f
#define FUSION_W_EMG_DEFAULT        0.20f
#define FUSION_THRESH_DEFAULT       0.50f

/*
 * FSR heel/toe 판정 임계값
 *
 * heel_ratio = heel_load / (heel_load + toe_load)
 *   1.0 = 완전 뒤꿈치 하중  /  0.0 = 완전 앞꿈치 하중
 *
 * fsr_heel_thresh (0.58):
 *   하강 시 체중이 뒤로 이동하는 특성. 이 값 이상이면 heel 우세 판정.
 *   에러: heel_dominant 가 항상 false → fsr_heel_thresh 를 0.52 로 낮출 것.
 *   에러: 가만히 서있어도 heel_dominant → 기립 자세가 뒤꿈치 편향. 0.65 로 올릴 것.
 *
 * fsr_toe_thresh (0.42):
 *   상승(concentric) 시 발의 앞꿈치 push-off 특성. 이 값 이하면 toe 우세 판정.
 *   에러: toe_dominant 가 항상 false → fsr_toe_thresh 를 0.48 로 올릴 것.
 *   주의: heel_thresh와 toe_thresh 사이(0.42~0.58)가 neutral zone.
 */
#define FSR_HEEL_THRESH_DEFAULT     0.58f
#define FSR_TOE_THRESH_DEFAULT      0.42f

/*
 * emg_phase_thresh (0.10):
 *   EMG norm이 이 값 이상이면 근육이 활성화된 것으로 판정.
 *   에러: 이완 상태인데도 emg_active = true → 0.15~0.20 으로 올릴 것.
 *   에러: 수축해도 emg_active = false → 0.05 로 낮추거나 Effort 캘 재수행.
 */
#define EMG_PHASE_THRESH_DEFAULT    0.10f

/* ============================================================================
 * ENUMERATIONS AND TYPES
 * ============================================================================ */

typedef enum { FSR_LT = 0, FSR_LH, FSR_RT, FSR_RH } FsrIdx_t;
typedef enum { EMG_RH = 0, EMG_LH } EmgIdx_t;

typedef enum {
    FSR_CAL_IDLE = 0,
    FSR_CAL_ZERO_RUNNING,
    FSR_CAL_LOAD_RUNNING,
    FSR_CAL_DONE
} FsrCalState_t;

typedef enum {
    EMG_CAL_IDLE = 0,
    EMG_CAL_REST_RUNNING,
    EMG_CAL_EFFORT_RUNNING,
    EMG_CAL_DONE
} EmgCalState_t;

typedef enum {
    SQUAT_STAND = 0,
    SQUAT_DESCENDING,
    SQUAT_BOTTOM,
    SQUAT_ASCENDING,
    SQUAT_RETURN
} SquatPhase_t;

typedef struct {
    XmBtnEvent_t btn1;
    XmBtnEvent_t btn2;
    XmBtnEvent_t btn3;
} BtnEvents_t;

/* ============================================================================
 * CDC STREAM CONFIGURATION  (16 channels, Module ID 0xF1)
 * ============================================================================ */
#define CDC_STREAM_CHANNELS(F, N)                                                    \
    F(emg_rh_env,    "EMG RH Env",  "V",    emg_rh_envelope_v)                      \
    N(emg_lh_env,    "EMG LH Env",  "V",    emg_lh_envelope_v)                      \
    N(emg_rh_nrm,    "EMG RH Nrm",  "-",    emg_rh_norm)                            \
    N(emg_lh_nrm,    "EMG LH Nrm",  "-",    emg_lh_norm)                            \
    N(fsr_lt,        "FSR LT",      "-",    fsr_lt_load)                            \
    N(fsr_lh,        "FSR LH",      "-",    fsr_lh_load)                            \
    N(fsr_rt,        "FSR RT",      "-",    fsr_rt_load)                            \
    N(fsr_rh,        "FSR RH",      "-",    fsr_rh_load)                            \
    N(squat_ph,      "Squat Ph",    "id",   squat_phase)                            \
    N(l_torque,      "L Torque",    "Nm",   left_assist_torque_nm)                  \
    N(r_torque,      "R Torque",    "Nm",   right_assist_torque_nm)                 \
    N(avg_emg_ch,    "Avg EMG",     "-",    avg_emg_norm)                           \
    N(fsr_cal,       "FSR Cal",     "bool", fsr_cal_ready)                          \
    N(ctrl_on,       "Control",     "bool", squat_control_ON)                       \
    N(heel_ratio_ch, "Heel Ratio",  "-",    fsr_heel_ratio)                         \
    N(assist_on,     "Assist",      "bool", assist_enable)

#define CDC_DECLARE_FIELD(field, name, unit, src) float field;
typedef struct {
    CDC_STREAM_CHANNELS(CDC_DECLARE_FIELD, CDC_DECLARE_FIELD)
} SquatStreamData_t;
#undef CDC_DECLARE_FIELD

#define CDC_META_FIRST(f, name, unit, src) "[{\"name\":\"" name "\",\"unit\":\"" unit "\"}"
#define CDC_META_NEXT( f, name, unit, src)  ",{\"name\":\"" name "\",\"unit\":\"" unit "\"}"
static const char s_cdc_stream_meta[] =
    CDC_STREAM_CHANNELS(CDC_META_FIRST, CDC_META_NEXT) "]";
#undef CDC_META_FIRST
#undef CDC_META_NEXT

_Static_assert(sizeof(s_cdc_stream_meta) <= 513U,
               "CDC metadata exceeds the XM_SetUsbCustomMeta 512-byte limit");

/* ============================================================================
 * STATIC (PRIVATE) VARIABLES
 * ============================================================================ */

/* --- FSR --- */
static const XmAdcPin_t s_fsr_pins[FSR_COUNT] = {
    XM_EXT_ADC_7, XM_EXT_ADC_8, XM_EXT_ADC_9, XM_EXT_ADC_10
};
static float        s_fsr_raw_v[FSR_COUNT];
static float        s_fsr_lpf_v[FSR_COUNT];
static float        s_fsr_zero_v[FSR_COUNT];
static float        s_fsr_full_v[FSR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static float        s_fsr_load[FSR_COUNT];
static bool         s_fsr_filter_init;
static FsrCalState_t s_fsr_cal_state;
static double       s_fsr_cal_sum[FSR_COUNT];
static uint32_t     s_fsr_cal_count;
static uint32_t     s_fsr_cal_tick;
static bool         s_fsr_zero_done;
static bool         s_fsr_full_done;

/* --- FSR 퓨전 중간 신호 --- */
/*
 * s_fsr_heel_ratio: heel/(heel+toe) 비율 (0=전부 toe, 1=전부 heel).
 *   total_load < fsr_min_total_load 이면 0.5(neutral)로 고정.
 *   FSM 조건 판정에 직접 사용.
 *
 * s_fsr_total_load: 4채널 FSR load 합산 (0~6.0 이론 상한).
 *   이 값이 0이면 FSR이 캘 안 됐거나 발이 완전히 들린 것.
 */
static float        s_fsr_heel_ratio;
static float        s_fsr_total_load;

/* --- EMG --- */
static const XmAdcPin_t s_emg_pins[EMG_COUNT] = {
    XM_EXT_ADC_5, XM_EXT_ADC_6
};
static float        s_emg_raw_v[EMG_COUNT];
static float        s_emg_centered_v[EMG_COUNT];
static float        s_emg_pre_lpf_v[EMG_COUNT];
static float        s_emg_rect_v[EMG_COUNT];
static float        s_emg_envelope_v[EMG_COUNT];
static float        s_emg_norm_priv[EMG_COUNT];
static float        s_emg_bias_v[EMG_COUNT]       = {EMG_DEFAULT_BIAS_V,
                                                      EMG_DEFAULT_BIAS_V,
                                                      EMG_DEFAULT_BIAS_V};
static float        s_emg_full_scale_v[EMG_COUNT] = {EMG_DEFAULT_FULL_SCALE_V,
                                                      EMG_DEFAULT_FULL_SCALE_V,
                                                      EMG_DEFAULT_FULL_SCALE_V};
static EmgCalState_t s_emg_cal_state;
static uint32_t     s_emg_cal_tick;
static uint32_t     s_emg_cal_count;
static double       s_emg_cal_sum[EMG_COUNT];
static float        s_emg_cal_max_env[EMG_COUNT];

/* --- EMG 퓨전 중간 신호 --- */
/*
 * s_avg_emg: 좌우 EMG norm 평균 (0~1).
 *   emg_phase_thresh 와 비교해 근육 활성 여부 판정.
 *   EMG 캘 미완료 시 이 값이 매우 작게 나오므로 퓨전 기여도가 낮아짐 (안전).
 */
static float        s_avg_emg;

/* --- Encoder / compensation --- */
static bool         s_comp_prev_valid;
static float        s_comp_prev_left_rad;
static float        s_comp_prev_right_rad;

/* --- Squat FSM --- */
static SquatPhase_t s_squat_phase;
static uint32_t     s_squat_debounce_tick;
static bool         s_squat_debounce_active;

/* --- System --- */
static bool              s_assist_session_active;
static bool              s_torque_mode_active;
static uint32_t          s_stream_tick;
static SquatStreamData_t s_stream;

/* ============================================================================
 * PUBLIC VARIABLES  (STM32CubeIDE Live Expressions)
 * ============================================================================ */

/* Controls */
uint16_t squat_control_ON       = 0U;
uint16_t compensation_ON        = 1U;
uint16_t friction_compensation_ON = 0U;
uint16_t cdc_stream_enable      = 1U;
uint16_t cdc_stream_period_ms   = 10U;

/* Status flags */
uint16_t assist_enable          = 0U;
uint16_t assist_mode_active     = 0U;
uint16_t fsr_cal_zero_done      = 0U;
uint16_t fsr_cal_full_done      = 0U;
uint16_t fsr_cal_ready          = 0U;
uint16_t emg_cal_rest_done      = 0U;
uint16_t emg_cal_effort_done    = 0U;
uint16_t emg_cal_done           = 0U;
uint16_t squat_phase            = 0U;

/* Sensor raw voltages */
float pf3_volt;
float pf4_volt;
float pf5_volt;
float pf6_volt;
float pf7_volt;
float pf8_volt;

/* FSR normalised loads */
float fsr_lt_load;
float fsr_lh_load;
float fsr_rt_load;
float fsr_rh_load;

/* FSR fusion observables */
/*
 * fsr_heel_ratio: 실시간 heel 하중 비율.
 *   스쿼트 하강 시 0.58+ 로 올라가는 것이 정상.
 *   상승 복귀 시 0.42- 로 내려가는 것이 정상.
 *   에러: 항상 0.5 → FSR 캘 미완료 또는 total_load < fsr_min_total_load.
 *
 * fsr_total_load: 4채널 합산 하중.
 *   체중 전체 하중 시 약 4.0 (각 채널 1.0 기준).
 *   에러: 0 고정 → FSR zero 캘 값이 full 캘 값보다 크거나 같은 것.
 */
float fsr_heel_ratio;
float fsr_total_load;

/* EMG processed */
float emg_rh_raw_v;
float emg_lh_raw_v;
float emg_rh_envelope_v;
float emg_lh_envelope_v;
float emg_rh_norm;
float emg_lh_norm;

/*
 * avg_emg_norm: 좌우 EMG norm 평균 (0~1).
 *   에러: 수축해도 0 → emg_phase_thresh 확인 후 EMG Effort 캘 재수행.
 *   에러: 이완해도 0.1+ → emg_phase_thresh 를 0.15 이상으로 올릴 것.
 */
float avg_emg_norm;

/* Encoder signals */
float left_encoder_angle_deg;
float right_encoder_angle_deg;
float left_control_angle_deg;
float right_control_angle_deg;
float left_angular_velocity_rads;
float right_angular_velocity_rads;

/* Compensation signals */
float left_gravity_torque_nm;
float right_gravity_torque_nm;
float left_compensation_torque_nm;
float right_compensation_torque_nm;

/* Final torque commands */
float left_assist_torque_nm;
float right_assist_torque_nm;

/* Encoder tuning */
float encoder_offset_lh_deg        = 0.0f;
float encoder_offset_rh_deg        = 0.0f;
float encoder_sign_lh              = 1.0f;
float encoder_sign_rh              = 1.0f;
float gravity_mgl_nm               = 1.0f;
float compensation_scale           = 0.20f;
float coulomb_friction_nm          = 0.10f;
float viscous_friction_nms         = 0.01f;
float velocity_deadzone_rads       = 0.10f;
float squat_max_torque_nm          = 0.3f;
float assist_torque_limit_nm       = 0.5f;
float torque_sign                  = 1.0f;
float squat_enter_threshold_deg    = 15.0f;
float squat_bottom_threshold_deg   = 45.0f;
float squat_stand_threshold_deg    = 5.0f;

/* ============================================================================
 * FUSION TUNING PARAMETERS  (Live Expressions에서 실시간 조정 가능)
 * ============================================================================
 *
 * 【퓨전 파라미터 조정 가이드】
 *
 * 퓨전 점수 계산식 (STAND→DESCENDING / RETURN→STAND 전환):
 *   score = phase_w_enc * enc_ok
 *         + phase_w_fsr * fsr_ok
 *         + phase_w_emg * emg_ok
 *   전환 확정: score >= phase_fusion_thresh
 *
 * ─ fsr_heel_thresh (0.58) ────────────────────────────────────────────────────
 *   하강 판정용 heel 우세 임계값.
 *   에러: 정상 스쿼트인데 DESCENDING 전환 안 됨 → 0.52 로 낮출 것.
 *   에러: 걸어다닐 때 오인식 → 0.65 이상으로 올릴 것.
 *
 * ─ fsr_toe_thresh (0.42) ─────────────────────────────────────────────────────
 *   상승 판정용 toe 우세 임계값.
 *   에러: BOTTOM→ASCENDING 전환이 속도 반전 없이 안 일어남 → 0.48 로 올릴 것.
 *   주의: heel_thresh - toe_thresh = 0.16이 neutral zone 폭. 너무 좁히면 진동 위험.
 *
 * ─ fsr_min_total_load (0.05) ─────────────────────────────────────────────────
 *   heel ratio 유효 판정 최소 하중.
 *   에러: 발을 드는 동작에서 heel_ratio 가 튐 → 0.10 으로 올릴 것.
 *
 * ─ emg_phase_thresh (0.10) ───────────────────────────────────────────────────
 *   근육 활성 판정 임계값.
 *   에러: 이완 중 활성 판정 → 0.15~0.20 으로 올릴 것.
 *   에러: 수축 중 미검출 → 0.05 로 낮출 것 (EMG 캘 재수행 권장).
 *
 * ─ phase_w_enc / phase_w_fsr / phase_w_emg ───────────────────────────────────
 *   가중치 합이 반드시 1.0일 필요 없음 — phase_fusion_thresh 로 기준 설정.
 *   EMG 캘 전: phase_w_emg = 0 으로 설정하면 encoder+FSR 2중 퓨전으로 동작.
 *   FSR 캘 전: phase_w_fsr = 0 으로 설정하면 encoder+EMG 2중 퓨전으로 동작.
 *
 * ─ phase_fusion_thresh (0.50) ────────────────────────────────────────────────
 *   전환 확정 점수 기준.
 *   기본값 0.50 에서 encoder(0.50) 하나만 만족해도 전환됨 = encoder 단독과 동일.
 *   FSR/EMG도 요구하려면 0.60~0.70 으로 올릴 것 (더 엄격한 퓨전).
 * ============================================================================ */
float fsr_heel_thresh       = FSR_HEEL_THRESH_DEFAULT;
float fsr_toe_thresh        = FSR_TOE_THRESH_DEFAULT;
float fsr_min_total_load    = FSR_MIN_TOTAL_LOAD_DEFAULT;
float emg_phase_thresh      = EMG_PHASE_THRESH_DEFAULT;
float phase_w_enc           = FUSION_W_ENC_DEFAULT;
float phase_w_fsr           = FUSION_W_FSR_DEFAULT;
float phase_w_emg           = FUSION_W_EMG_DEFAULT;
float phase_fusion_thresh   = FUSION_THRESH_DEFAULT;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

static BtnEvents_t _ReadButtons(void);

/* FSR */
static void _SampleFsr(void);
static void _UpdateFsrCal(const BtnEvents_t *ev);
static void _StartFsrCal(FsrCalState_t state);
static void _ResetFsrCal(void);
static void _UpdateLoads(void);
static void _ComputeHeelToeRatio(void);   /* 신규: heel/toe 비율 계산 */

/* EMG */
static void _SampleEmg(void);
static void _ResetEmgFilters(void);
static void _ProcessEmgSignals(void);
static void _UpdateEmgCal(const BtnEvents_t *ev);
static void _StartEmgRestCal(void);
static void _StartEmgEffortCal(void);
static void _ResetEmgCal(void);
static float _GetEmgNorm(int ch);

/* Encoder + compensation */
static void _SampleEncoder(void);
static void _UpdateCompensation(void);

/* Squat FSM + output */
static void _UpdateSquatFsm(void);   /* 3중 퓨전 버전 */
static void _ApplyTorque(void);

/* Housekeeping */
static void _UpdatePublicSignals(void);
static void _SendStream(void);

/* Utilities */
static float _LowPassUpdate(float prev, float input, float cutoff_hz);
static float _ClampFloat(float value, float lo, float hi);
static float _AbsFloat(float x);
static float _SignFloat(float x);

/* ============================================================================
 * PUBLIC ENTRY POINTS
 * ============================================================================ */

void User_Setup(void)
{
    XM_SetExtPowerVoltage(XM_EXT_PWR_5V);
    XM_SwitchDioToAdc(XM_EXT_DIO_1);
    XM_SwitchDioToAdc(XM_EXT_DIO_2);
    XM_SwitchDioToAdc(XM_EXT_DIO_3);
    XM_SwitchDioToAdc(XM_EXT_DIO_4);
    XM_SwitchDioToAdc(XM_EXT_DIO_5);
    XM_SwitchDioToAdc(XM_EXT_DIO_6);

    XM_SetControlMode(XM_CTRL_MONITOR);
    XM_SetH10AssistExistingMode(false);
    XM_SetUsbTotalDataStream(false);
    XM_SetUsbCustomMeta(USB_MODULE_ID, s_cdc_stream_meta);

    _ResetFsrCal();
    _ResetEmgCal();

    s_fsr_heel_ratio = 0.5f;
    s_fsr_total_load = 0.0f;
    s_avg_emg        = 0.0f;

    s_stream_tick = XM_GetTick();
    XM_SendUsbDebugMessage(
        "[FSR_ASSIST] ready | FSR: BTN1-PRESS=zero  BTN2-PRESS=load  BTN3-PRESS=reset\r\n"
        "[FSR_ASSIST]        | EMG: BTN1-CLICK=rest  BTN2-CLICK=effort  BTN3-CLICK=reset\r\n"
        "[FSR_ASSIST] Fusion: encoder+FSR heel-toe+EMG activation\r\n");
}

void User_Loop(void)
{
    BtnEvents_t ev;
    bool is_assist_mode;

    XM_IO_Update();

    _SampleFsr();
    _SampleEmg();
    _SampleEncoder();

    is_assist_mode = (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST);

    if (!is_assist_mode) {
        if (s_assist_session_active) {
            s_fsr_cal_state         = FSR_CAL_IDLE;
            s_fsr_zero_done         = false;
            s_fsr_full_done         = false;
            _ResetEmgCal();
            s_squat_phase           = SQUAT_STAND;
            s_squat_debounce_active = false;
            s_comp_prev_valid       = false;
            s_assist_session_active = false;
            XM_SendUsbDebugMessage("[FSR_ASSIST] session ended\r\n");
        }
        _ApplyTorque();
        _UpdatePublicSignals();
        _SendStream();
        return;
    }

    if (!s_assist_session_active) {
        _ResetFsrCal();
        _ResetEmgCal();
        s_squat_phase           = SQUAT_STAND;
        s_squat_debounce_active = false;
        s_comp_prev_valid       = false;
        s_fsr_heel_ratio        = 0.5f;
        s_fsr_total_load        = 0.0f;
        s_avg_emg               = 0.0f;
        s_assist_session_active = true;
        XM_SendUsbDebugMessage("[FSR_ASSIST] session started\r\n");
    }

    ev = _ReadButtons();

    /* (1) FSR 캘 */
    _UpdateFsrCal(&ev);

    /* (2) 정규화 FSR 하중 */
    _UpdateLoads();

    /* (3) Heel-Toe 비율 계산 (FSM 퓨전 입력) */
    _ComputeHeelToeRatio();

    /* (4) EMG 신호 처리 */
    if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
        _ResetEmgFilters();
    } else {
        _ProcessEmgSignals();
    }

    /* (5) EMG 캘 */
    _UpdateEmgCal(&ev);

    /* (6) 중력/마찰 보상 */
    _UpdateCompensation();

    /* (7) 3중 퓨전 스쿼트 FSM */
    _UpdateSquatFsm();

    /* (8) 안전 게이트 + 토크 출력 */
    _ApplyTorque();

    /* (9) public 변수 동기화 */
    _UpdatePublicSignals();

    /* (10) CDC USB 스트림 전송 */
    _SendStream();
}

/* ============================================================================
 * BUTTON
 * ============================================================================ */

static BtnEvents_t _ReadButtons(void)
{
    BtnEvents_t ev;
    ev.btn1 = XM_GetButtonEvent(XM_BTN_1);
    ev.btn2 = XM_GetButtonEvent(XM_BTN_2);
    ev.btn3 = XM_GetButtonEvent(XM_BTN_3);
    return ev;
}

/* ============================================================================
 * FSR
 * ============================================================================ */

static void _SampleFsr(void)
{
    const float rc    = 1.0f / (TWO_PI * FSR_LPF_CUTOFF_HZ);
    const float alpha = CONTROL_DT_S / (rc + CONTROL_DT_S);
    int i;

    for (i = 0; i < (int)FSR_COUNT; i++) {
        s_fsr_raw_v[i] = (float)XM_AnalogReadMillivolts(s_fsr_pins[i]) * 0.001f;
        if (!s_fsr_filter_init) {
            s_fsr_lpf_v[i] = s_fsr_raw_v[i];
        } else {
            s_fsr_lpf_v[i] += alpha * (s_fsr_raw_v[i] - s_fsr_lpf_v[i]);
        }
    }
    s_fsr_filter_init = true;

    if (s_fsr_cal_state == FSR_CAL_ZERO_RUNNING ||
        s_fsr_cal_state == FSR_CAL_LOAD_RUNNING) {
        for (i = 0; i < (int)FSR_COUNT; i++) {
            s_fsr_cal_sum[i] += s_fsr_lpf_v[i];
        }
        s_fsr_cal_count++;
    }
}

static void _UpdateFsrCal(const BtnEvents_t *ev)
{
    bool busy;
    bool elapsed;
    FsrCalState_t done_state;
    int i;

    busy = (s_fsr_cal_state == FSR_CAL_ZERO_RUNNING ||
            s_fsr_cal_state == FSR_CAL_LOAD_RUNNING);

    if (ev->btn3 == XM_BTN_PRESSED && !busy) {
        _ResetFsrCal();
        XM_SendUsbDebugMessage("[FSR CAL] reset to defaults\r\n");
        return;
    }
    if (ev->btn1 == XM_BTN_PRESSED && !busy) {
        _StartFsrCal(FSR_CAL_ZERO_RUNNING);
        XM_SendUsbDebugMessage("[FSR CAL] zero: stand unloaded for 1 s\r\n");
        return;
    }
    if (ev->btn2 == XM_BTN_PRESSED && !busy) {
        if (s_fsr_zero_done) {
            _StartFsrCal(FSR_CAL_LOAD_RUNNING);
            XM_SendUsbDebugMessage("[FSR CAL] full load: apply weight for 1 s\r\n");
        } else {
            XM_SendUsbDebugMessage("[FSR CAL] do BTN1 zero before BTN2 load\r\n");
        }
        return;
    }

    if (!busy) { return; }

    elapsed = ((XM_GetTick() - s_fsr_cal_tick) >= FSR_CAL_DURATION_MS);
    if (!elapsed) { return; }

    done_state = s_fsr_cal_state;
    if (s_fsr_cal_count > 0U) {
        for (i = 0; i < (int)FSR_COUNT; i++) {
            float avg = (float)(s_fsr_cal_sum[i] / (double)s_fsr_cal_count);
            if (done_state == FSR_CAL_ZERO_RUNNING) {
                s_fsr_zero_v[i] = avg;
            } else {
                s_fsr_full_v[i] = avg;
            }
        }
        if (done_state == FSR_CAL_ZERO_RUNNING) {
            s_fsr_zero_done = true;
        } else {
            s_fsr_full_done = true;
        }
    }
    s_fsr_cal_state = (s_fsr_zero_done && s_fsr_full_done) ? FSR_CAL_DONE
                                                            : FSR_CAL_IDLE;
    XM_SetLedEffect(XM_LED_1, XM_LED_ONESHOT, 500);
    XM_SendUsbDebugMessage("[FSR CAL] capture complete\r\n");
}

static void _StartFsrCal(FsrCalState_t state)
{
    memset(s_fsr_cal_sum, 0, sizeof(s_fsr_cal_sum));
    s_fsr_cal_count = 0U;
    s_fsr_cal_tick  = XM_GetTick();
    s_fsr_cal_state = state;
    if (state == FSR_CAL_ZERO_RUNNING) {
        s_fsr_zero_done = false;
        s_fsr_full_done = false;
    } else {
        s_fsr_full_done = false;
    }
    XM_SetLedEffect(
        (state == FSR_CAL_ZERO_RUNNING) ? XM_LED_1 : XM_LED_2,
        XM_LED_BLINK, 100);
}

static void _ResetFsrCal(void)
{
    int i;
    memset(s_fsr_zero_v, 0, sizeof(s_fsr_zero_v));
    memset(s_fsr_load,   0, sizeof(s_fsr_load));
    for (i = 0; i < (int)FSR_COUNT; i++) {
        s_fsr_full_v[i] = 1.0f;
    }
    s_fsr_cal_state   = FSR_CAL_IDLE;
    s_fsr_filter_init = false;
    s_fsr_zero_done   = false;
    s_fsr_full_done   = false;
    fsr_cal_zero_done = 0U;
    fsr_cal_full_done = 0U;
    fsr_cal_ready     = 0U;
    s_fsr_heel_ratio  = 0.5f;
    s_fsr_total_load  = 0.0f;
    XM_SetLedEffect(XM_LED_3, XM_LED_ONESHOT, 500);
}

static void _UpdateLoads(void)
{
    int i;
    for (i = 0; i < (int)FSR_COUNT; i++) {
        float span = s_fsr_full_v[i] - s_fsr_zero_v[i];
        if (span < FSR_MINIMUM_SPAN_V) {
            span = FSR_MINIMUM_SPAN_V;
        }
        s_fsr_load[i] = _ClampFloat(
            (s_fsr_lpf_v[i] - s_fsr_zero_v[i]) / span, 0.0f, 1.5f);
    }
}

/* ============================================================================
 * HEEL-TOE RATIO  (신규 함수)
 * ============================================================================
 *
 * 역할:
 *   s_fsr_load[] 에서 heel(LH/RH)과 toe(LT/RT) 를 분리해
 *   s_fsr_heel_ratio (0=all-toe ~ 1=all-heel) 와 s_fsr_total_load 를 갱신.
 *
 * 핀/인덱스 매핑:
 *   FSR_LT = index 0 (PF5, left toe)   FSR_RT = index 2 (PF7, right toe)
 *   FSR_LH = index 1 (PF6, left heel)  FSR_RH = index 3 (PF8, right heel)
 *
 * 에러: s_fsr_heel_ratio 가 항상 0.5
 *   → s_fsr_total_load < fsr_min_total_load (FSR 캘 미완료 or 무부하)
 *   → FSR 캘 완료 후 체중을 신발 안에 올린 상태에서 확인.
 *
 * 에러: heel_ratio 가 0.90 이상으로 극단적
 *   → FSR toe 센서가 물리적으로 닿지 않는 것. 신발 내 센서 위치 확인.
 *
 * 에러: 걷거나 뛸 때 heel_ratio 이 크게 요동침
 *   → FSR_LPF_CUTOFF_HZ 를 4 Hz 로 낮추면 완화됨 (단, 응답 느려짐).
 */
static void _ComputeHeelToeRatio(void)
{
    float heel;
    float toe;
    float total;

    heel  = s_fsr_load[FSR_LH] + s_fsr_load[FSR_RH];
    toe   = s_fsr_load[FSR_LT] + s_fsr_load[FSR_RT];
    total = heel + toe;

    s_fsr_total_load = total;

    if (total > _ClampFloat(fsr_min_total_load, 0.01f, 1.0f)) {
        s_fsr_heel_ratio = heel / total;
    } else {
        /* 하중 불충분 → neutral 유지, FSR 퓨전 기여 없음 */
        s_fsr_heel_ratio = 0.5f;
    }
}

/* ============================================================================
 * EMG
 * ============================================================================ */

static void _SampleEmg(void)
{
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_raw_v[i] = (float)XM_AnalogReadMillivolts(s_emg_pins[i]) * 0.001f;
        if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
            s_emg_cal_sum[i] += s_emg_raw_v[i];
        }
    }
    if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
        s_emg_cal_count++;
    }
}

static void _ResetEmgFilters(void)
{
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_centered_v[i] = 0.0f;
        s_emg_pre_lpf_v[i]  = 0.0f;
        s_emg_rect_v[i]     = 0.0f;
        s_emg_envelope_v[i] = 0.0f;
        s_emg_norm_priv[i]  = 0.0f;
    }
    s_avg_emg = 0.0f;
}

static void _ProcessEmgSignals(void)
{
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_centered_v[i] = s_emg_raw_v[i] - s_emg_bias_v[i];
        s_emg_pre_lpf_v[i]  = _LowPassUpdate(s_emg_pre_lpf_v[i],
                                              s_emg_centered_v[i],
                                              EMG_RAW_LPF_CUTOFF_HZ);
        s_emg_rect_v[i]     = _AbsFloat(s_emg_pre_lpf_v[i]);
        s_emg_envelope_v[i] = _LowPassUpdate(s_emg_envelope_v[i],
                                              s_emg_rect_v[i],
                                              EMG_ENV_LPF_CUTOFF_HZ);
        s_emg_norm_priv[i]  = _GetEmgNorm(i);

        if (s_emg_cal_state == EMG_CAL_EFFORT_RUNNING) {
            if (s_emg_envelope_v[i] > s_emg_cal_max_env[i]) {
                s_emg_cal_max_env[i] = s_emg_envelope_v[i];
            }
        }
    }

    /* 퓨전 입력용 EMG 평균 갱신 */
    s_avg_emg = (s_emg_norm_priv[EMG_LH] + s_emg_norm_priv[EMG_RH]) * 0.5f;
}

static float _GetEmgNorm(int ch)
{
    float active_v;
    float span;
    float norm;

    active_v = s_emg_envelope_v[ch] - EMG_DEADBAND_V;
    if (active_v <= 0.0f) { return 0.0f; }
    span = s_emg_full_scale_v[ch] - EMG_DEADBAND_V;
    if (span < EMG_MIN_FULL_SCALE_V) { span = EMG_MIN_FULL_SCALE_V; }
    norm = active_v / span;
    return _ClampFloat(norm, 0.0f, 1.0f);
}

static void _UpdateEmgCal(const BtnEvents_t *ev)
{
    bool busy;
    bool elapsed;
    int i;

    busy = (s_emg_cal_state == EMG_CAL_REST_RUNNING ||
            s_emg_cal_state == EMG_CAL_EFFORT_RUNNING);

    if (ev->btn3 == XM_BTN_CLICK && !busy) {
        _ResetEmgCal();
        XM_SendUsbDebugMessage("[EMG CAL] reset to defaults\r\n");
        return;
    }
    if (ev->btn1 == XM_BTN_CLICK && !busy) {
        _StartEmgRestCal();
        return;
    }
    if (ev->btn2 == XM_BTN_CLICK && !busy) {
        _StartEmgEffortCal();
        return;
    }

    if (!busy) { return; }

    elapsed = ((XM_GetTick() - s_emg_cal_tick) >= EMG_CAL_DURATION_MS);
    if (!elapsed) { return; }

    if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
        if (s_emg_cal_count > 0U) {
            for (i = 0; i < (int)EMG_COUNT; i++) {
                s_emg_bias_v[i] = (float)(s_emg_cal_sum[i] /
                                          (double)s_emg_cal_count);
            }
        }
        s_emg_cal_state  = EMG_CAL_IDLE;
        emg_cal_rest_done = 1U;
        _ResetEmgFilters();
        XM_SetLedEffect(XM_LED_1, XM_LED_ONESHOT, 500);
        XM_SendUsbDebugMessage("[EMG CAL] rest done\r\n");
    } else {
        for (i = 0; i < (int)EMG_COUNT; i++) {
            float scale = s_emg_cal_max_env[i] - EMG_DEADBAND_V;
            if (scale < EMG_MIN_FULL_SCALE_V) { scale = EMG_MIN_FULL_SCALE_V; }
            s_emg_full_scale_v[i] = scale + EMG_DEADBAND_V;
        }
        s_emg_cal_state     = EMG_CAL_DONE;
        emg_cal_effort_done = 1U;
        emg_cal_done        = (emg_cal_rest_done == 1U) ? 1U : 0U;
        _ResetEmgFilters();
        XM_SetLedEffect(XM_LED_2, XM_LED_ONESHOT, 500);
        XM_SendUsbDebugMessage("[EMG CAL] effort done\r\n");
    }
}

static void _StartEmgRestCal(void)
{
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) { s_emg_cal_sum[i] = 0.0; }
    s_emg_cal_count   = 0U;
    s_emg_cal_tick    = XM_GetTick();
    s_emg_cal_state   = EMG_CAL_REST_RUNNING;
    emg_cal_rest_done = 0U;
    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 50);
    XM_SendUsbDebugMessage("[EMG CAL] rest: relax muscles for 3 s\r\n");
}

static void _StartEmgEffortCal(void)
{
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) { s_emg_cal_max_env[i] = 0.0f; }
    s_emg_cal_tick      = XM_GetTick();
    s_emg_cal_state     = EMG_CAL_EFFORT_RUNNING;
    emg_cal_effort_done = 0U;
    _ResetEmgFilters();
    XM_SetLedEffect(XM_LED_2, XM_LED_BLINK, 50);
    XM_SendUsbDebugMessage("[EMG CAL] effort: contract target muscles for 3 s\r\n");
}

static void _ResetEmgCal(void)
{
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_bias_v[i]       = EMG_DEFAULT_BIAS_V;
        s_emg_full_scale_v[i] = EMG_DEFAULT_FULL_SCALE_V;
    }
    s_emg_cal_state     = EMG_CAL_IDLE;
    emg_cal_rest_done   = 0U;
    emg_cal_effort_done = 0U;
    emg_cal_done        = 0U;
    _ResetEmgFilters();
    XM_SetLedEffect(XM_LED_3, XM_LED_ONESHOT, 500);
}

/* ============================================================================
 * ENCODER + COMPENSATION
 * ============================================================================ */

static void _SampleEncoder(void)
{
    float left_angle_rad;
    float right_angle_rad;
    float left_raw_vel;
    float right_raw_vel;
    float cutoff;
    float alpha;

    cutoff = _ClampFloat(COMP_VEL_LPF_HZ, 0.1f, 100.0f);
    alpha  = (TWO_PI * cutoff * CONTROL_DT_S) /
             (1.0f + TWO_PI * cutoff * CONTROL_DT_S);

    left_encoder_angle_deg  = XM.status.h10.leftHipMotorAngle;
    right_encoder_angle_deg = XM.status.h10.rightHipMotorAngle;

    left_control_angle_deg  =
        encoder_sign_lh * (left_encoder_angle_deg  - encoder_offset_lh_deg);
    right_control_angle_deg =
        encoder_sign_rh * (right_encoder_angle_deg - encoder_offset_rh_deg);

    left_angle_rad  = left_control_angle_deg  * DEG_TO_RAD;
    right_angle_rad = right_control_angle_deg * DEG_TO_RAD;

    left_raw_vel  = 0.0f;
    right_raw_vel = 0.0f;

    if (s_comp_prev_valid) {
        left_raw_vel  = (left_angle_rad  - s_comp_prev_left_rad)  / CONTROL_DT_S;
        right_raw_vel = (right_angle_rad - s_comp_prev_right_rad) / CONTROL_DT_S;
    } else {
        s_comp_prev_valid = true;
    }

    s_comp_prev_left_rad  = left_angle_rad;
    s_comp_prev_right_rad = right_angle_rad;

    left_angular_velocity_rads  +=
        alpha * (left_raw_vel  - left_angular_velocity_rads);
    right_angular_velocity_rads +=
        alpha * (right_raw_vel - right_angular_velocity_rads);
}

static void _UpdateCompensation(void)
{
    float left_rad;
    float right_rad;
    float deadzone;
    float scale;
    float left_fric;
    float right_fric;

    left_rad  = left_control_angle_deg  * DEG_TO_RAD;
    right_rad = right_control_angle_deg * DEG_TO_RAD;
    deadzone  = _ClampFloat(velocity_deadzone_rads, 0.0f, 10.0f);
    scale     = _ClampFloat(compensation_scale, 0.0f, 1.0f);

    left_gravity_torque_nm  = gravity_mgl_nm * sinf(left_rad);
    right_gravity_torque_nm = gravity_mgl_nm * sinf(right_rad);

    left_fric  = 0.0f;
    right_fric = 0.0f;

    if (friction_compensation_ON == 1U) {
        if (_AbsFloat(left_angular_velocity_rads) > deadzone) {
            left_fric =
                coulomb_friction_nm * _SignFloat(left_angular_velocity_rads) +
                viscous_friction_nms * left_angular_velocity_rads;
        }
        if (_AbsFloat(right_angular_velocity_rads) > deadzone) {
            right_fric =
                coulomb_friction_nm * _SignFloat(right_angular_velocity_rads) +
                viscous_friction_nms * right_angular_velocity_rads;
        }
    }

    if (compensation_ON == 1U) {
        left_compensation_torque_nm  = scale * (left_gravity_torque_nm  + left_fric);
        right_compensation_torque_nm = scale * (right_gravity_torque_nm + right_fric);
    } else {
        left_compensation_torque_nm  = 0.0f;
        right_compensation_torque_nm = 0.0f;
    }
}

/* ============================================================================
 * SQUAT FSM — 3중 퓨전 버전
 * ============================================================================
 *
 * 퓨전 신호 요약:
 *   [Enc] avg_angle         좌우 고관절 평균 각도 (하강→증가)
 *   [FSR] s_fsr_heel_ratio  heel/(heel+toe) 비율 (하강→증가, 상승→감소)
 *   [EMG] s_avg_emg         좌우 EMG norm 평균 (근육 활성→증가)
 *
 * 퓨전 점수 계산 (STAND→DESC, RETURN→STAND 전환에만 적용):
 *   score = phase_w_enc * enc_cond
 *         + phase_w_fsr * fsr_cond
 *         + phase_w_emg * emg_cond
 *   전환: score >= phase_fusion_thresh (AND 150 ms debounce)
 *
 * 나머지 전환 (DESC→BOTTOM, BOTTOM→ASC, ASC→RETURN) 은 물리적으로
 * 더 명확한 조건이므로 가중 퓨전 대신 논리적 OR/AND 조합 사용.
 *
 * ─ STAND → DESCENDING ───────────────────────────────────────────────────────
 *   enc_cond : avg_angle >= enter_threshold (기하학적 진입)
 *   fsr_cond : heel_ratio >= fsr_heel_thresh (하강 시 heel 쏠림)
 *   emg_cond : avg_emg >= emg_phase_thresh  (eccentric 제어 근활성)
 *   debounce 150 ms.
 *
 * ─ DESCENDING → BOTTOM ───────────────────────────────────────────────────────
 *   encoder 단독: avg_angle >= bottom_threshold (deep squat 진입).
 *   FSR 보조: 총 하중이 높으면 확신 → but primary는 각도.
 *   (EMG는 이 전환에서 판정 기준 미사용)
 *
 * ─ DESCENDING → RETURN (중도 복귀) ───────────────────────────────────────────
 *   avg_angle < enter_threshold (encoder 단독, 즉시).
 *
 * ─ BOTTOM → ASCENDING ────────────────────────────────────────────────────────
 *   조건 (아래 중 하나 이상):
 *     (A) 속도 반전: avg_velocity < -0.05 rad/s
 *     (B) FSR + EMG 복합: toe 우세(fsr_toe_dominant) AND emg_active
 *   (B)는 속도 센서 노이즈가 심할 때 대체 경로.
 *   에러: 너무 자주 ASCENDING 조기 진입 → (B) 를 제거하고 (A)만 남길 것.
 *
 * ─ ASCENDING → RETURN ────────────────────────────────────────────────────────
 *   avg_angle < enter_threshold (encoder 단독, 즉시).
 *
 * ─ RETURN → STAND ────────────────────────────────────────────────────────────
 *   enc_cond : avg_angle <= stand_threshold
 *   fsr_cond : heel_ratio 이 neutral zone 안 (not heel_dominant AND not toe_dominant)
 *   emg_cond : avg_emg < emg_phase_thresh (근육 이완 완료)
 *   퓨전 점수 >= phase_fusion_thresh + 150 ms debounce.
 *
 * ─ RETURN → ASCENDING (재하강) ───────────────────────────────────────────────
 *   avg_angle >= enter_threshold (encoder 단독, 즉시).
 * ============================================================================ */

static void _UpdateSquatFsm(void)
{
    float avg_angle;
    float avg_velocity;
    float w_enc, w_fsr, w_emg, f_thresh;
    float h_thresh, t_thresh, e_thresh;
    float fusion_score;
    uint32_t now;

    /* --- 퓨전 파라미터 클램프 (Live Expressions 비정상 값 방지) --- */
    w_enc    = _ClampFloat(phase_w_enc,         0.0f, 1.0f);
    w_fsr    = _ClampFloat(phase_w_fsr,         0.0f, 1.0f);
    w_emg    = _ClampFloat(phase_w_emg,         0.0f, 1.0f);
    f_thresh = _ClampFloat(phase_fusion_thresh,  0.0f, 1.0f);
    h_thresh = _ClampFloat(fsr_heel_thresh,      0.50f, 0.95f);
    t_thresh = _ClampFloat(fsr_toe_thresh,       0.05f, 0.50f);
    e_thresh = _ClampFloat(emg_phase_thresh,     0.01f, 0.99f);

    avg_angle    = (left_control_angle_deg     + right_control_angle_deg)     * 0.5f;
    avg_velocity = (left_angular_velocity_rads + right_angular_velocity_rads) * 0.5f;
    now          = XM_GetTick();

    switch (s_squat_phase) {

    /* ── STAND: 스쿼트 시작 대기 ───────────────────────────────────────────── */
    case SQUAT_STAND:
    {
        /*
         * 퓨전 조건:
         *   enc_cond = avg_angle >= enter_threshold → 기하학적으로 스쿼트 진입
         *   fsr_cond = heel_ratio >= h_thresh       → 하강 시 heel 하중 증가
         *   emg_cond = avg_emg >= e_thresh          → eccentric 근활성
         *
         * [중요] enc_cond 가 false이면 score < w_enc <= f_thresh 이므로 전환 없음.
         * [중요] enc_cond 가 true이고 fsr/emg 중 하나라도 보조하면 score > f_thresh.
         *        encoder 단독(score=w_enc=0.50)도 f_thresh=0.50 이면 통과.
         *
         * 에러: 정지 상태에서 DESCENDING 오진입
         *   → phase_fusion_thresh 를 0.60 이상으로 높일 것.
         *     (encoder 단독으로는 0.50이므로 FSR/EMG 추가 확인 강제됨)
         */
        bool enc_cond = (avg_angle >= squat_enter_threshold_deg);
        bool fsr_cond = (s_fsr_heel_ratio >= h_thresh);
        bool emg_cond = (s_avg_emg >= e_thresh);

        if (enc_cond) {
            fusion_score = w_enc
                         + w_fsr * (fsr_cond ? 1.0f : 0.0f)
                         + w_emg * (emg_cond ? 1.0f : 0.0f);

            if (fusion_score >= f_thresh) {
                if (!s_squat_debounce_active) {
                    s_squat_debounce_active = true;
                    s_squat_debounce_tick   = now;
                } else if ((now - s_squat_debounce_tick) >= SQUAT_DEBOUNCE_MS) {
                    s_squat_phase           = SQUAT_DESCENDING;
                    s_squat_debounce_active = false;
                }
            } else {
                /* 각도 조건은 맞지만 FSR/EMG가 뒷받침 못함 → debounce 리셋 */
                s_squat_debounce_active = false;
            }
        } else {
            s_squat_debounce_active = false;
        }
        break;
    }

    /* ── DESCENDING: 하강 중 ───────────────────────────────────────────────── */
    case SQUAT_DESCENDING:
    {
        /*
         * BOTTOM 진입: 각도 기준이 primary.
         *   FSR 총 하중이 높으면 실제로 바닥에 앉은 것을 확인하지만
         *   total_load 조건을 AND로 걸면 빠른 스쿼트에서 타이밍 어긋날 수 있어
         *   encoder 단독으로 판정 (원본과 동일).
         *
         * 중도 복귀: 각도가 enter_threshold 아래로 내려가면 즉시 RETURN.
         *   FSR/EMG로 재확인하면 오히려 lag 발생 가능 → encoder 단독.
         */
        bool bottom_zone = (avg_angle >= squat_bottom_threshold_deg);
        bool enter_zone  = (avg_angle >= squat_enter_threshold_deg);

        if (bottom_zone) {
            s_squat_phase           = SQUAT_BOTTOM;
            s_squat_debounce_active = false;
        } else if (!enter_zone) {
            s_squat_phase           = SQUAT_RETURN;
            s_squat_debounce_active = false;
        }
        break;
    }

    /* ── BOTTOM: 최저 자세 ─────────────────────────────────────────────────── */
    case SQUAT_BOTTOM:
    {
        /*
         * 상승 시작 판정 — 두 가지 경로:
         *   경로 (A) 속도 반전: avg_velocity < -0.05 rad/s
         *     가장 직접적인 지표. 각도 미분이기 때문에 노이즈 존재.
         *
         *   경로 (B) FSR 앞꿈치 + EMG 활성:
         *     toe_dominant (push-off 시작) AND emg_active (concentric 근활성)
         *     (A) 경로의 보완책. 속도 노이즈가 심할 때 유용.
         *
         * 에러: BOTTOM에서 바로 ASCENDING으로 오전환
         *   → (B) 경로 제거: 이 case 에서 vel_reversal 조건만 남길 것.
         *   → 또는 fsr_toe_thresh 를 더 낮게(0.35), emg_phase_thresh 를 더 높게(0.15).
         *
         * 에러: BOTTOM→ASCENDING 이 전혀 안 일어남 (계속 BOTTOM 유지)
         *   → avg_velocity 가 -0.05 미만으로 안 떨어짐.
         *     속도 임계값을 -0.02 로 완화하거나 (B) 경로만 의존.
         *   → encoder_sign 확인: 상승이 양수 속도로 보이면 조건 반전됨.
         */
        bool vel_reversal = (avg_velocity < -0.05f);
        bool toe_dominant = (s_fsr_heel_ratio <= t_thresh);
        bool emg_active   = (s_avg_emg >= e_thresh);

        if (vel_reversal || (toe_dominant && emg_active)) {
            s_squat_phase = SQUAT_ASCENDING;
        }
        break;
    }

    /* ── ASCENDING: 상승 중 (보조 토크 출력 구간) ─────────────────────────── */
    case SQUAT_ASCENDING:
    {
        /*
         * RETURN 진입: 각도가 enter_threshold 아래 → 거의 일어선 것.
         *   FSR 균형/EMG 이완을 AND로 요구하면 토크가 너무 오래 지속될 수 있어
         *   encoder 단독으로 즉시 전환 (원본과 동일).
         *
         * [중요] 이 phase 에서만 EMG 비례 보조 토크가 출력됨 (_ApplyTorque 참조).
         */
        bool enter_zone = (avg_angle >= squat_enter_threshold_deg);

        if (!enter_zone) {
            s_squat_phase           = SQUAT_RETURN;
            s_squat_debounce_active = false;
        }
        break;
    }

    /* ── RETURN: 기립 복귀 중 ──────────────────────────────────────────────── */
    case SQUAT_RETURN:
    {
        /*
         * STAND 복귀 조건 (퓨전):
         *   enc_cond = avg_angle <= stand_threshold  → 기하학적으로 기립
         *   fsr_cond = heel_ratio neutral zone 내     → 발 하중 균등 배분
         *   emg_cond = avg_emg < e_thresh (NOT active) → 근육 이완 완료
         *
         * [중요] emg_cond 는 "EMG가 낮을 때" 점수를 줌.
         *   즉 근육이 이완됐을 때 STAND 복귀를 확정함 (과잉 보조 방지).
         *
         * 에러: RETURN → STAND 전환이 안 됨
         *   → EMG가 여전히 e_thresh 이상 → 근육이 이완되지 않은 것.
         *     phase_fusion_thresh 를 낮추거나 phase_w_emg 를 0으로 설정.
         *
         * 재하강 (RETURN → ASCENDING):
         *   각도가 다시 enter_threshold 이상 → 즉시 복귀 (encoder 단독).
         */
        bool stand_zone  = (avg_angle <= squat_stand_threshold_deg);
        bool enter_zone  = (avg_angle >= squat_enter_threshold_deg);
        bool heel_neutral = (s_fsr_heel_ratio > t_thresh &&
                             s_fsr_heel_ratio < h_thresh);
        bool emg_relaxed = (s_avg_emg < e_thresh);

        if (stand_zone) {
            float enc_c = 1.0f;
            float fsr_c = heel_neutral ? 1.0f : 0.0f;
            float emg_c = emg_relaxed  ? 1.0f : 0.0f;

            fusion_score = w_enc * enc_c + w_fsr * fsr_c + w_emg * emg_c;

            if (fusion_score >= f_thresh) {
                if (!s_squat_debounce_active) {
                    s_squat_debounce_active = true;
                    s_squat_debounce_tick   = now;
                } else if ((now - s_squat_debounce_tick) >= SQUAT_DEBOUNCE_MS) {
                    s_squat_phase           = SQUAT_STAND;
                    s_squat_debounce_active = false;
                }
            } else {
                s_squat_debounce_active = false;
            }
        } else if (enter_zone) {
            /* 완전히 일어서기 전에 다시 앉음 → ASCENDING 복귀 */
            s_squat_phase           = SQUAT_ASCENDING;
            s_squat_debounce_active = false;
        } else {
            s_squat_debounce_active = false;
        }
        break;
    }

    default:
        s_squat_phase           = SQUAT_STAND;
        s_squat_debounce_active = false;
        break;
    }
}

/* ============================================================================
 * TORQUE OUTPUT  (Squat_EMG_Assist.c 와 동일)
 * ============================================================================ */

static void _ApplyTorque(void)
{
    float torque_limit;
    float base_lh;
    float base_rh;
    bool assist_requested;

    torque_limit = _ClampFloat(assist_torque_limit_nm,
                               0.0f, HARD_MAX_ASSIST_TORQUE_NM);

    assist_requested = (squat_control_ON    == 1U) &&
                       (fsr_cal_ready == 1U) &&
                       (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST);

    if (!assist_requested) {
        assist_enable                = 0U;
        left_assist_torque_nm        = 0.0f;
        right_assist_torque_nm       = 0.0f;
        left_compensation_torque_nm  = 0.0f;
        right_compensation_torque_nm = 0.0f;
        XM_SetAssistTorqueLH(0.0f);
        XM_SetAssistTorqueRH(0.0f);
        if (s_torque_mode_active) {
            XM_SetControlMode(XM_CTRL_MONITOR);
            XM_SetH10AssistExistingMode(false);
            s_torque_mode_active = false;
        }
        return;
    }

    if (!s_torque_mode_active) {
        XM_SetH10AssistExistingMode(false);
        XM_SetControlMode(XM_CTRL_TORQUE);
        s_torque_mode_active = true;
    }
    assist_enable = 1U;

    if (s_squat_phase == SQUAT_ASCENDING) {
        base_lh = s_emg_norm_priv[EMG_LH] * squat_max_torque_nm;
        base_rh = s_emg_norm_priv[EMG_RH] * squat_max_torque_nm;
    } else {
        base_lh = 0.0f;
        base_rh = 0.0f;
    }

    left_assist_torque_nm = _ClampFloat(
        base_lh + left_compensation_torque_nm,
        -torque_limit, torque_limit);
    right_assist_torque_nm = _ClampFloat(
        base_rh + right_compensation_torque_nm,
        -torque_limit, torque_limit);

    XM_SetAssistTorqueLH(torque_sign * left_assist_torque_nm);
    XM_SetAssistTorqueRH(torque_sign * right_assist_torque_nm);
}

/* ============================================================================
 * HOUSEKEEPING
 * ============================================================================ */

static void _UpdatePublicSignals(void)
{
    /* EMG raw voltages */
    pf3_volt = s_emg_raw_v[EMG_RH];
    pf4_volt = s_emg_raw_v[EMG_LH];
    /* FSR raw voltages */
    pf5_volt    = s_fsr_raw_v[FSR_LT];
    pf6_volt    = s_fsr_raw_v[FSR_LH];
    pf7_volt    = s_fsr_raw_v[FSR_RT];
    pf8_volt    = s_fsr_raw_v[FSR_RH];
    fsr_lt_load = s_fsr_load[FSR_LT];
    fsr_lh_load = s_fsr_load[FSR_LH];
    fsr_rt_load = s_fsr_load[FSR_RT];
    fsr_rh_load = s_fsr_load[FSR_RH];

    /* FSR fusion observables */
    fsr_heel_ratio = s_fsr_heel_ratio;
    fsr_total_load = s_fsr_total_load;

    /* EMG processed */
    emg_rh_raw_v      = s_emg_raw_v[EMG_RH];
    emg_lh_raw_v      = s_emg_raw_v[EMG_LH];
    emg_rh_envelope_v = s_emg_envelope_v[EMG_RH];
    emg_lh_envelope_v = s_emg_envelope_v[EMG_LH];
    emg_rh_norm       = s_emg_norm_priv[EMG_RH];
    emg_lh_norm       = s_emg_norm_priv[EMG_LH];
    avg_emg_norm      = s_avg_emg;

    /* Calibration flags */
    fsr_cal_zero_done = s_fsr_zero_done ? 1U : 0U;
    fsr_cal_full_done = s_fsr_full_done ? 1U : 0U;
    fsr_cal_ready     = (s_fsr_zero_done && s_fsr_full_done) ? 1U : 0U;

    /* System */
    assist_mode_active = (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) ? 1U : 0U;
    squat_phase        = (uint16_t)s_squat_phase;
}

static void _SendStream(void)
{
    uint32_t now;

    now = XM_GetTick();
    if (cdc_stream_enable != 1U ||
        cdc_stream_period_ms == 0U ||
        (now - s_stream_tick) < (uint32_t)cdc_stream_period_ms) {
        return;
    }
    s_stream_tick = now;

#define CDC_ASSIGN_FIELD(field, name, unit, src) s_stream.field = (float)(src);
    CDC_STREAM_CHANNELS(CDC_ASSIGN_FIELD, CDC_ASSIGN_FIELD)
#undef CDC_ASSIGN_FIELD

    XM_SendUsbDataWithId(&s_stream, sizeof(s_stream), USB_MODULE_ID);
}

/* ============================================================================
 * UTILITIES
 * ============================================================================ */

static float _LowPassUpdate(float prev, float input, float cutoff_hz)
{
    float rc;
    float alpha;
    rc    = 1.0f / (TWO_PI * cutoff_hz);
    alpha = CONTROL_DT_S / (rc + CONTROL_DT_S);
    return prev + alpha * (input - prev);
}

static float _ClampFloat(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static float _AbsFloat(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float _SignFloat(float x)
{
    if (x > 0.0f) { return  1.0f; }
    if (x < 0.0f) { return -1.0f; }
    return 0.0f;
}
