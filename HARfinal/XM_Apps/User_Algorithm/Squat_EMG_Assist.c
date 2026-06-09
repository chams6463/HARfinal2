/**
 ******************************************************************************
 * @file    Squat_EMG_Assist.c
 * @brief   EMG-proportional squat assist with bilateral FSR calibration.
 *          Built fresh from source blocks in Final_FSR_Fuzzy_Logic.c,
 *          Final_EMG.c, Final_Compensation_AddOn.c, Final_Encoder_ex01.c,
 *          Final_FSR.c.  Original files are NOT modified.
 *
 * ============================================================================
 * Pin mapping
 * ============================================================================
 *   PF3 = EMG right hip   (XM_EXT_DIO_1 -> XM_EXT_ADC_5)
 *   PF4 = EMG left hip    (XM_EXT_DIO_2 -> XM_EXT_ADC_6)
 *   PF5 = FSR left toe    (XM_EXT_DIO_3 -> XM_EXT_ADC_7)
 *   PF6 = FSR left heel   (XM_EXT_DIO_4 -> XM_EXT_ADC_8)
 *   PF7 = FSR right toe   (XM_EXT_DIO_5 -> XM_EXT_ADC_9)
 *   PF8 = FSR right heel  (XM_EXT_DIO_6 -> XM_EXT_ADC_10)
 *
 * ============================================================================
 * Operating procedure
 * ============================================================================
 *  1) Switch H10 to ASSIST mode.
 *  2) Stand unloaded, press BTN1 (PRESSED) -> FSR zero cal, hold still 1 s.
 *  3) Apply full body weight, press BTN2 (PRESSED) -> FSR load cal, hold 1 s.
 *  4) Relax muscles, press BTN1 (CLICK) -> EMG rest cal, stay relaxed 3 s.
 *  5) Contract target muscles, press BTN2 (CLICK) -> EMG effort cal, 3 s.
 *  6) Confirm fsr_cal_ready==1 and emg_cal_done==1 in Live Expressions.
 *  7) Set squat_control_ON=1 in Live Expressions to enable squat assistance.
 *  BTN3 PRESSED resets FSR cal.  BTN3 CLICK resets EMG cal.
 *
 * ============================================================================
 * Safety gate
 * ============================================================================
 *  Torque output only when:
 *    squat_control_ON == 1  AND  fsr_cal_ready == 1  AND  H10 mode == ASSIST
 *
 * ============================================================================
 * Important Live Expressions variables
 * ============================================================================
 * Write from debugger:
 *   squat_control_ON              0=disabled  1=request torque
 *   compensation_ON         0=gravity off  1=gravity+friction on
 *   friction_compensation_ON 0=gravity only  1=add friction term
 *   squat_max_torque_nm     EMG full-scale torque (default 1.0 Nm)
 *   assist_torque_limit_nm  project limit (default 1.5 Nm)
 *   encoder_offset_lh_deg / encoder_offset_rh_deg
 *   encoder_sign_lh / encoder_sign_rh
 *   gravity_mgl_nm          M*g*L model (start low, 1.0)
 *   compensation_scale      feedforward scale (default 0.20)
 *   squat_enter_threshold_deg  angle to start squat (default 15 deg)
 *   squat_bottom_threshold_deg angle for deep squat (default 45 deg)
 *   squat_stand_threshold_deg  angle to return to STAND (default 5 deg)
 *   cdc_stream_enable       0=CDC off  1=CDC on
 *   cdc_stream_period_ms    CDC period (default 10 ms = 100 Hz)
 *
 * Observe only:
 *   assist_enable           1 while torque mode is active
 *   assist_mode_active      1 while H10 ASSIST mode
 *   fsr_cal_ready           1 after both FSR cal steps done
 *   emg_cal_done            1 after both EMG cal steps done
 *   squat_phase             0=STAND 1=DESCENDING 2=BOTTOM 3=ASCENDING 4=RETURN
 *   pf3_volt/pf4_volt       raw EMG voltages  pf5~pf8_volt raw FSR voltages
 *   fsr_lt_load..fsr_rh_load calibrated FSR loads 0..1
 *   emg_rh_envelope_v / emg_lh_envelope_v
 *   emg_rh_norm / emg_lh_norm  normalised EMG 0..1
 *   left_assist_torque_nm / right_assist_torque_nm
 *
 * ============================================================================
 * CDC stream (Module ID 0xF0, 14 float channels, 100 Hz)
 * ============================================================================
 *  1) PF3 V        4) PF6 V         7) EMG RH Nrm   10) R Torque
 *  2) PF4 V        5) EMG RH Env    8) EMG LH Nrm   11) FSR Cal
 *  3) PF5 V        6) EMG LH Env    9) Squat Ph     12) Control
 *                                                   13) Assist
 * (14 channels total; see CDC_STREAM_CHANNELS macro below)
 ******************************************************************************
 */

#include "xm_api.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * CONSTANTS
 * ============================================================================
 *
 * 【조정 가이드 — 실측 후 문제 생겼을 때 건드려야 할 상수들】
 *
 * ─ FSR_LPF_CUTOFF_HZ (8 Hz) ─────────────────────────────────────────────
 *   FSR 로우패스 필터 차단주파수.
 *   에러: fsr_*_load 가 발걸음 충격에 너무 민감하게 튀면 → 낮춤 (예: 4 Hz).
 *   에러: fsr_*_load 반응이 너무 느려 캘리브레이션에 lag 생기면 → 높임 (예: 15 Hz).
 *   주의: 이 값을 바꾸면 캘리브레이션 정확도도 같이 변하므로 재캘 필요.
 *
 * ─ FSR_CAL_DURATION_MS (1000 ms) ─────────────────────────────────────────
 *   FSR 캘 동안 평균을 내는 시간.
 *   에러: 캘값이 안정되지 않고 출렁이면 → 2000으로 늘릴 것.
 *   에러: BTN 누르고 LED 깜빡임이 끝났는데 캘값이 이상하면 → 캘 중 움직인 것.
 *
 * ─ FSR_MINIMUM_SPAN_V (0.05 V) ────────────────────────────────────────────
 *   zero/full 전압 차이 최솟값. 이보다 작으면 0.05V로 강제.
 *   에러: fsr_*_load 가 수십~수백 값으로 발산하면 → span이 거의 0인 것.
 *     센서 접촉 불량 또는 zero/full 캘을 같은 조건에서 두 번 한 것.
 *
 * ─ EMG_RAW_LPF_CUTOFF_HZ (80 Hz) ─────────────────────────────────────────
 *   정류 전 노이즈 제거용 LPF. EMG 주파수 대역(20~500 Hz) 중 저주파 성분 보존.
 *   에러: envelope 에 고주파 스파이크가 심하면 → 60 Hz 로 낮출 것.
 *   주의: 너무 낮추면 EMG 신호 자체가 깎여 나가 norm 이 낮아짐.
 *
 * ─ EMG_ENV_LPF_CUTOFF_HZ (5 Hz) ──────────────────────────────────────────
 *   포락선 평활화 LPF. 너무 낮으면 응답이 느리고, 너무 높으면 노이즈 보임.
 *   에러: 근육 수축 후 norm이 0으로 너무 느리게 돌아오면 → 높임 (예: 8 Hz).
 *   에러: norm 이 흔들리면 → 낮춤 (예: 3 Hz).
 *
 * ─ EMG_DEADBAND_V (0.020 V = 20 mV) ──────────────────────────────────────
 *   이 이하 envelope 는 0으로 처리 (노이즈 컷).
 *   에러: 이완 시 norm이 0이 아닌 작은 값이 계속 나오면 → 0.030~0.050 으로 높임.
 *   에러: 살짝 수축해도 norm이 0이면 → 0.010 으로 낮출 것. 단, 노이즈 오검출 위험.
 *
 * ─ EMG_DEFAULT_BIAS_V (1.65 V) ────────────────────────────────────────────
 *   캘 안 했을 때 쓰는 기본 바이어스. 실제 센서는 1.5~1.8 V 범위.
 *   에러: 캘 전에도 EMG가 올바르게 동작하길 원하면 이 값을 실측 전압으로 수정.
 *
 * ─ EMG_DEFAULT_FULL_SCALE_V (1.000 V) ─────────────────────────────────────
 *   캘 안 했을 때 기본 full-scale. 실제 최대 envelope은 0.05~0.3 V 수준.
 *   즉 캘 없이 쓰면 norm이 최대 0.1~0.3 수준으로 매우 작게 나옴 (의도적 안전장치).
 *   에러: 캘 전인데 norm이 너무 작아서 토크가 거의 안 걸리면 → Effort 캘 먼저 수행.
 *
 * ─ COMP_VEL_LPF_HZ (5 Hz) ────────────────────────────────────────────────
 *   각속도 미분 후 적용하는 LPF.
 *   에러: angular_velocity 가 매우 노이즈하면 → 2~3 Hz 로 낮출 것.
 *   에러: 보상 토크가 진동하면 friction_compensation_ON 을 0으로 끄거나 이 값을 낮춤.
 *
 * ─ SQUAT_DEBOUNCE_MS (150 ms) ─────────────────────────────────────────────
 *   FSM 상태 전환 안정화 시간.
 *   에러: 걷다가 squat_phase 가 STAND↔DESCENDING 을 반복하면 → 250 ms 로 늘릴 것.
 *   에러: 스쿼트 시작인데 phase 전환이 너무 느리면 → 50~100 ms 로 줄일 것.
 * ============================================================================ */

#define FSR_COUNT                   4U   /* PF5=ADC_7(LT) PF6=ADC_8(LH) PF7=ADC_9(RT) PF8=ADC_10(RH) */
#define EMG_COUNT                   2U   /* PF3=ADC_5(RH EMG) PF4=ADC_6(LH EMG) */
#define CONTROL_DT_S                0.001f   /* 1 ms 루프 주기. core_process.c 에서 고정값 */
#define USB_MODULE_ID               0xF0U    /* Python Decoder 의 모듈 ID와 일치해야 함 */
#define HARD_MAX_ASSIST_TORQUE_NM   2.5f     /* 하드웨어 절대 상한. 절대 초과 안 됨 */

/* FSR */
#define FSR_LPF_CUTOFF_HZ           8.0f
#define FSR_CAL_DURATION_MS         1000U
#define FSR_MINIMUM_SPAN_V          0.05f

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

/* ============================================================================
 * ENUMERATIONS AND TYPES
 * ============================================================================ */

typedef enum { FSR_LT = 0, FSR_LH, FSR_RT, FSR_RH } FsrIdx_t; /* LT=PF5, LH=PF6, RT=PF7, RH=PF8 */
typedef enum { EMG_RH = 0, EMG_LH } EmgIdx_t;                 /* RH=PF3, LH=PF4 */

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

/* Button events captured once per loop to prevent double-consume. */
typedef struct {
    XmBtnEvent_t btn1;
    XmBtnEvent_t btn2;
    XmBtnEvent_t btn3;
} BtnEvents_t;

/* ============================================================================
 * CDC STREAM CONFIGURATION  (14 channels, Module ID 0xF0)
 * Add/remove/reorder CDC_STREAM_NEXT rows only.  Keep metadata <= 512 bytes.
 * ============================================================================ */
#define CDC_STREAM_CHANNELS(F, N)                                               \
    F(emg_rh_env,    "EMG RH Env", "V",    emg_rh_envelope_v)                  \
    N(emg_lh_env,    "EMG LH Env", "V",    emg_lh_envelope_v)                  \
    N(emg_rh_nrm,    "EMG RH Nrm", "-",    emg_rh_norm)                        \
    N(emg_lh_nrm,    "EMG LH Nrm", "-",    emg_lh_norm)                        \
    N(fsr_lt,        "FSR LT",     "-",    fsr_lt_load)                        \
    N(fsr_lh,        "FSR LH",     "-",    fsr_lh_load)                        \
    N(fsr_rt,        "FSR RT",     "-",    fsr_rt_load)                        \
    N(fsr_rh,        "FSR RH",     "-",    fsr_rh_load)                        \
    N(squat_ph,      "Squat Ph",   "id",   squat_phase)                        \
    N(l_torque,      "L Torque",   "Nm",   left_assist_torque_nm)              \
    N(r_torque,      "R Torque",   "Nm",   right_assist_torque_nm)             \
    N(fsr_cal,       "FSR Cal",    "bool", fsr_cal_ready)                      \
    N(ctrl_on,       "Control",    "bool", squat_control_ON)                   \
    N(assist_on,     "Assist",     "bool", assist_enable)

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
 * ============================================================================
 *
 * 【static 변수 구조 이해 — 디버깅 시 핵심】
 *
 * static 변수는 Live Expressions에서 직접 보이지 않음.
 * 모든 internal 상태는 루프 끝 _UpdatePublicSignals()에서 public 변수로 복사됨.
 * 즉 Live Expressions에서 값이 이상하면 → 여기 internal 배열을 의심.
 *
 * 에러 진단 시 필요하면 CubeIDE Expressions 창에
 * s_fsr_raw_v[0], s_emg_bias_v[0] 등 배열 원소를 직접 입력해서 볼 수 있음.
 * ============================================================================ */

/* --- FSR --- PF5=ADC_7(LT) PF6=ADC_8(LH) PF7=ADC_9(RT) PF8=ADC_10(RH) --- */
/* [중요] 핀 배열 순서: index 0=LT(PF5), 1=LH(PF6), 2=RT(PF7), 3=RH(PF8).
 * FsrIdx_t enum (FSR_LT/LH/RT/RH) 과 이 배열 순서가 반드시 일치해야 함.
 * 에러: 좌우 FSR 값이 뒤바뀌면 → s_fsr_pins 배열 순서가 잘못된 것. */
static const XmAdcPin_t s_fsr_pins[FSR_COUNT] = {
    XM_EXT_ADC_7, XM_EXT_ADC_8, XM_EXT_ADC_9, XM_EXT_ADC_10
};
static float        s_fsr_raw_v[FSR_COUNT];   /* ADC 직결 전압 (LPF 전) */
static float        s_fsr_lpf_v[FSR_COUNT];   /* 8Hz LPF 후 전압 (캘/하중 계산 기준) */
static float        s_fsr_zero_v[FSR_COUNT];  /* BTN1 zero 캘 결과 (무부하 전압) */
static float        s_fsr_full_v[FSR_COUNT]    = {1.0f, 1.0f, 1.0f, 1.0f};  /* BTN2 full-load 캘 결과. 기본 1V */
static float        s_fsr_load[FSR_COUNT];    /* 정규화된 하중 0~1.5 */
static bool         s_fsr_filter_init;        /* false=첫 루프(LPF 시드 필요), true=정상 동작 */
static FsrCalState_t s_fsr_cal_state;
static double       s_fsr_cal_sum[FSR_COUNT]; /* double: 1000번 누적 시 float 정밀도 부족 방지 */
static uint32_t     s_fsr_cal_count;
static uint32_t     s_fsr_cal_tick;
static bool         s_fsr_zero_done;
static bool         s_fsr_full_done;

/* --- EMG --- PF3=ADC_5(RH), PF4=ADC_6(LH) --- */
/* [중요] EMG_RH=index 0=PF3(DIO_1=ADC_5), EMG_LH=index 1=PF4(DIO_2=ADC_6).
 * 에러: 좌우 EMG가 뒤바뀌면 → s_emg_pins 배열 순서 확인. */
static const XmAdcPin_t s_emg_pins[EMG_COUNT] = {
    XM_EXT_ADC_5, XM_EXT_ADC_6   /* [0]=EMG_RH, [1]=EMG_LH */
};
static float        s_emg_raw_v[EMG_COUNT];       /* ADC 직결 전압 (~1.65V 기준) */
static float        s_emg_centered_v[EMG_COUNT];  /* 바이어스 제거 후 (≈0V 기준) */
static float        s_emg_pre_lpf_v[EMG_COUNT];   /* 80Hz LPF 후 */
static float        s_emg_rect_v[EMG_COUNT];       /* 전파정류 후 (항상 양수) */
static float        s_emg_envelope_v[EMG_COUNT];  /* 5Hz 포락선 LPF 후 */
static float        s_emg_norm_priv[EMG_COUNT];   /* 데드밴드+정규화 결과 0~1 */
/* [중요] s_emg_bias_v 초기화 크기가 3인데 EMG_COUNT=2 — 초기화 원소 3개는 실제 [0],[1]만 사용.
 * 에러: EMG_COUNT를 3 이상으로 바꾸면 배열 초기화 크기도 같이 늘려야 함. */
static float        s_emg_bias_v[EMG_COUNT]       = {EMG_DEFAULT_BIAS_V,
                                                      EMG_DEFAULT_BIAS_V,
                                                      EMG_DEFAULT_BIAS_V};
static float        s_emg_full_scale_v[EMG_COUNT] = {EMG_DEFAULT_FULL_SCALE_V,
                                                      EMG_DEFAULT_FULL_SCALE_V,
                                                      EMG_DEFAULT_FULL_SCALE_V};
static EmgCalState_t s_emg_cal_state;
static uint32_t     s_emg_cal_tick;
static uint32_t     s_emg_cal_count;
static double       s_emg_cal_sum[EMG_COUNT];        /* 3초×1000Hz = 3000샘플 누적, double로 정밀도 확보 */
static float        s_emg_cal_max_env[EMG_COUNT];    /* Effort 캘: 최대 envelope 추적 */

/* --- Encoder / compensation --- */
/* [중요] s_comp_prev_valid: false=첫 루프(속도 미분 스파이크 방지용 플래그).
 * ASSIST 모드 재진입 시 false로 리셋되므로 모드 전환 시 속도 스파이크 없음. */
static bool         s_comp_prev_valid;
static float        s_comp_prev_left_rad;
static float        s_comp_prev_right_rad;

/* --- Squat FSM --- */
static SquatPhase_t s_squat_phase;       /* 현재 스쿼트 단계 */
static uint32_t     s_squat_debounce_tick;
static bool         s_squat_debounce_active;

/* --- System --- */
/* [중요] s_assist_session_active: ASSIST 모드 첫 진입 감지용.
 * 이 플래그가 없으면 ASSIST 재진입마다 캘이 리셋되지 않음. */
static bool              s_assist_session_active;
/* [중요] s_torque_mode_active: XM_CTRL_TORQUE 모드 중복 설정 방지.
 * 에러: 1ms마다 XM_SetControlMode를 반복 호출하면 통신 부하 발생 가능. */
static bool              s_torque_mode_active;
static uint32_t          s_stream_tick;
static SquatStreamData_t s_stream;

/* ============================================================================
 * PUBLIC VARIABLES  (STM32CubeIDE Live Expressions)
 * ============================================================================ */

/* Controls — prefixed squat_ to avoid symbol collision with other files */
uint16_t squat_control_ON       = 0U;
uint16_t compensation_ON        = 1U;
uint16_t friction_compensation_ON = 0U;
uint16_t cdc_stream_enable      = 1U;
uint16_t cdc_stream_period_ms   = 10U;

/* Status flags (observe only) */
uint16_t assist_enable          = 0U;
uint16_t assist_mode_active     = 0U;
uint16_t fsr_cal_zero_done      = 0U;
uint16_t fsr_cal_full_done      = 0U;
uint16_t fsr_cal_ready          = 0U;
uint16_t emg_cal_rest_done      = 0U;
uint16_t emg_cal_effort_done    = 0U;
uint16_t emg_cal_done           = 0U;
uint16_t squat_phase            = 0U;

/* Sensor raw voltages
 * PF3/PF4 = EMG (ADC_5/6)
 * PF5~PF8 = FSR (ADC_7/8/9/10) */
float pf3_volt;   /* EMG RH raw  */
float pf4_volt;   /* EMG LH raw  */
float pf5_volt;   /* FSR LT raw  */
float pf6_volt;   /* FSR LH raw  */
float pf7_volt;   /* FSR RT raw  */
float pf8_volt;   /* FSR RH raw  */

/* FSR normalised loads */
float fsr_lt_load;
float fsr_lh_load;
float fsr_rt_load;
float fsr_rh_load;

/* EMG sensor signals */
float emg_rh_raw_v;
float emg_lh_raw_v;
float emg_rh_envelope_v;
float emg_lh_envelope_v;
float emg_rh_norm;
float emg_lh_norm;

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

/* Tuning parameters */
float encoder_offset_lh_deg        = 0.0f;
float encoder_offset_rh_deg        = 0.0f;
float encoder_sign_lh              = 1.0f;
float encoder_sign_rh              = 1.0f;
float gravity_mgl_nm               = 1.0f;
float compensation_scale           = 0.20f;
float coulomb_friction_nm          = 0.10f;
float viscous_friction_nms         = 0.01f;
float velocity_deadzone_rads       = 0.10f;
float squat_max_torque_nm          = 0.3f;   /* START LOW — increase only after direction confirmed */
float assist_torque_limit_nm       = 0.5f;   /* hard cap during initial testing */
float torque_sign                  = 1.0f;   /* set -1.0 in Live Expressions if torque direction wrong */
float squat_enter_threshold_deg    = 15.0f;
float squat_bottom_threshold_deg   = 45.0f;
float squat_stand_threshold_deg    = 5.0f;

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
static void _UpdateSquatFsm(void);
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
    /* EMG : PF3=DIO_1(RH)  PF4=DIO_2(LH)
     * FSR : PF5=DIO_3(LT) PF6=DIO_4(LH) PF7=DIO_5(RT) PF8=DIO_6(RH) */

    /* ── 센서 전원 및 핀 설정 ─────────────────────────────────────────────────
     * [에러] 전체 센서 0V → XM_SetExtPowerVoltage 누락 또는 순서 오류.
     *        반드시 XM_SwitchDioToAdc 보다 먼저 호출해야 함.
     * [에러] 특정 채널만 0V → 해당 DIO 번호의 XM_SwitchDioToAdc 호출 누락.
     *        DIO↔핀 매핑: DIO_1=PF3(EMG RH), DIO_2=PF4(EMG LH),
     *                     DIO_3=PF5(FSR LT), DIO_4=PF6(FSR LH),
     *                     DIO_5=PF7(FSR RT), DIO_6=PF8(FSR RH)
     * [에러] 값이 3.3V 고정 → 외부 5V 전원은 켜졌으나 센서 물리 연결 단선.
     * [에러] 값이 노이즈만 → 5V 전원 불안정 또는 GND 미연결.
     * [주의] XM_EXT_PWR_5V vs XM_EXT_PWR_3V3: FSR/EMG 모두 5V 필요. 3.3V로 하면
     *        ADC 입력 범위 대비 센서 출력 범위가 줄어 분해능 저하. */
    XM_SetExtPowerVoltage(XM_EXT_PWR_5V);
    XM_SwitchDioToAdc(XM_EXT_DIO_1);
    XM_SwitchDioToAdc(XM_EXT_DIO_2);
    XM_SwitchDioToAdc(XM_EXT_DIO_3);
    XM_SwitchDioToAdc(XM_EXT_DIO_4);
    XM_SwitchDioToAdc(XM_EXT_DIO_5);
    XM_SwitchDioToAdc(XM_EXT_DIO_6);

    /* ── 제어 모드 초기화 ──────────────────────────────────────────────────────
     * [에러] 전원 ON 직후 토크가 걸림 → XM_CTRL_MONITOR 설정 누락.
     * [에러] USB CDC 스트림이 안 보임 → XM_SetUsbTotalDataStream(false) + CustomMeta 설정 필요.
     *        TotalDataStream(true)로 하면 우리 스트림이 묻힐 수 있음.
     * [에러] Python Decoder에서 채널 이름이 이상하게 나옴 → USB_MODULE_ID(0xF0)가
     *        Decoder 설정과 다른 것. 또는 s_cdc_stream_meta 내용 확인.
     * [주의] XM_SetH10AssistExistingMode(false): true로 하면 H10이 기존 보조 모드를
     *        유지하려 해서 우리 토크 명령과 충돌할 수 있음. */
    XM_SetControlMode(XM_CTRL_MONITOR);
    XM_SetH10AssistExistingMode(false);
    XM_SetUsbTotalDataStream(false);
    XM_SetUsbCustomMeta(USB_MODULE_ID, s_cdc_stream_meta);

    _ResetFsrCal();
    _ResetEmgCal();

    s_stream_tick = XM_GetTick();
    XM_SendUsbDebugMessage(
        "[SQUAT] ready | FSR: BTN1-PRESS=zero  BTN2-PRESS=load  BTN3-PRESS=reset\r\n"
        "[SQUAT]        | EMG: BTN1-CLICK=rest  BTN2-CLICK=effort  BTN3-CLICK=reset\r\n");
}

void User_Loop(void)
{
    BtnEvents_t ev;
    bool is_assist_mode;

    /* ── XM_IO_Update() ─────────────────────────────────────────────────────
     * [에러] 버튼이 전혀 반응 없음 → 이 함수 호출 누락.
     * [에러] 버튼이 간헐적으로 반응 → 이 함수가 루프 중간에 있어서 타이밍 어긋남.
     *        반드시 루프 최상단 첫 번째 호출이어야 함.
     * [에러] LED가 설정한 대로 안 켜짐 → 역시 이 함수가 먼저 실행되지 않은 것. */
    XM_IO_Update();

    /* ── 센서 샘플링 (모드 무관, 항상 실행) ──────────────────────────────────
     * [중요] 이 세 함수는 ASSIST 조건 분기 바깥에 있어야 함 (의도적 설계).
     *   이유 1: MONITOR 모드에서도 Live Expressions에서 센서 값이 보여야 캘 확인 가능.
     *   이유 2: FSR 캘 누적(s_fsr_cal_sum)과 EMG bias 누적(s_emg_cal_sum)이
     *            _SampleFsr/_SampleEmg 내부에서 일어나므로 반드시 매 루프 호출 필요.
     * [에러] 센서 값이 ASSIST 모드 진입 전까지 0으로 보임
     *   → _Sample 함수들이 is_assist_mode 체크 안쪽으로 이동된 것. */
    _SampleFsr();
    _SampleEmg();
    _SampleEncoder();

    is_assist_mode = (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST);

    /* ------------------------------------------------------------------ */
    /* Not in ASSIST mode: flush control state, zero torque, stream data  */
    /* ------------------------------------------------------------------ */
    /* ── ASSIST 모드가 아닐 때 처리 ─────────────────────────────────────────
     * [에러] MONITOR 모드인데 토크 느껴짐 → _ApplyTorque 내부 안전 게이트 h10Mode 조건 누락.
     * [에러] 모드 전환 후 Live Expressions 값들이 갱신 안 됨
     *   → _UpdatePublicSignals()/_SendStream() 호출이 이 블록에 없는 것.
     * [주의] s_assist_session_active 플래그: ASSIST→비ASSIST 전환이 일어날 때
     *   딱 한 번만 캘 리셋을 실행하는 gate임. 매 루프 리셋되면 안 됨. */
    if (!is_assist_mode) {
        if (s_assist_session_active) {
            /* [중요] raw/lpf 버퍼(s_fsr_raw_v, s_fsr_lpf_v)는 절대 지우지 않음.
             * 지우면 MONITOR 모드에서 Live Expressions가 0V를 보여줌.
             * 캘 상태만 리셋해서 다음 ASSIST 세션에서 새로 캘하게 함. */
            s_fsr_cal_state        = FSR_CAL_IDLE;
            s_fsr_zero_done        = false;
            s_fsr_full_done        = false;
            _ResetEmgCal();
            s_squat_phase          = SQUAT_STAND;
            s_squat_debounce_active = false;
            /* [중요] s_comp_prev_valid=false: ASSIST 재진입 시 첫 루프 속도 스파이크 방지 */
            s_comp_prev_valid      = false;
            s_assist_session_active = false;
            XM_SendUsbDebugMessage("[SQUAT] session ended\r\n");
        }
        _ApplyTorque();
        _UpdatePublicSignals();
        _SendStream();
        return;
    }

    /* ── ASSIST 모드 최초 진입 초기화 ───────────────────────────────────────
     * [에러] ASSIST 진입마다 캘이 리셋됨 → s_assist_session_active 플래그가
     *        모드 이탈 시 false로 안 돌아가는 것 (위 비ASSIST 블록 확인).
     * [에러] ASSIST 진입 후 이전 캘값이 그대로 남아있음 → 이 블록이 실행 안 되는 것.
     *        s_assist_session_active 초기값이 이미 true인 경우 (전역 변수 오염 의심).
     * [주의] USB 디버그 메시지 "[SQUAT] session started"가 뜨는지 시리얼 터미널로 확인.
     *        이게 안 뜨면 ASSIST 모드 전환 자체가 안 된 것. */
    if (!s_assist_session_active) {
        _ResetFsrCal();
        _ResetEmgCal();
        s_squat_phase          = SQUAT_STAND;
        s_squat_debounce_active = false;
        s_comp_prev_valid      = false;
        s_assist_session_active = true;
        XM_SendUsbDebugMessage("[SQUAT] session started\r\n");
    }

    /* ── 버튼 이벤트 캡처 ────────────────────────────────────────────────────
     * [중요] BTN 이벤트는 루프 내에서 단 한 번만 읽어야 함.
     *   XM_GetButtonEvent()를 _UpdateFsrCal과 _UpdateEmgCal에서 각각 직접 호출하면
     *   하나의 CLICK이 두 함수에서 동시에 소비되어 의도치 않은 동작 발생.
     * [에러] BTN1 CLICK이 FSR 캘과 EMG 캘에 동시에 반응함 → 각 함수에서 직접
     *        XM_GetButtonEvent를 호출하고 있는 것. ev 구조체를 통해서만 전달해야 함. */
    ev = _ReadButtons();

    /* ── ASSIST 모드 제어 파이프라인 (실행 순서 절대 변경 금지) ──────────────
     *
     * 순서 의존성 요약:
     *   _SampleFsr()     → s_fsr_raw_v/lpf 갱신 (위에서 이미 실행)
     *   _SampleEmg()     → s_emg_raw_v 갱신 + rest 캘 누적 (위에서 이미 실행)
     *   _SampleEncoder() → 각도/속도 갱신 (위에서 이미 실행)
     *   _UpdateFsrCal()  → LPF 값 이용해 캘 완료 판단 (SampleFsr 이후여야 함)
     *   _UpdateLoads()   → 캘값 이용해 정규화 (UpdateFsrCal 이후여야 함)
     *   _ProcessEmgSignals() → raw_v 이용해 envelope 계산 (SampleEmg 이후여야 함)
     *   _UpdateEmgCal()  → envelope 이용해 full_scale 산출 (ProcessEmg 이후여야 함)
     *   _UpdateCompensation() → 각도/속도 이용 (SampleEncoder 이후여야 함)
     *   _UpdateSquatFsm()  → 각도 이용 (SampleEncoder 이후여야 함)
     *   _ApplyTorque()   → norm, phase, compensation 모두 사용 (위 모두 완료 후)
     *   _UpdatePublicSignals() → 최종 내부값을 public으로 복사 (ApplyTorque 이후)
     *   _SendStream()    → public 변수로 CDC 전송 (UpdatePublicSignals 이후)
     *
     * [에러] 파이프라인 순서를 바꾸면 1루프 lag 또는 stale 데이터로 동작 이상 발생. */

    /* (1) FSR 캘 상태머신 — 버튼 이벤트 소비 + 1초 타이머 완료 판단 */
    _UpdateFsrCal(&ev);

    /* (2) 정규화 FSR 하중 계산 */
    _UpdateLoads();

    /* (3) EMG 신호 처리
     * [중요] REST 캘 진행 중에는 필터를 리셋(0으로) 하고 ProcessEmgSignals 호출 안 함.
     *   이유: 바이어스 측정 기준인 raw 전압이 필터로 왜곡되면 안 되기 때문.
     *   s_emg_cal_sum 에는 raw_v가 누적되고 있음 (_SampleEmg 에서).
     * [에러] Rest 캘 중 envelope 값이 올라감 → _ResetEmgFilters 호출이 빠진 것. */
    if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
        _ResetEmgFilters();
    } else {
        _ProcessEmgSignals();
    }

    /* (4) EMG 캘 상태머신 — 버튼 이벤트 소비 + 3초 타이머 완료 판단 */
    _UpdateEmgCal(&ev);

    /* (5) 중력/마찰 보상 토크 계산 */
    _UpdateCompensation();

    /* (6) 스쿼트 FSM 단계 판정 */
    _UpdateSquatFsm();

    /* (7) 안전 게이트 + 토크 출력 */
    _ApplyTorque();

    /* (8) public 변수 동기화 (Live Expressions에 반영) */
    _UpdatePublicSignals();

    /* (9) CDC USB 스트림 전송 */
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
    /* [중요] XM_AnalogReadMillivolts 는 mV 단위 반환 → 0.001 곱해서 V로 변환.
     * [에러 진단] pf5~pf8_volt 값이 정상 범위(0~3.3V)보다 1000배 크게 나오면:
     *   → 0.001f 변환 계수가 빠진 것. */
    const float rc    = 1.0f / (TWO_PI * FSR_LPF_CUTOFF_HZ);
    const float alpha = CONTROL_DT_S / (rc + CONTROL_DT_S);
    int i;

    for (i = 0; i < (int)FSR_COUNT; i++) {
        s_fsr_raw_v[i] = (float)XM_AnalogReadMillivolts(s_fsr_pins[i]) * 0.001f;
        /* [중요] 첫 루프에서는 LPF를 현재 raw 값으로 초기화(seed).
         * 이 처리 없으면 0에서 시작한 LPF가 실제 값으로 수렴하는 데 수백ms 걸림.
         * [에러 진단] 전원 ON 직후 FSR 값이 0에서 천천히 올라오면:
         *   → s_fsr_filter_init 플래그가 제대로 동작하지 않는 것. */
        if (!s_fsr_filter_init) {
            s_fsr_lpf_v[i] = s_fsr_raw_v[i];
        } else {
            s_fsr_lpf_v[i] += alpha * (s_fsr_raw_v[i] - s_fsr_lpf_v[i]);
        }
    }
    s_fsr_filter_init = true;

    /* [중요] 캘리브레이션 누적은 _UpdateFsrCal이 아닌 여기서 발생.
     * _UpdateFsrCal에서 1초 경과 후 이 합계를 count로 나눠 평균 산출.
     * [에러 진단] FSR 캘리브레이션 값이 이상하게 나오면:
     *   → 캘 시작 시 s_fsr_cal_sum이 0으로 초기화됐는지 _StartFsrCal() 확인. */
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

    /* ── FSR 캘리브레이션 버튼 처리 ──────────────────────────────────────────
     *
     * 버튼 이벤트 종류 구분 (혼동 주의):
     *   XM_BTN_PRESSED = 꾹 누름 (버튼 누른 순간 이벤트)
     *   XM_BTN_CLICK   = 짧게 눌렀다 뗌 (FSR 캘에서는 사용 안 함)
     *   XM_BTN_HOLD    = 길게 누른 상태 유지
     *
     * FSR 캘은 PRESSED, EMG 캘은 CLICK 사용 — 헷갈리면 안 됨!
     *
     * [에러] 버튼을 눌러도 캘이 시작 안 됨:
     *   케이스 1: busy==true → 이전 캘이 아직 진행 중. fsr_cal_zero_done/full_done 확인.
     *   케이스 2: ASSIST 모드가 아님 → 이 함수 자체가 호출 안 됨.
     *   케이스 3: XM_BTN_PRESSED 대신 XM_BTN_CLICK으로 눌린 것 (너무 빠르게 눌렀다 뗌).
     *
     * [에러] LED가 깜빡이기 시작했는데 1초 후 완료가 안 됨:
     *   → 캘 중 FSR이 들렸거나 발을 들면서 진동이 심해 LPF 값이 수렴 안 된 것.
     *   → FSR_CAL_DURATION_MS를 2000으로 늘릴 것.
     *
     * [에러] BTN3(리셋)이 반응 안 함:
     *   → 캘 진행 중(busy==true)이면 BTN3 리셋은 무시됨 (의도적).
     *      캘이 끝날 때까지 기다린 후 BTN3 누를 것. */

    /* BTN3 PRESSED -> reset (only when idle) */
    if (ev->btn3 == XM_BTN_PRESSED && !busy) {
        _ResetFsrCal();
        XM_SendUsbDebugMessage("[FSR CAL] reset to defaults\r\n");
        return;
    }
    /* BTN1 PRESSED -> zero calibration */
    /* [중요] Zero 캘 조건: 기립 자세에서 FSR에 체중이 실리지 않은 상태여야 함.
     * 발뒤꿈치를 들거나 앞꿈치만 닿은 상태로 캘하면 안 됨.
     * 에러: zero_v 값이 너무 높음 → 이미 체중이 올려진 상태에서 BTN1을 누른 것. */
    if (ev->btn1 == XM_BTN_PRESSED && !busy) {
        _StartFsrCal(FSR_CAL_ZERO_RUNNING);
        XM_SendUsbDebugMessage("[FSR CAL] zero: stand unloaded for 1 s\r\n");
        return;
    }
    /* BTN2 PRESSED -> full-load calibration (requires zero done first) */
    /* [중요] Full-load 캘 조건: 양발에 체중을 고르게 싣고 BTN2 누름.
     *   에러: fsr_lt_load와 fsr_rt_load 값이 캘 후 크게 다름 →
     *     full-load 캘 시 좌우 무게 분포가 달랐던 것.
     *     같은 사람이 같은 자세로 다시 캘할 것.
     *
     *   에러: "do BTN1 zero before BTN2 load" 뜨면 → s_fsr_zero_done==false.
     *     BTN1 캘이 1초 완료된 후(LED ONESHOT) BTN2 누를 것. */
    if (ev->btn2 == XM_BTN_PRESSED && !busy) {
        if (s_fsr_zero_done) {
            _StartFsrCal(FSR_CAL_LOAD_RUNNING);
            XM_SendUsbDebugMessage("[FSR CAL] full load: apply weight for 1 s\r\n");
        } else {
            XM_SendUsbDebugMessage("[FSR CAL] do BTN1 zero before BTN2 load\r\n");
        }
        return;
    }

    if (!busy) {
        return;
    }

    elapsed = ((XM_GetTick() - s_fsr_cal_tick) >= FSR_CAL_DURATION_MS);
    if (!elapsed) {
        return;
    }

    /* ── 캘 완료: 1초 평균값 저장 ─────────────────────────────────────────────
     * s_fsr_cal_sum 은 double, count 도 uint32_t → float로 나눠 저장.
     * [에러] s_fsr_cal_count == 0 이면 저장 안 함 (XM_GetTick 이상 동작 방지).
     * [에러] 캘값이 계속 달라짐 → 캘 중 발에 진동/충격이 가해진 것.
     *        LPF 덕분에 어느 정도 평활화되지만 완전히 막진 못함.
     *        캘 중 최대한 정지 상태 유지할 것.
     * [에러] 양쪽 채널 중 한쪽만 캘값이 이상함 →
     *        해당 FSR 물리 접촉 불량 또는 s_fsr_pins 배열 순서 오류. */
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
    /* [중요] 두 캘(zero+full)이 모두 완료됐을 때만 FSR_CAL_DONE 으로 전환.
     * fsr_cal_ready 는 _UpdatePublicSignals에서 이 두 flag를 보고 최종 확정됨. */
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
    /* ── FSR 캘 리셋: 캘 상태만 초기화, 센서 버퍼는 보존 ─────────────────────
     * [중요] s_fsr_raw_v, s_fsr_lpf_v 는 절대 지우지 않음.
     *   이유: ASSIST 모드 이탈/재진입 또는 BTN3 리셋 시 Live Expressions에서
     *         센서가 갑자기 0V로 보이면 안 됨.
     *
     * [중요] s_fsr_filter_init = false 로 리셋:
     *   다음 _SampleFsr 호출 시 현재 raw값으로 LPF를 재시드(seed)함.
     *   안 하면 이전 LPF값이 그대로 남아 새 캘값에 lag 발생.
     *
     * [에러] 리셋 후 fsr_cal_ready가 여전히 1로 보임 →
     *   fsr_cal_ready = 0U; 라인 확인. _UpdatePublicSignals에서도 덮어쓰므로
     *   다음 루프에서는 0으로 확정됨.
     *
     * [에러] 리셋했는데도 load 값이 이전 캘 기준으로 계속 나옴 →
     *   s_fsr_zero_v, s_fsr_full_v 가 제대로 초기화됐는지 확인.
     *   (zero=0, full=1.0이 기본값) */
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
    XM_SetLedEffect(XM_LED_3, XM_LED_ONESHOT, 500);
}

static void _UpdateLoads(void)
{
    /* ── 정규화 하중 계산 ────────────────────────────────────────────────────
     *   공식: load = (lpf_v - zero_v) / (full_v - zero_v)
     *
     * [에러] load가 음수 → lpf_v < zero_v. Zero 캘 시 체중이 실려있었던 것.
     *   (클램프로 0에 고정되므로 출력에는 음수 안 나옴, 그러나 캘 재수행 필요.)
     *
     * [에러] load가 항상 1.0 이상:
     *   케이스 1: Zero 캘 시 이미 체중이 올라가 있었음 → 재캘.
     *   케이스 2: 체중이 예상보다 많이 실려 full_v를 초과 → 정상 (임시과부하).
     *
     * [에러] load가 0.0~0.1 수준으로 너무 작음:
     *   → span이 너무 큰 것. Full 캘 시 한쪽 발에만 힘을 주거나 tiptoe 상태였음.
     *   → 또는 FSR 센서가 신발 안에서 미끄러져 압력을 못 받는 것.
     *
     * [에러] 4개 채널 중 하나만 0.0 고정:
     *   → 해당 FSR 센서 물리 단선 또는 연결 불량.
     *   → pf5~pf8_volt 확인. 특정 채널이 항상 같은 전압이면 하드웨어 문제.
     *
     * [에러] 좌우 값이 뒤바뀜 (왼발 밟으면 rt_load가 올라감):
     *   → s_fsr_pins 배열 순서 오류. LT=ADC_7, LH=ADC_8, RT=ADC_9, RH=ADC_10 확인. */
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
 * EMG
 * ============================================================================ */

static void _SampleEmg(void)
{
    /* ── EMG ADC 샘플링 + Rest 캘 누적 ──────────────────────────────────────
     *
     * 정상 동작 시 기대값:
     *   emg_rh_raw_v = 1.5 ~ 1.8 V (근육 수축 없을 때 중간 바이어스 전압)
     *   emg_lh_raw_v = 1.5 ~ 1.8 V
     *   두 채널이 완전히 같은 값은 아님 (센서 개별 특성 차이).
     *
     * [에러] raw_v = 0.0 V 정확히:
     *   → XM_SwitchDioToAdc(DIO_1/2) 호출 안 됨. User_Setup 확인.
     *   → 또는 XM_AnalogReadMillivolts 가 0 반환 (핀 오입력).
     *
     * [에러] raw_v = 3.3 V 고정:
     *   → ADC 레퍼런스 전압 이상 또는 센서 전원 연결 오류.
     *   → GND 연결 확인.
     *
     * [에러] raw_v 가 랜덤하게 0V ↔ 정상값 교대:
     *   → 커넥터 접촉 불량. 물리 연결 확인.
     *
     * [에러] 좌우 raw_v 가 둘 다 똑같은 값:
     *   → 두 채널이 같은 ADC 핀을 공유하는 것. s_emg_pins 배열 확인.
     *
     * [중요] Rest 캘 중 raw_v 가 누적됨 (s_emg_cal_sum).
     *   _ProcessEmgSignals 가 이 기간에는 호출 안 되므로
     *   바이어스 측정에 필터 왜곡 없음. */
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
    /* ── EMG 필터 내부 상태 전부 0으로 초기화 ────────────────────────────────
     * 호출 시점:
     *   1) Rest 캘 진행 중 매 루프 (바이어스 측정 중 envelope 억제 목적)
     *   2) Rest 캘 완료 직후 (새 바이어스 기준으로 필터 재시작)
     *   3) Effort 캘 시작 시 (이전 envelope 잔류값 제거)
     *   4) EMG 캘 전체 리셋 시
     *
     * [에러] Rest 캘 중 emg_rh_envelope_v 가 0이 아닌 값:
     *   → 이 함수가 Rest 캘 중 매 루프 호출되지 않는 것.
     *     User_Loop의 EMG_CAL_REST_RUNNING 분기 확인.
     *
     * [에러] Effort 캘 시작 후 초반에 갑자기 norm이 1.0:
     *   → 이전 envelope 잔류값이 남아 있는 것. _StartEmgEffortCal에서
     *     _ResetEmgFilters 호출 확인.
     *
     * [주의] 필터 리셋 후 첫 몇 ms 동안 envelope는 0에서 올라오는 과도 구간.
     *   Effort 캘의 max 추적은 이 과도 구간 이후에 피크가 나와야 정확.
     *   피험자가 BTN2 클릭 후 즉시 최대 수축하면 과도 구간과 겹칠 수 있음.
     *   → 클릭 후 1~2초 후에 힘 주도록 안내. */
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_centered_v[i] = 0.0f;
        s_emg_pre_lpf_v[i]  = 0.0f;
        s_emg_rect_v[i]     = 0.0f;
        s_emg_envelope_v[i] = 0.0f;
        s_emg_norm_priv[i]  = 0.0f;
    }
}

static void _ProcessEmgSignals(void)
{
    /* ── EMG 신호 처리 파이프라인 ───────────────────────────────────────────
     *
     * 단계별 기대 신호값 (정상 수축 시):
     *   raw_v       : ~1.65 V (바이어스) + ±0.02~0.1V 교류 성분
     *   centered_v  : ~0 V 기준 ±0.02~0.1 V 교류
     *   pre_lpf_v   : centered에서 80Hz 이상 제거 (크기 약간 감소)
     *   rect_v      : 항상 양수, 0~0.1 V
     *   envelope_v  : 5Hz LPF 후 평활화, 0~0.05 V
     *   norm        : 0.0~1.0
     *
     * [에러] 이완 시 envelope 가 0 아닌 상태 유지 (오프셋):
     *   케이스 1: s_emg_bias_v 가 실제 기준 전압과 다름 → Rest 캘 재수행.
     *   케이스 2: EMG 센서 전극이 피부에서 들뜬 상태 → 전극 부착 확인.
     *   케이스 3: 주변 전기 노이즈(60Hz 상용전원 간섭) → 접지 확인, 센서선 차폐.
     *
     * [에러] 수축 시 norm이 0.0에서 전혀 안 올라옴:
     *   케이스 1: Effort 캘 미수행 → full_scale = 기본 1.0V = norm 극히 낮음.
     *   케이스 2: 바이어스 오차로 centered가 전부 음수 → rect 후 0으로 평균.
     *   케이스 3: 전극 위치가 근복(belly)이 아닌 힘줄 위 → 신호 강도 매우 낮음.
     *   케이스 4: 수축 강도가 너무 약해서 deadband(20mV) 아래 → 더 강하게 수축.
     *
     * [에러] norm이 항상 1.0 고정:
     *   → full_scale 이 너무 작은 것. Effort 캘 시 힘을 거의 안 준 것.
     *   → Effort 캘 재수행 (최대 자발적 수축 MVC 기준으로).
     *
     * [에러] 좌우 norm 차이가 심함 (한쪽이 훨씬 큼):
     *   → 정상일 수 있음 (근육 발달 차이).
     *   → 또는 한쪽 전극이 더 잘 붙어 있는 것. 전극 부착 상태 비교.
     *   → 각 채널이 개별적으로 캘되므로 캘값이 다를 수 있음.
     *     s_emg_full_scale_v[0], s_emg_full_scale_v[1] 을 Expressions에서 직접 확인. */
    int i;
    for (i = 0; i < (int)EMG_COUNT; i++) {
        /* 바이어스 제거: raw에서 rest 캘로 산출한 기준 전압을 뺌 */
        s_emg_centered_v[i] = s_emg_raw_v[i] - s_emg_bias_v[i];
        /* 80 Hz LPF before rectification */
        s_emg_pre_lpf_v[i] = _LowPassUpdate(s_emg_pre_lpf_v[i],
                                             s_emg_centered_v[i],
                                             EMG_RAW_LPF_CUTOFF_HZ);
        /* Full-wave rectification */
        s_emg_rect_v[i] = _AbsFloat(s_emg_pre_lpf_v[i]);
        /* 5 Hz envelope LPF */
        s_emg_envelope_v[i] = _LowPassUpdate(s_emg_envelope_v[i],
                                              s_emg_rect_v[i],
                                              EMG_ENV_LPF_CUTOFF_HZ);
        /* Deadband + normalise to 0..1 */
        s_emg_norm_priv[i] = _GetEmgNorm(i);

        /* [중요] Effort 캘 중 피크 추적: 평균이 아닌 최댓값 사용.
         * 근육 수축의 최대 강도를 기준으로 정규화하기 위함. */
        if (s_emg_cal_state == EMG_CAL_EFFORT_RUNNING) {
            if (s_emg_envelope_v[i] > s_emg_cal_max_env[i]) {
                s_emg_cal_max_env[i] = s_emg_envelope_v[i];
            }
        }
    }
}

static float _GetEmgNorm(int ch)
{
    /* ── EMG 정규화 ────────────────────────────────────────────────────────
     *   공식: norm = clamp( (envelope - deadband) / (full_scale - deadband), 0, 1 )
     *
     * 수치 예:
     *   deadband = 0.020 V, full_scale = 0.080 V (Effort 캘 후 전형적인 값)
     *   envelope = 0.050 V → norm = (0.050-0.020)/(0.080-0.020) = 0.030/0.060 = 0.50
     *   envelope = 0.010 V → norm = 0 (deadband 이하)
     *   envelope = 0.100 V → norm = 1.0 (clamp)
     *
     * [에러] norm이 항상 0:
     *   → emg_rh_envelope_v 가 0.020 V 를 못 넘는 것.
     *     envelope 값 자체를 먼저 확인. 0이면 신호 파이프라인 문제.
     *     0.001~0.015 수준이면 deadband를 0.010으로 낮출 것.
     *
     * [에러] 캘 없이 norm이 지나치게 낮음 (< 0.05):
     *   → s_emg_full_scale_v[ch] 가 기본값 1.0V 인 것.
     *     Effort 캘 후 이 값이 0.05~0.15V 수준으로 바뀌어야 함.
     *     Expressions에서 s_emg_full_scale_v 배열 직접 확인.
     *
     * [주의] EMG_MIN_FULL_SCALE_V(0.05V): Effort 캘 시 거의 힘을 안 줘도
     *   full_scale이 0.05 아래로는 안 내려가게 하는 클램프.
     *   norm이 극단적으로 높게 뜨는 것을 방지함. */
    float active_v;
    float span;
    float norm;

    active_v = s_emg_envelope_v[ch] - EMG_DEADBAND_V;
    if (active_v <= 0.0f) {
        return 0.0f;
    }
    span = s_emg_full_scale_v[ch] - EMG_DEADBAND_V;
    if (span < EMG_MIN_FULL_SCALE_V) {
        span = EMG_MIN_FULL_SCALE_V;
    }
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

    /* ── EMG 캘리브레이션 버튼 처리 ──────────────────────────────────────────
     *
     * 필수 순서: Rest 캘(BTN1 CLICK) → Effort 캘(BTN2 CLICK)
     *
     * [에러] emg_cal_done 이 0으로 남음:
     *   케이스 1: Effort 캘은 했는데 Rest 캘을 안 함 → Rest 먼저.
     *   케이스 2: Rest 캘 완료 후 ASSIST 모드를 이탈했다가 재진입 →
     *             _ResetEmgCal() 호출로 emg_cal_rest_done 이 0 으로 리셋.
     *             두 캘을 같은 ASSIST 세션 안에서 연속으로 수행해야 함.
     *
     * [에러] Rest 캘 중 다른 동작(몸을 움직임)을 해서 바이어스가 이상하게 잡힘:
     *   → BTN1 CLICK 후 3초 동안 완전히 정지 상태 유지.
     *   → LED1이 50ms 주기로 빠르게 깜빡이는 동안 수집 중임.
     *
     * [에러] Effort 캘 중 최대값이 너무 낮음 (norm이 여전히 작음):
     *   → 3초 내에 최대 수축을 했는지 확인.
     *   → BTN2 CLICK 후 0.5~1초 지연 후 수축 시작하는 것이 좋음 (필터 과도 구간 대기).
     *
     * [에러] BTN1 CLICK이 눌렸는데 FSR zero 캘이 시작됨:
     *   → FSR 캘은 PRESSED, EMG 캘은 CLICK → 버튼 누름 길이 확인.
     *     XM_BTN_PRESSED: 버튼 누르는 순간 / XM_BTN_CLICK: 짧게 눌렀다 뗌.
     *
     * [주의] busy 체크: 3초 진행 중에는 다른 BTN 이벤트 모두 무시됨. */

    /* BTN3 CLICK -> reset EMG cal */
    if (ev->btn3 == XM_BTN_CLICK && !busy) {
        _ResetEmgCal();
        XM_SendUsbDebugMessage("[EMG CAL] reset to defaults\r\n");
        return;
    }
    /* BTN1 CLICK -> rest calibration */
    if (ev->btn1 == XM_BTN_CLICK && !busy) {
        _StartEmgRestCal();
        return;
    }
    /* BTN2 CLICK -> effort calibration */
    if (ev->btn2 == XM_BTN_CLICK && !busy) {
        _StartEmgEffortCal();
        return;
    }

    if (!busy) {
        return;
    }

    elapsed = ((XM_GetTick() - s_emg_cal_tick) >= EMG_CAL_DURATION_MS);
    if (!elapsed) {
        return;
    }

    if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
        /* ── Rest 캘 완료: 평균 바이어스 저장 ──────────────────────────────
         * s_emg_cal_sum / s_emg_cal_count = 3초 동안 raw_v 의 평균값.
         * [에러] 완료 후 bias_v 가 1.65V 에서 크게 벗어남 (예: 0.5 V):
         *   → 캘 중 근육을 수축했거나 전극이 들뜬 것. 다시 수행.
         * [에러] 두 채널 bias 가 0.2V 이상 차이남:
         *   → 정상 범위 내. 센서 개별 특성 차이임.
         *     그러나 한쪽이 0V면 해당 채널 연결 불량. */
        if (s_emg_cal_count > 0U) {
            for (i = 0; i < (int)EMG_COUNT; i++) {
                s_emg_bias_v[i] = (float)(s_emg_cal_sum[i] /
                                          (double)s_emg_cal_count);
            }
        }
        s_emg_cal_state  = EMG_CAL_IDLE;
        emg_cal_rest_done = 1U;
        /* [중요] 새 바이어스 기준으로 필터 재시작 (이전 centered 잔류값 제거) */
        _ResetEmgFilters();
        XM_SetLedEffect(XM_LED_1, XM_LED_ONESHOT, 500);
        XM_SendUsbDebugMessage("[EMG CAL] rest done\r\n");
    } else {
        /* ── Effort 캘 완료: 최대 envelope → full_scale 저장 ─────────────
         * full_scale = max_env (3초 동안 추적한 최댓값).
         * [중요] 최댓값 기반이므로 3초 동안 단 한 번 강하게 수축해도 됨.
         *   그러나 5Hz LPF의 응답 시간(약 200ms) 때문에 피크 반영에 지연 있음.
         *   → BTN2 클릭 후 500ms 정도 후에 최대 수축 권장.
         *
         * [에러] s_emg_cal_max_env 가 0.0:
         *   → _StartEmgEffortCal 에서 0으로 초기화됐는데 EMG 신호가 전혀 없는 것.
         *     envelope 자체를 확인. _ProcessEmgSignals 호출 중이어야 함.
         *   → Effort 캘 중 EMG_CAL_REST_RUNNING 분기에 걸리지 않는지 확인.
         *
         * [에러] full_scale 이 너무 작아서 clamp됨 (EMG_MIN_FULL_SCALE_V = 0.05):
         *   → 실제 최대 envelope가 0.05 + deadband = 0.07 V 미만인 것.
         *     전극 위치를 근복으로 옮기거나 더 강하게 수축.
         *
         * [중요] emg_cal_done 은 rest_done이 1일 때만 1이 됨 — 순서 강제. */
        for (i = 0; i < (int)EMG_COUNT; i++) {
            float scale = s_emg_cal_max_env[i] - EMG_DEADBAND_V;
            if (scale < EMG_MIN_FULL_SCALE_V) {
                scale = EMG_MIN_FULL_SCALE_V;
            }
            s_emg_full_scale_v[i] = scale + EMG_DEADBAND_V;
        }
        s_emg_cal_state    = EMG_CAL_DONE;
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
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_cal_sum[i] = 0.0;
    }
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
    for (i = 0; i < (int)EMG_COUNT; i++) {
        s_emg_cal_max_env[i] = 0.0f;
    }
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
    /* ── 엔코더 각도/속도 샘플링 ────────────────────────────────────────────
     *
     * 데이터 소스: XM.status.h10.leftHipMotorAngle / rightHipMotorAngle
     *   → CAN 버스를 통해 H10 모터 드라이버에서 수신.
     *   → _FetchAllInputs() 에서 갱신 (core_process.c, User_Loop 호출 전).
     *
     * [에러] left_encoder_angle_deg 가 고정값 (freeze):
     *   → CAN 수신 패킷이 안 오는 것. h10 드라이버 전원/CAN 버스 확인.
     *   → 전원 재시작(H10 포함) 으로 CAN 재초기화.
     *   → 동작 중 갑자기 freeze: 과부하로 인한 CAN 에러 프레임 누적 가능성.
     *
     * [에러] 각도 부호가 반대 (앉으면 음수):
     *   → encoder_sign_lh = -1.0 으로 Live Expressions에서 수정.
     *   → 좌우 부호가 다를 수 있음 (H10 좌우 모터 장착 방향 차이).
     *
     * [에러] 서 있을 때 각도가 0이 아님 (예: +30도에서 시작):
     *   → encoder_offset_lh_deg = 30.0 으로 설정.
     *   → 이후 control_angle_deg = sign * (raw - offset) → 0도 기준.
     *
     * [에러] 각속도(angular_velocity_rads)가 매우 노이즈:
     *   → 미분은 노이즈 증폭이 심함. COMP_VEL_LPF_HZ 를 2~3Hz로 낮출 것.
     *   → 또는 velocity_deadzone_rads 를 0.2~0.3으로 높여 소신호 마찰 보상 제거.
     *
     * [에러] ASSIST 모드 재진입 직후 속도값이 순간 튀고 가라앉음:
     *   → 정상. s_comp_prev_valid=false 로 리셋되어 첫 루프에서 raw_vel=0
     *     처리됨. 그러나 LPF 값이 이전 잔류값에서 0으로 수렴하는 과도 구간 있음.
     *
     * [에러] left/right 각도가 같은 값만 나옴:
     *   → leftHipMotorAngle / rightHipMotorAngle 가 같은 레지스터를 가리키는 것.
     *     xm_api.h 에서 두 필드 확인. */
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

    /* ── 중력/마찰 보상 토크 계산 ───────────────────────────────────────────
     *
     * 중력 보상 모델: T_gravity = M*g*L * sin(θ)
     *   여기서 gravity_mgl_nm = M(kg) * g(9.81) * L(m) = 질량×중력×팔길이.
     *   기본값 1.0 Nm는 테스트 시작값. 실제 피험자에 맞게 조정 필요.
     *   스쿼트 중 고관절 각도 θ에 따라 토크가 sinusoidal하게 변함.
     *
     * [에러] 기립 시(θ≈0°) 보상 토크가 0 아닌 값:
     *   → control_angle_deg 가 0이 아닌 것. encoder_offset 조정 필요.
     *   → sin(0) = 0 이어야 함.
     *
     * [에러] 보상 토크 방향이 반대 (스쿼트 시 저항으로 느껴짐):
     *   케이스 1: encoder_sign 부호 반대 → sin(-θ) = -sin(θ).
     *   케이스 2: torque_sign = -1.0으로 설정 필요.
     *   진단: left_gravity_torque_nm 값이 스쿼트 시 양수인지 음수인지 확인.
     *
     * [에러] 보상 토크가 너무 강해서 피험자가 뒤로 밀림:
     *   → compensation_scale 을 0.05~0.10으로 낮출 것.
     *   → gravity_mgl_nm 도 낮출 것. 처음엔 compensation_scale=0.1부터.
     *
     * [에러] 마찰 보상(friction_compensation_ON=1) 후 토크 진동 발생:
     *   → 각속도 노이즈 × viscous_friction_nms = 토크 노이즈.
     *   → velocity_deadzone_rads 를 높이거나 friction 보상을 꺼야 함.
     *   → 기본값 OFF (friction_compensation_ON=0) 로 두고 필요 시에만 켬.
     *
     * [중요] compensation_ON=0 이면 보상 토크 전부 0.
     *   EMG base 토크만 나옴. 안전 테스트 초기엔 보상 끈 상태로 시작 권장. */
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
        left_compensation_torque_nm  =
            scale * (left_gravity_torque_nm  + left_fric);
        right_compensation_torque_nm =
            scale * (right_gravity_torque_nm + right_fric);
    } else {
        left_compensation_torque_nm  = 0.0f;
        right_compensation_torque_nm = 0.0f;
    }
}

/* ============================================================================
 * SQUAT FSM
 * ============================================================================
 *
 * Phase transitions (based on average left+right hip angle):
 *
 *   STAND      -> DESCENDING : avg_angle >= squat_enter_threshold  (150 ms debounce)
 *   DESCENDING -> BOTTOM     : avg_angle >= squat_bottom_threshold
 *   DESCENDING -> RETURN     : avg_angle < squat_enter_threshold   (aborted rep)
 *   BOTTOM     -> ASCENDING  : avg_velocity < -0.05 rad/s          (reversing up)
 *   ASCENDING  -> RETURN     : avg_angle < squat_enter_threshold
 *   RETURN     -> STAND      : avg_angle <= squat_stand_threshold   (150 ms debounce)
 *   RETURN     -> ASCENDING  : avg_angle >= squat_enter_threshold   (re-descend guard)
 *
 * Torque is applied during ASCENDING only.
 * ============================================================================ */

static void _UpdateSquatFsm(void)
{
    float avg_angle;
    float avg_velocity;
    uint32_t now;
    bool enter_zone;
    bool bottom_zone;
    bool stand_zone;

    /* ── 스쿼트 FSM 입력 계산 ────────────────────────────────────────────────
     *
     * avg_angle: 좌우 고관절 평균 각도 (기립 0°, 스쿼트 시 양수 증가).
     * avg_velocity: 각도 변화율 (하강 시 양수, 상승 복귀 시 음수).
     *
     * [에러] avg_angle이 스쿼트해도 0 근처에서 변하지 않음:
     *   케이스 1: 두 엔코더 모두 freeze → CAN 문제.
     *   케이스 2: encoder_sign이 반대라 좌우가 서로 상쇄 → 한쪽만 확인.
     *   케이스 3: encoder_offset이 잘못 설정되어 부호 반전된 것.
     *   진단 방법: left_control_angle_deg 와 right_control_angle_deg 를
     *              각각 Live Expressions에서 스쿼트하며 변화 확인.
     *
     * [에러] avg_velocity가 항상 0:
     *   → s_comp_prev_valid가 true가 된 후에도 각도 변화가 0인 것.
     *     각도 자체가 freeze되어 있는 것. 엔코더 CAN 확인.
     *
     * [FSM 전환 조건 요약 — 문제 생기면 임계값 조정]:
     *   STAND→DESC  : avg_angle >= squat_enter_threshold (기본 15°) + 150ms 지속
     *   DESC→BOTTOM : avg_angle >= squat_bottom_threshold (기본 45°)
     *   DESC→RETURN : avg_angle < squat_enter (조기 복귀)
     *   BOTTOM→ASC  : avg_velocity < -0.05 rad/s (방향 반전 감지)
     *   ASC→RETURN  : avg_angle < squat_enter (거의 일어선 것)
     *   RETURN→STAND: avg_angle <= squat_stand_threshold (기본 5°) + 150ms
     *   RETURN→ASC  : avg_angle >= squat_enter (중간에 다시 앉는 경우 대비)
     *
     * [에러] STAND→DESC 전환이 안 됨:
     *   → squat_enter_threshold(15°)가 너무 큼. 실제 시작 각도를 확인하고 낮출 것.
     *
     * [에러] BOTTOM→ASC 전환이 안 됨 (상승 시작해도 BOTTOM 유지):
     *   → avg_velocity가 -0.05 기준을 못 넘는 것. 두 경우:
     *     (a) 각속도 노이즈로 -0.05 미만이 되지 않음 → 임계값을 -0.02로 완화.
     *     (b) encoder_sign 때문에 실제 상승이 양수 속도로 보임 → sign 확인.
     *
     * [에러] ASC→RETURN 전환이 안 됨 (torque가 끝나지 않음):
     *   → 일어선 후에도 avg_angle이 enter_threshold(15°) 아래로 안 내려옴.
     *   → encoder_offset이 과도하게 설정되거나 기립 자세가 완전하지 않은 것.
     *   → squat_enter_threshold 를 더 크게 설정해서 조건을 쉽게 빠져나오게 할 것. */
    avg_angle    = (left_control_angle_deg      + right_control_angle_deg)      * 0.5f;
    avg_velocity = (left_angular_velocity_rads  + right_angular_velocity_rads)  * 0.5f;
    now          = XM_GetTick();

    enter_zone  = (avg_angle >= squat_enter_threshold_deg);
    bottom_zone = (avg_angle >= squat_bottom_threshold_deg);
    stand_zone  = (avg_angle <= squat_stand_threshold_deg);

    switch (s_squat_phase) {

    case SQUAT_STAND:
        if (enter_zone) {
            if (!s_squat_debounce_active) {
                s_squat_debounce_active = true;
                s_squat_debounce_tick   = now;
            } else if ((now - s_squat_debounce_tick) >= SQUAT_DEBOUNCE_MS) {
                s_squat_phase           = SQUAT_DESCENDING;
                s_squat_debounce_active = false;
            }
        } else {
            s_squat_debounce_active = false;
        }
        break;

    case SQUAT_DESCENDING:
        if (bottom_zone) {
            s_squat_phase           = SQUAT_BOTTOM;
            s_squat_debounce_active = false;
        } else if (!enter_zone) {
            /* Aborted before reaching bottom */
            s_squat_phase           = SQUAT_RETURN;
            s_squat_debounce_active = false;
        }
        break;

    case SQUAT_BOTTOM:
        /* Wait for reversal (angle decreasing -> ascending) */
        if (avg_velocity < -0.05f) {
            s_squat_phase = SQUAT_ASCENDING;
        }
        break;

    case SQUAT_ASCENDING:
        if (!enter_zone) {
            s_squat_phase           = SQUAT_RETURN;
            s_squat_debounce_active = false;
        }
        break;

    case SQUAT_RETURN:
        if (stand_zone) {
            if (!s_squat_debounce_active) {
                s_squat_debounce_active = true;
                s_squat_debounce_tick   = now;
            } else if ((now - s_squat_debounce_tick) >= SQUAT_DEBOUNCE_MS) {
                s_squat_phase           = SQUAT_STAND;
                s_squat_debounce_active = false;
            }
        } else {
            /* Re-descent before fully standing -> back to ascending */
            if (enter_zone) {
                s_squat_phase           = SQUAT_ASCENDING;
                s_squat_debounce_active = false;
            } else {
                s_squat_debounce_active = false;
            }
        }
        break;

    default:
        s_squat_phase           = SQUAT_STAND;
        s_squat_debounce_active = false;
        break;
    }
}

/* ============================================================================
 * TORQUE OUTPUT  (safety gate + XM API)
 * ============================================================================ */

static void _ApplyTorque(void)
{
    float torque_limit;
    float base_lh;
    float base_rh;
    bool assist_requested;

    torque_limit = _ClampFloat(assist_torque_limit_nm,
                               0.0f, HARD_MAX_ASSIST_TORQUE_NM);

    /* ── 안전 게이트 ────────────────────────────────────────────────────────
     *
     * 3개 조건 AND:
     *   [1] squat_control_ON == 1  : 사용자가 Live Expressions에서 명시적으로 1 설정
     *   [2] fsr_cal_ready == 1     : FSR zero + full 두 단계 캘 완료
     *   [3] H10 mode == ASSIST     : 하드웨어가 실제로 ASSIST 상태
     *
     * [에러] assist_enable = 0 → 토크 없음. 진단 순서:
     *   step1: assist_mode_active 확인 → 0이면 H10 모드 전환 안 된 것.
     *   step2: fsr_cal_ready 확인 → 0이면 FSR 캘 미완료.
     *   step3: squat_control_ON 확인 → 0이면 Live Expressions에서 1로 설정.
     *
     * [에러] assist_enable = 1 인데 토크가 전혀 안 느껴짐:
     *   케이스 1: squat_phase ≠ ASCENDING(3) → EMG base 토크는 ASCENDING만.
     *     compensation_ON=1 이라면 보상 토크는 모든 active 상태에서 나옴.
     *   케이스 2: squat_max_torque_nm = 0.3 너무 작음. 느껴지지 않을 수 있음.
     *   케이스 3: emg_rh_norm / emg_lh_norm = 0 → EMG 캘 안 됐거나 수축 안 함.
     *   케이스 4: torque_sign 부호 반전으로 기계 저항으로 작용 (H10 임피던스 모드).
     *
     * [에러] 모드 전환 직후 갑자기 강한 토크:
     *   → s_torque_mode_active 플래그 때문에 XM_CTRL_TORQUE 설정은 한 번만 됨.
     *     그런데 squat_control_ON이 1이고 fsr_cal_ready=1이면 즉시 토크 시작.
     *     → 캘 완료 후 squat_control_ON을 1로 설정하는 타이밍 조절.
     *
     * [에러] 안전 게이트 통과했는데 XM_SetAssistTorqueLH 가 실제 모터 출력 안 됨:
     *   → XM_SetControlMode(XM_CTRL_TORQUE) 가 설정됐는지 확인.
     *     s_torque_mode_active=false 상태에서 첫 진입 시에만 설정.
     *
     * [중요] fsr_cal_ready: _UpdatePublicSignals에서 갱신되지만 _ApplyTorque 이후에 실행.
     *   → 실제 게이트는 이전 루프의 fsr_cal_ready 사용 (1ms lag, 무해). */
    assist_requested = (squat_control_ON     == 1U) &&
                       (fsr_cal_ready  == 1U) &&
                       (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST);

    if (!assist_requested) {
        /* ── 토크 출력 금지: 0으로 클리어 ──────────────────────────────────
         * [중요] XM_SetAssistTorqueLH(0) 을 매 루프 명시적으로 호출.
         *   이 호출이 없으면 H10가 마지막 토크 명령을 유지할 수 있음.
         * [중요] CTRL_TORQUE 상태였다면 CTRL_MONITOR 로 복귀.
         *   복귀 안 하면 H10가 0Nm 토크 명령을 계속 실행 → 브레이크 효과. */
        assist_enable               = 0U;
        left_assist_torque_nm       = 0.0f;
        right_assist_torque_nm      = 0.0f;
        left_compensation_torque_nm = 0.0f;
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

    /* ── 토크 모드 첫 진입 설정 ────────────────────────────────────────────
     * [중요] XM_SetControlMode(XM_CTRL_TORQUE) 는 한 번만 호출 (s_torque_mode_active 플래그).
     *   매 루프 반복 호출은 불필요한 통신 부하 유발 가능.
     * [에러] 토크 모드 전환 후 H10가 응답 안 함:
     *   → XM_SetH10AssistExistingMode(false) 가 먼저 와야 함. 순서 확인. */
    if (!s_torque_mode_active) {
        XM_SetH10AssistExistingMode(false);
        XM_SetControlMode(XM_CTRL_TORQUE);
        s_torque_mode_active = true;
    }
    assist_enable = 1U;

    /* ── EMG 비례 보조 토크 (ASCENDING 구간에서만) ───────────────────────────
     *
     * base = emg_norm(0~1) * squat_max_torque_nm
     * 최종 = clamp(base + compensation, -limit, +limit) * torque_sign
     *
     * [에러] ASCENDING인데 base 토크가 0:
     *   케이스 1: emg_rh_norm = emg_lh_norm = 0 → EMG 수축 없거나 캘 미수행.
     *   케이스 2: squat_max_torque_nm = 0 → Live Expressions에서 0으로 설정됨.
     *   케이스 3: s_squat_phase가 ASCENDING(3)이 아님 → squat_phase 확인.
     *
     * [중요] 보상 토크(compensation_torque_nm)는 ASCENDING 여부와 무관하게 더해짐.
     *   즉 안전 게이트 통과 후 STAND/DESCENDING/BOTTOM에서도 보상 토크는 출력됨.
     *   이 보상만으로도 토크가 느껴질 수 있음. 초기 테스트 시 compensation_ON=0 권장.
     *
     * [에러] 토크가 지나치게 강해서 위험함:
     *   → squat_max_torque_nm 을 0.1 Nm부터 시작.
     *   → assist_torque_limit_nm 을 추가 안전망으로 사용.
     *   → HARD_MAX_ASSIST_TORQUE_NM(2.5 Nm)은 절대 초과 불가.
     *
     * [에러] 토크가 비대칭 (한쪽만 강함):
     *   → emg_rh_norm vs emg_lh_norm 값 차이 확인.
     *   → 또는 s_emg_full_scale_v[0] vs [1] 이 많이 다른 것.
     *   → Effort 캘 시 양쪽 근육을 동등하게 수축했는지 확인. */
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

    /* ── 최종 토크 clamp 및 부호 적용 ──────────────────────────────────────
     * left/right_assist_torque_nm = clamp(base + comp, -limit, +limit)
     *
     * [에러] 토크가 limit(기본 0.5 Nm)으로 고정:
     *   → 입력 합계가 limit을 초과. squat_max_torque_nm 또는 gravity_mgl_nm 낮출 것.
     *
     * [에러] 토크 방향 반대 (저항으로 느껴짐):
     *   케이스 1: torque_sign = -1.0 으로 Live Expressions 수정.
     *   케이스 2: compensation 토크 방향 문제 → compensation_ON=0 해서 base 토크만 확인.
     *   케이스 3: 두 케이스를 구분: ASCENDING에서 squat_max_torque_nm만 켠 상태로 확인.
     *
     * [중요] torque_sign은 left/right 동시에 적용됨.
     *   좌우가 각각 반대 방향이면 (비대칭 장착) torque_sign으로는 해결 안 됨.
     *   → encoder_sign_lh/rh 로 각 채널 개별 보정 필요.
     *
     * [에러] XM_SetAssistTorqueLH 호출 후 실제 모터에서 응답 없음:
     *   → s_torque_mode_active=true + XM_CTRL_TORQUE 상태 확인.
     *   → H10 통신 상태 확인 (CAN 연결). */
    XM_SetAssistTorqueLH(torque_sign * left_assist_torque_nm);
    XM_SetAssistTorqueRH(torque_sign * right_assist_torque_nm);
}

/* ============================================================================
 * HOUSEKEEPING
 * ============================================================================ */

static void _UpdatePublicSignals(void)
{
    /* ── public 변수 동기화 (Live Expressions 전용) ─────────────────────────
     *
     * [구조 이해] 실제 계산은 모두 static(s_*) 변수로 이루어짐.
     *   public 변수는 관찰/디버그 전용이며 루프 끝에 한 번만 복사.
     *   → 실시간 제어 로직에서 public 변수를 직접 사용하면 안 됨.
     *
     * [에러] Live Expressions 값이 갱신 안 됨:
     *   → 이 함수 호출이 빠진 것. User_Loop 파이프라인 마지막 확인.
     *
     * [에러] pf3_volt/pf4_volt 와 emg_rh_raw_v/lh_raw_v 가 같은 값:
     *   → 정상. 둘 다 s_emg_raw_v[0/1] 에서 복사함.
     *
     * [에러] fsr_cal_ready 가 1이 됐는데 토크가 다음 루프부터 나옴:
     *   → 정상. _ApplyTorque 이후에 이 함수가 실행되므로 1루프 lag 발생. */

    /* EMG raw voltages (PF3/PF4) */
    pf3_volt = s_emg_raw_v[EMG_RH];
    pf4_volt = s_emg_raw_v[EMG_LH];
    /* FSR raw voltages (PF5~PF8) */
    pf5_volt   = s_fsr_raw_v[FSR_LT];
    pf6_volt   = s_fsr_raw_v[FSR_LH];
    pf7_volt   = s_fsr_raw_v[FSR_RT];
    pf8_volt   = s_fsr_raw_v[FSR_RH];
    fsr_lt_load = s_fsr_load[FSR_LT];
    fsr_lh_load = s_fsr_load[FSR_LH];
    fsr_rt_load = s_fsr_load[FSR_RT];
    fsr_rh_load = s_fsr_load[FSR_RH];

    /* EMG processed */
    emg_rh_raw_v      = s_emg_raw_v[EMG_RH];
    emg_lh_raw_v      = s_emg_raw_v[EMG_LH];
    emg_rh_envelope_v = s_emg_envelope_v[EMG_RH];
    emg_lh_envelope_v = s_emg_envelope_v[EMG_LH];
    emg_rh_norm       = s_emg_norm_priv[EMG_RH];
    emg_lh_norm       = s_emg_norm_priv[EMG_LH];

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
    /* ── CDC USB 스트림 전송 ─────────────────────────────────────────────────
     *
     * [에러] Python Decoder에서 아무 데이터도 안 들어옴:
     *   케이스 1: cdc_stream_enable = 0 → 1로 설정.
     *   케이스 2: cdc_stream_period_ms = 0 → 0으로 설정하면 전송 안 됨.
     *   케이스 3: USB_MODULE_ID(0xF0) 가 Decoder 설정과 다름.
     *   케이스 4: USB 케이블 연결 불량 또는 USB CDC 드라이버 미설치.
     *   케이스 5: XM_SetUsbTotalDataStream(true)로 되어 있어 우리 스트림이 묻힘.
     *
     * [에러] 데이터가 주기적으로 끊김 (dropout):
     *   → USB 버퍼가 가득 찬 것. cdc_stream_period_ms 를 20~50으로 늘릴 것.
     *   → 또는 Python Decoder가 데이터를 처리하는 속도가 느린 것.
     *
     * [에러] 채널 값이 뒤섞임 (EMG에 FSR 값이 나옴):
     *   → CDC_STREAM_CHANNELS 매크로에서 채널 순서 확인.
     *   → Decoder의 채널 정의와 이 매크로 순서가 일치해야 함.
     *
     * [에러] 채널 수가 Decoder에서 다르게 보임:
     *   → CDC_STREAM_CHANNELS의 F/N 라인 개수(=채널 수) 와
     *     Decoder 설정이 다른 것. 현재 14채널.
     *
     * [중요] s_stream 구조체는 float 배열이므로 sizeof(s_stream) = 14 * 4 = 56 바이트.
     *   XM_SendUsbDataWithId 의 크기 인자에 sizeof 사용으로 자동 계산. */
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
    /* ── 1차 IIR 저역통과 필터 (단일 루프 호출용) ───────────────────────────
     *   공식: y[n] = y[n-1] + α*(x[n] - y[n-1])
     *   α = dt / (RC + dt),  RC = 1 / (2π * cutoff_hz)
     *
     *   cutoff_hz 가 높을수록 α→1 → 거의 raw 값 통과 (필터 약함).
     *   cutoff_hz 가 낮을수록 α→0 → 느리게 추종 (필터 강함).
     *
     * [에러] 필터 출력이 입력과 같음 (필터 효과 없음):
     *   → cutoff_hz 가 너무 높은 것. α≈1 이 되는 것.
     *   → 예: cutoff=500Hz, dt=0.001s → α = 0.001/(0.000318+0.001) ≈ 0.76 (약한 필터).
     *
     * [에러] 필터 출력이 항상 0에 머뭄 (반응 없음):
     *   → prev 초기값 문제. _ResetXxxFilters()에서 0으로 초기화 후
     *     실제 신호가 들어오면 서서히 수렴함. 과도 구간 정상.
     *
     * [주의] CONTROL_DT_S = 0.001f (1ms) 가 실제 루프 주기와 다르면
     *   필터 특성이 설계값과 달라짐. 루프 주기가 바뀌면 이 값도 수정. */
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
