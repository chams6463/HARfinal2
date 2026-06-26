/**
 ******************************************************************************
 * @file    Squat_EMG_Assist2.c
 * @brief   [v2] User-personalized squat assist (lookup-based, dwell BOTTOM, 500ms ramp).
 *          Squat_EMG_Assist.c 의 사본 — 원본은 그대로 보존.
 *
 *          핵심 변경점 vs v1:
 *            1) DESCENDING EMG-비례 토크 제거 (compensation만 작용)
 *            2) ASCENDING/BOTTOM(HOLD) 토크를 사용자 EMG baseline 룩업으로 산출
 *               - MaxTorque0.csv 분석 기반, sticking point(15-25°)에 max 보조
 *               - compensation 몫은 룩업이 미리 차감해 둠
 *            3) BOTTOM 진입을 각도 임계 대신 sliding-window dwell-time 으로 인식
 *               (사용자가 더 내려가려는 의도면 시스템이 BOTTOM 으로 안 빠짐)
 *            4) BOTTOM/ASCENDING 토크 인가 시 500ms ramp-in
 *               (Reinkensmeyer 2009 인간 응답 지연 ~500ms 매치) */
 *
 * ============================================================================
 * Pin mapping
 * ============================================================================
 *   PF3 = EMG right hip   (XM_EXT_DIO_1 -> XM_EXT_ADC_5)
 *   PF4 = EMG left hip    (XM_EXT_DIO_2 -> XM_EXT_ADC_6)
 *   (FSR pins PF5~PF8 not used)
 *
 * ============================================================================
 * Operating procedure
 * ============================================================================
 *  1) Switch H10 to ASSIST mode.
 *  2) Relax muscles, press BTN1 (CLICK) -> EMG rest cal, stay relaxed 3 s.
 *  3) Contract target muscles, press BTN2 (CLICK) -> EMG effort cal, 3 s.
 *  4) Confirm emg_cal_done==1 in Live Expressions.
 *  5) Set squat_control_ON=1 in Live Expressions to enable squat assistance.
 *  BTN3 CLICK resets EMG cal.
 *
 * ============================================================================
 * Safety gate
 * ============================================================================
 *  Torque output only when:
 *    squat_control_ON == 1  AND  H10 mode == ASSIST
 *
 * ============================================================================
 * Important Live Expressions variables
 * ============================================================================
 * Write from debugger:
 *   [Master control]
 *   squat_control_ON              0=disabled  1=request torque
 *   compensation_ON               0=gravity off  1=gravity(+friction) on
 *   friction_compensation_ON      0=gravity only  1=add Coulomb+viscous term
 *
 *   [EMG calibration trigger — write 1 to start, auto-cleared on accept]
 *   emg_cal_rest_request          1 → start REST cal (3 s, stay relaxed)
 *   emg_cal_effort_request        1 → start EFFORT cal (3 s, max contraction)
 *   emg_cal_reset_request         1 → revert to offline cal defaults
 *
 *   [Torque magnitude / safety]
 *   squat_max_torque_nm           [v2] 룩업 전역 게인 (default 6.0 = 룩업 그대로,
 *                                  0.0 → 룩업 비활성=comp만, hard cap 10.0)
 *   assist_torque_limit_nm        절대 허용 상한 (default 10.0, hard cap 10.0)
 *   torque_sign                   +1.0 / -1.0 (방향 반전용)
 *
 *   [Encoder zeroing]
 *   encoder_offset_lh_deg / encoder_offset_rh_deg  기립 자세 raw 각도 입력 (default 30.0)
 *   encoder_sign_lh / encoder_sign_rh              +1.0 / -1.0
 *
 *   [Compensation model]
 *   gravity_mgl_nm                M*g*L (default 8.0)  ※ 룩업 테이블이 이 값 기준으로 차감됨
 *   compensation_scale            comp 출력 비율 0~1 (default 0.50)  ※ 룩업 설계 기준값
 *   coulomb_friction_nm           default 0.10
 *   viscous_friction_nms          default 0.01
 *   velocity_deadzone_rads        마찰 보상 소신호 컷 (default 0.10)
 *
 *   [Squat FSM thresholds]
 *   squat_enter_threshold_deg     STAND→DESC 임계 (default 15)
 *   squat_bottom_threshold_deg    [v2] 안전 가드용 (BOTTOM 진입은 dwell 기반)  (default 45)
 *   squat_stand_threshold_deg     RETURN→STAND 임계 (default 5)
 *   emg_ascend_threshold          [v2] BOTTOM→ASC EMG 강제 임계 0~1 (default 0.5, 0=비활성)
 *
 *   [Ramp tuning]
 *   assist_ramp_time_ms           [v2] base 토크 0→target 선형 ramp 지속시간 ms
 *                                  (default 500, 권장 100~500, 0=즉시 step)
 *                                  ASCEND 가 빠르면 200~300ms 권장
 *   torque_slew_nm_per_ms         [v2-fix] 출력 토크 최대 변화율 (Nm/ms)
 *                                  (default 0.02 = 20 Nm/s, 권장 0.01~0.05)
 *                                  phase 전이 step 차단 → 모터 과전류 트립 방지
 *                                  너무 낮추면 응답 둔함, 너무 높이면 보호 다시 트립
 *
 *   [Dwell tuning — BOTTOM 인식 파라미터]
 *   dwell_angle_dead_deg          HOLD 진입 임계 (default 1.5°, 좁으면 흔들림에 약함)
 *   dwell_threshold_ms            dwell 윈도우 길이 (default 200, 짧으면 false HOLD)
 *   dwell_exit_angle_deg          HOLD 탈출 임계 (default 2.5°, 진입보다 커야 함)
 *
 *   [CDC stream]
 *   cdc_stream_enable             0=CDC off  1=CDC on
 *   cdc_stream_period_ms          CDC period (default 10 ms = 100 Hz)
 *
 * Observe only:
 *   assist_enable                 1 while torque mode is active
 *   assist_mode_active            1 while H10 ASSIST mode
 *   emg_cal_rest_done             1 after REST cal accepted
 *   emg_cal_effort_done           1 after EFFORT cal accepted
 *   emg_cal_done                  1 only after BOTH cal steps done (순서: rest → effort)
 *   squat_phase                   0=STAND 1=DESCENDING 2=BOTTOM 3=ASCENDING 4=RETURN
 *   pf3_volt / pf4_volt           raw EMG voltages (PF3=RH, PF4=LH)
 *   emg_rh_raw_v / emg_lh_raw_v   동일 raw (별칭)
 *   emg_rh_envelope_v / emg_lh_envelope_v   5 Hz LPF envelope
 *   emg_rh_norm / emg_lh_norm     normalised EMG 0..1
 *   left_encoder_angle_deg / right_encoder_angle_deg  raw motor shaft angle (누적)
 *   left_control_angle_deg / right_control_angle_deg  offset/sign 적용 후 joint angle
 *   left_angular_velocity_rads / right_angular_velocity_rads  5 Hz LPF 적용 후
 *   left_gravity_torque_nm / right_gravity_torque_nm           M*g*L*sin(θ)
 *   left_compensation_torque_nm / right_compensation_torque_nm scale*(gravity+fric)
 *   left_assist_torque_nm / right_assist_torque_nm             최종 명령 토크 (clamp 후)
 *
 * ============================================================================
 * CDC stream (Module ID 0xF0, 8 float channels, 100 Hz)
 * ============================================================================
 *  1) EMG RH Env   3) EMG RH Nrm   5) Squat Ph   7) R Torque
 *  2) EMG LH Env   4) EMG LH Nrm   6) L Torque   8) Control
 * (8 channels total; see CDC_STREAM_CHANNELS macro below)
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
 * ─ encoder_offset_lh_deg / rh_deg (기본 0.0) ────────────────────────────
 *   leftHipMotorAngle 은 전원 ON 이후 누적값이라 초기값이 0이 아님.
 *   기립 자세에서 left_encoder_angle_deg 를 Live Expressions 로 읽어
 *   encoder_offset_lh_deg 에 그 값을 입력 → control_angle 이 0이 됨.
 *   값이 크게 튀는 이유: offset 미설정으로 누적값 그대로 보임.
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
 * ─ EMG_OFFLINE_CAL_BIAS_V (1.592 V) ───────────────────────────────────────
 *   피험자별 사전 측정한 EMG bias (offline calibration).
 *   런타임 Rest 캘 미수행 시 이 값이 그대로 신호 처리에 사용됨.
 *   실제 센서는 1.5~1.8 V 범위. 피험자 변경 시 사전 측정 후 이 값 갱신 필요.
 *
 * ─ EMG_OFFLINE_CAL_FULL_SCALE_V (0.750 V) ─────────────────────────────────
 *   피험자별 사전 측정한 MVC envelope full-scale (offline calibration).
 *   런타임 Effort 캘 미수행 시 이 값이 정규화 분모로 사용됨.
 *   실제 최대 envelope은 0.05~0.3 V 수준이나 피험자에 따라 다름.
 *   에러: norm이 너무 작으면 → 실측값 갱신하거나 런타임 Effort 캘 수행.
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


#define EMG_COUNT                   2U   /* PF3=ADC_5(RH EMG) PF4=ADC_6(LH EMG) */
#define CONTROL_DT_S                0.001f   /* 1 ms 루프 주기. core_process.c 에서 고정값 */
#define USB_MODULE_ID               0xF0U    /* Python Decoder 의 모듈 ID와 일치해야 함 */
#define HARD_MAX_ASSIST_TORQUE_NM   10.0f    /* 하드웨어 절대 상한. 절대 초과 안 됨 */

/* EMG */
#define EMG_RAW_LPF_CUTOFF_HZ       80.0f
#define EMG_ENV_LPF_CUTOFF_HZ       5.0f
#define EMG_DEADBAND_V              0.020f
/* 실측 데이터 (xm10_4ch_20260609_210543.csv) 기반 고정 범위:
 *   raw min = 1.592 V (휴식 노이즈 최저)  →  bias 기준점
 *   raw max = 3.130 V (최대 수축 순간 피크)
 *   최대 centered 진폭 = 3.130 - 1.592 = 1.538 V
 *   5Hz 포락선 추정값 ≈ 1.538 / 2 = 0.769 V  →  full_scale 0.750 V */
/* Offline (pre-measured) calibration constants.
 * 피험자별 사전 측정값. 버튼/런타임 캘 미수행 시 이 값이 활성 캘 값으로 사용됨.
 * 피험자 변경 시 사전 측정 후 갱신할 것. */
#define EMG_OFFLINE_CAL_BIAS_V          1.592f
#define EMG_OFFLINE_CAL_FULL_SCALE_V    0.750f
#define EMG_MIN_FULL_SCALE_V        0.050f
#define EMG_CAL_DURATION_MS         3000U

/* Encoder / compensation */
#define DEG_TO_RAD                  0.01745329252f
#define TWO_PI                      6.28318530718f
#define COMP_VEL_LPF_HZ             5.0f

/* Squat FSM debounce */
#define SQUAT_DEBOUNCE_MS           150U

/* ============================================================================
 * v2 변경 사항 — 사용자 맞춤 룩업 + Dwell-time + 500ms Ramp
 * ============================================================================
 *
 * [개요] MaxTorque0.csv (사용자 무토크 baseline) 분석 기반 개인화 보조 토크.
 *
 *   1. ASCENDING/BOTTOM/DESCENDING 토크를 EMG-비례 대신 각도-룩업으로 산출.
 *      - 룩업 값 = 사용자 EMG profile × scale - compensation(이미 인가됨)
 *      - 0-torque 실험은 compensation 없이 측정 → 룩업 테이블이 compensation
 *        몫을 미리 차감해서 들고 있음. 합쳤을 때 사용자 부담에 맞춰짐.
 *      - 사용자 sticking point(~15-25°)에서 자동 max 토크 → 가장 필요한 곳 집중.
 *
 *   2. BOTTOM phase 진입을 각도 임계 대신 dwell-time으로 인식.
 *      - 사용자가 의도적으로 깊이 도달 후 정지 → 그제서야 BOTTOM 인정.
 *      - 시스템 각도 ≥ 45°지만 사용자는 더 내려가려는 경우 → DESCENDING 유지.
 *      - 모터가 사용자 의도 방해 X.
 *
 *   3. BOTTOM/ASCENDING 진입 시 500ms ramp으로 토크 부드럽게 인가.
 *      - 인간 응답 지연(~500ms, Reinkensmeyer 2009) 매치.
 *      - 갑작스러운 토크 단차 → 사용자 동시수축 유발 방지. */

/* User-specific ASCEND lookup table — additional torque to add ON TOP of compensation.
 * Derived from MaxTorque0.csv (single subject, no-torque baseline):
 *   peak EMG ~1.46V @ 15-20° (sticking point)
 *   scale ≈ 3.42 Nm/V (peak torque 5 Nm)
 *   lookup(θ) = max(0, EMG(θ)×scale - compensation(θ))
 *   여기서 compensation = 0.5 × 8 × sin(θ) (현재 gravity_mgl_nm=8, scale=0.5 기준).
 * 깊은 각도(55°+)는 compensation 만으로 충분 → 룩업 0.
 * 다항식 피팅 대신 5° 간격 14점 룩업 + 선형 보간으로 매끄럽게.
 * 피험자 변경 시 baseline 재측정 후 이 표 갱신. */
#define USER_LOOKUP_ASCEND_POINTS   14
#define USER_LOOKUP_BASE_DEG        5.0f
#define USER_LOOKUP_STEP_DEG        5.0f

/* Dwell-time BOTTOM 인식 파라미터 — Live Expressions 튜닝 가능 (PUBLIC 변수 참조).
 * 사용자가 의도적으로 정지한 것으로 판정하는 임계.
 *   dwell_angle_dead_deg : 이 이내 변동은 정지로 간주 (엔코더 분해능 0.022°의 ~70배).
 *   dwell_threshold_ms   : 이 시간 동안 정지 유지 → BOTTOM 확정.
 *   dwell_exit_angle_deg : BOTTOM 탈출 임계 (히스테리시스, 진입보다 큰 값). */
#define DWELL_ANGLE_DEAD_DEG_DEFAULT  1.5f
#define DWELL_THRESHOLD_MS_DEFAULT    200U
#define DWELL_EXIT_ANGLE_DEG_DEFAULT  2.5f

/* 보조 토크 ramp-in (BOTTOM/ASCENDING 진입 시).
 * 기본 500ms = 인간 평균 응답 지연 (Reinkensmeyer 2009 Fig. 2).
 * 토크가 0 → target 으로 선형 증가. 사용자가 적응할 시간 확보.
 * [튜닝] 실제 ASCEND 가 ~1초로 빠르면 500ms 가 과해서 sticking point 에서 부분 출력만 나옴.
 *        Live Expressions 의 assist_ramp_time_ms 를 조정 (권장 범위 100~500ms).
 *        - 100~200ms: 빠른 동작 매칭, jolt 위험 ↑
 *        - 250~350ms: 균형점 (권장 시작값)
 *        - 500ms    : 보수적, 안전 우선 */
#define ASSIST_RAMP_TIME_MS_DEFAULT 500U

/* ============================================================================
 * ENUMERATIONS AND TYPES
 * ============================================================================ */

typedef enum { EMG_RH = 0, EMG_LH } EmgIdx_t;   /* RH=PF3(ADC_5), LH=PF4(ADC_6) */

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

/* [v2] 사용자 의도 추정 상태.
 * BOTTOM phase 내부에서 사용자가 정말 정지했는지(HOLD) 아직 더 내려가려는지
 * (DESCENDING_INTENT) 구분하기 위한 보조 상태.
 * Dwell-time 으로 판정. */
typedef enum {
    USER_INTENT_DESCEND = 0,  /* 내려가는 중(또는 더 내려가려는 의도) */
    USER_INTENT_HOLD,         /* 의도적 정지 (dwell 확정) */
    USER_INTENT_ASCEND        /* 일어나는 중 */
} UserIntent_t;

/* Button events captured once per loop to prevent double-consume. */
typedef struct {
    XmBtnEvent_t btn1;
    XmBtnEvent_t btn2;
    XmBtnEvent_t btn3;
} BtnEvents_t;

/* ============================================================================
 * CDC STREAM CONFIGURATION  (8 channels, Module ID 0xF0)
 * FSR 제거 후 채널 수 14→8.
 * Add/remove/reorder CDC_STREAM_NEXT rows only.  Keep metadata <= 512 bytes.
 * ============================================================================ */
/* 채널 순서 — 좌/우 페어 + phase + intent + control 9채널:
 *   ch1 = EMG LH raw (V)     ch2 = EMG RH raw (V)
 *   ch3 = L Encoder (deg)    ch4 = R Encoder (deg)
 *   ch5 = L Torque (Nm)      ch6 = R Torque (Nm)
 *   ch7 = Squat Phase (0-4)  ch8 = User Intent (0-2)  ch9 = Control ON (bool) */
#define CDC_STREAM_CHANNELS(F, N)                                               \
    F(emg_lh_raw,    "EMG LH Raw", "V",    emg_lh_raw_v)                       \
    N(emg_rh_raw,    "EMG RH Raw", "V",    emg_rh_raw_v)                       \
    N(enc_lh,        "L Encoder",  "deg",  left_control_angle_deg)             \
    N(enc_rh,        "R Encoder",  "deg",  right_control_angle_deg)            \
    N(l_torque,      "L Torque",   "Nm",   left_assist_torque_nm)              \
    N(r_torque,      "R Torque",   "Nm",   right_assist_torque_nm)             \
    N(sq_phase,      "Squat Ph",   "-",    squat_phase)                        \
    N(usr_intent,    "Intent",     "-",    user_intent)                        \
    N(ctrl_on,       "Control",    "bool", squat_control_ON)

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
 * s_emg_bias_v[0] 등 배열 원소를 직접 입력해서 볼 수 있음.
 * ============================================================================ */

/* --- EMG --- PF3=ADC_5(RH), PF4=ADC_6(LH) --- */
/* [중요] EMG_RH=index 0=PF3(DIO_1=ADC_5), EMG_LH=index 1=PF4(DIO_2=ADC_6).
 * 에러: 좌우 EMG가 뒤바뀌면 → s_emg_pins 배열 순서 확인. */
static const XmAdcPin_t s_emg_pins[EMG_COUNT] = {
    XM_EXT_ADC_5, XM_EXT_ADC_6   /* [0]=EMG_RH, [1]=EMG_LH */
};

/* [v2] 사용자 ASCEND 룩업 테이블 — 룩업 값 = max(0, EMG_baseline × scale - compensation).
 * 각도 5°~70° 5° 간격, 14점. 선형 보간으로 매끄럽게.
 * 데이터 출처: MaxTorque0.csv (피험자 무토크 baseline)
 * 가정: gravity_mgl_nm=8, compensation_scale=0.5 (이 값 변경 시 룩업도 재계산 필요)
 * 단위: Nm (compensation 외 추가로 인가되는 토크). */
static const float s_user_lookup_ascend[USER_LOOKUP_ASCEND_POINTS] = {
    /*  5°  */ 3.94f,
    /* 10°  */ 3.80f,
    /* 15°  */ 3.96f,  /* sticking point 최대 보조 */
    /* 20°  */ 3.51f,
    /* 25°  */ 2.74f,
    /* 30°  */ 2.16f,
    /* 35°  */ 2.07f,
    /* 40°  */ 1.20f,
    /* 45°  */ 0.42f,
    /* 50°  */ 0.21f,
    /* 55°  */ 0.00f,  /* 깊은 자세는 compensation 만으로 충분 */
    /* 60°  */ 0.00f,
    /* 65°  */ 0.00f,
    /* 70°  */ 0.00f
};
static float        s_emg_raw_v[EMG_COUNT];       /* ADC 직결 전압 (~1.65V 기준) */
static float        s_emg_centered_v[EMG_COUNT];  /* 바이어스 제거 후 (≈0V 기준) */
static float        s_emg_pre_lpf_v[EMG_COUNT];   /* 80Hz LPF 후 */
static float        s_emg_rect_v[EMG_COUNT];      /* 전파정류 후 (항상 양수) */
static float        s_emg_envelope_v[EMG_COUNT];  /* 5Hz 포락선 LPF 후 */
static float        s_emg_norm_priv[EMG_COUNT];   /* 데드밴드+정규화 결과 0~1 */
static float        s_emg_bias_v[EMG_COUNT]       = {EMG_OFFLINE_CAL_BIAS_V,
                                                      EMG_OFFLINE_CAL_BIAS_V};
static float        s_emg_full_scale_v[EMG_COUNT] = {EMG_OFFLINE_CAL_FULL_SCALE_V,
                                                      EMG_OFFLINE_CAL_FULL_SCALE_V};

static EmgCalState_t s_emg_cal_state;
static uint32_t     s_emg_cal_tick;
static uint32_t     s_emg_cal_count;
static double       s_emg_cal_sum[EMG_COUNT];
/*ADC 회로의 op-amp가 신호를 1.65V(전원 3.3V의 절반) 
근처로 끌어올려 음전압을 양전압으로 변환하지만, 
실제 바이어스는 1.55~1.70V 사이에서 부품 편차/온도/접지 임피던스에 따라 변동 
3초×1000Hz = 3000샘플 누적, double로 정밀도 확보
세션 시작 시점에 두 단계 캘 시행
Rest 캘 (3초): 가만히 있을 때 → 바이어스(DC offset) 측정
Effort 캘: 최대 힘 줄 때 → 신호 정규화 분모(0~1 스케일링 기준) 측정

캘 시작:        s_emg_cal_sum[i] = 0.0 
매 샘플(1kHz):  s_emg_cal_sum[i] += s_emg_raw_v[i]
3초 경과 후:    s_emg_bias_v[i] = s_emg_cal_sum[i] / s_emg_cal_count

왜 double인가 — 기술적 핵심
float (32bit IEEE 754)는 가수부가 23bit (≈7자리 십진수 정밀도).

수치 시나리오:
한 샘플: 1.592V (개별 진폭 변화 ~10µV = 0.00001V 단위까지 의미 있음)
3000샘플 합: 약 4776V
float에서 4776 근처 값의 분해능: 4776 / 2^23 ≈ 0.00057
*/
static float        s_emg_cal_max_env[EMG_COUNT];
/* Effort 캘: 최대 envelope 추적
채널별 독립 — 최대값만 유지
누적합이 아니라 running maximum

캘 시작:        s_emg_cal_max_env[i] = 0.0f
매 샘플(1kHz):  if (s_emg_envelope_v[i] > s_emg_cal_max_env[i])
                    s_emg_cal_max_env[i] = s_emg_envelope_v[i];
캘 종료:        s_emg_full_scale_v[i] = s_emg_cal_max_env[i]

MVC (Maximum Voluntary Contraction) 개념
근전도 표준 정규화 기법: 사용자가 자기 최대 힘을 낼 때의 신호 = 100%로 정의하고,
이후 모든 실시간 신호를 이 값으로 나눠 0~1로 표현. 
사람마다 절대값이 달라도 **상대적 노력도 (%MVC)**는 비교 가능해짐.

double이 아닌 float로 충분한 이유
max 연산은 누적 오차가 없습니다. 비교는 lossless 연산이고, 대입은 그 한 샘플의 정밀도만 유지하면 됨.
 1kHz × 수 초 동안 max를 추적해도 결과는 그 중 한 샘플 값일 뿐이라 
 float (7자리 정밀도, mV 분해능)로 충분함.
 
 왜 raw가 아니라 envelope에서 추적?
Effort 캘에서 비교 대상은 s_emg_envelope_v[i] (5Hz LPF 통과한 부드러운 포락선)입니다.
 raw는 ±수백 mV로 빠르게 진동하므로 단순 peak는 잡음(spike) 영향을 크게 받아 비현실적으로 
 큰 값으로 튐. envelope에서 max를 잡으면 잡음 스파이크를 무시할 수 있고,
 의미 있는 지속적 힘의 최대치를 잡고, 정규화 분모로 안정적일 수 있음
 
 캘이 끝나고 체인에서 역할
 // raw → centered (bias 제거)  ← s_emg_bias_v 사용
centered = raw - s_emg_bias_v[i];

// → pre-LPF (80Hz) → rectify → envelope (5Hz LPF)
// envelope → 정규화 0~1  ← s_emg_full_scale_v 사용
norm = clamp(envelope / s_emg_full_scale_v[i], 0.0f, 1.0f);

즉 s_emg_cal_sum과 s_emg_cal_max_env는 캘 중에만 살아있는 임시 버퍼이고, 
캘이 끝나면 그 결과물(s_emg_bias_v, s_emg_full_scale_v)이 실시간 신호 처리의 
두 개의 핵심 상수가 됩니다. 이 두 상수가 EMG 
→ 어시스트 토크로 가는 전체 파이프라인의 정확도 기반을 잡아주는 것임.
*/

/* Encoder() / compensation */
/*  s_comp_prev_valid: false=첫 루프
 속도 미분 시 첫 루프에서는 prev=0 -> 스파이크 방지용 플래그. 
 if (s_comp_prev_valid) 블록 참고 

 * ASSIST 모드 재진입 시 false로 리셋되므로 모드 전환 시 속도 스파이크 없음.
 
 사용자가 ASSIST → IDLE → ASSIST 로 모드를 옮기면, 
 그 사이 외골격이 다른 자세로 옮겨졌을 수 있음. 재진입 순간 prev는 수 초~수 분 전의 각도를 가리킬 수 있고, 그 차이를 dt(1ms)로 나누면 또 거대한 거짓 ω 발생. → ASSIST 진입 hook에서 s_comp_prev_valid = false로 초기화 → 첫 루프에서 다시 안전하게 시작.

이 패턴이 "derivative initialization guard". 
모든 수치 미분(D 제어기, 칼만 필터 prediction step 등)에서 표준적으로 쓰임.*/

static bool         s_comp_prev_valid;
static float        s_comp_prev_left_rad;
static float        s_comp_prev_right_rad;

/* --- Squat FSM --- */
static SquatPhase_t s_squat_phase;
static uint32_t     s_squat_debounce_tick;
static bool         s_squat_debounce_active;

/* --- [v2] Dwell-time BOTTOM 인식 --- */
static UserIntent_t s_user_intent;             /* 현재 의도 추정 */
static uint32_t     s_dwell_window_start_tick; /* dwell 윈도우 시작 tick */
static float        s_dwell_window_start_angle;/* dwell 윈도우 시작 각도 */

/* --- [v2] 보조 토크 ramp-in (500ms) --- */
static uint32_t     s_assist_ramp_tick;   /* ramp 시작 tick */
static bool         s_assist_ramp_active; /* ramp 진행 중 여부 */
static bool         s_prev_base_active;   /* 직전 루프 base>0 여부 (edge detect) */

/* --- [v2-fix] 출력 토크 slew-rate limiter (phase 전이 step 완화) ---
 * 직전 루프 출력값을 기억하고, 다음 루프 출력이 그로부터 너무 멀어지지 않게 제한.
 * 목적: ASC→RETURN 전이 시 base 가 즉시 0이 되며 발생하는 8 Nm step 차단.
 * 이 step 이 모터 di/dt 스파이크 → 과전류 보호 트립 → "lost target" 끊김 유발. */
static float        s_torque_lh_out;
static float        s_torque_rh_out;

/* --- System --- */
/* s_assist_session_active: ASSIST 모드 첫 진입 감지용.
 * 이 플래그가 없으면 ASSIST 재진입마다 캘이 리셋되지 않음. */
static bool              s_assist_session_active;
/* s_torque_mode_active: XM_CTRL_TORQUE 모드 중복 설정 방지.
 * 에러: 1ms마다 XM_SetControlMode를 반복 호출하면 통신 부하 발생 가능. */
static bool              s_torque_mode_active;
static uint32_t          s_stream_tick;
static SquatStreamData_t s_stream;

/* ==============================================
 * PUBLIC VARIABLES  (STM32CubeIDE Live Expressions)
 * ==================================================== */

/* Controls */
uint16_t squat_control_ON       = 0U;
uint16_t compensation_ON        = 1U;
uint16_t friction_compensation_ON = 0U;
uint16_t cdc_stream_enable      = 1U;
uint16_t cdc_stream_period_ms   = 10U;

/* EMG calibration trigger (Live Expressions write-to-1 to start)
 * 버튼 캘 대체용. 디버거에서 1 쓰면 캘 시작, 시작 후 자동으로 0 으로 클리어.
 * 향후 버튼 부활 시 _ReadButtons() 에서 이 변수를 1 로 세팅하면 그대로 호환. */
uint16_t emg_cal_rest_request   = 0U;
uint16_t emg_cal_effort_request = 0U;
uint16_t emg_cal_reset_request  = 0U;

/* Status flags (observe only) */
uint16_t assist_enable          = 0U;
uint16_t assist_mode_active     = 0U;
uint16_t emg_cal_rest_done      = 0U;
uint16_t emg_cal_effort_done    = 0U;
uint16_t emg_cal_done           = 0U;
uint16_t squat_phase            = 0U;
/* [v2] dwell-based user intent (0=DESCEND, 1=HOLD, 2=ASCEND) — observe only */
uint16_t user_intent            = 0U;

/* EMG raw voltages — PF3=ADC_5(RH), PF4=ADC_6(LH) */
float pf3_volt;   /* EMG RH raw  */
float pf4_volt;   /* EMG LH raw  */

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
/* [중요] offset 설정 방법: 기립 자세에서 Live Expressions로 left_encoder_angle_deg
 * 값을 읽어 encoder_offset_lh_deg 에 그 값을 입력 → control_angle 이 0 이 됨.
 * leftHipMotorAngle 은 전원 ON 이후 누적 각도이므로 초기값이 임의의 큰 수일 수 있음. */
/* [중요] 기립 자세에서 raw 엔코더가 +30° 로 측정됨 → offset=30° 로 설정하면 control_angle=0°.
 *   실측값이 다르면 Live Expressions 에서 left_encoder_angle_deg 값을 읽어 직접 수정. */
float encoder_offset_lh_deg        = 30.0f;
float encoder_offset_rh_deg        = 30.0f;
float encoder_sign_lh              = 1.0f;
float encoder_sign_rh              = 1.0f;
/* gravity_mgl_nm: 외골격 본체 팔(링크+액추에이터) 자체의 무게 보상 계수 (~8 Nm).
 *   M(외골격 팔 ~1.8kg) × g(9.81) × L(COM 거리 ~0.35m) ≈ 6~8 Nm.
 *   ※ 사용자 몸 무게 보상이 아님 — 사람 70kg 부담은 한쪽 ~60 Nm 인데 과제 10 Nm
 *      hard cap 으로 불가. 본 시스템은 "외골격 팔 무게는 모터가 받고, 사용자 몸
 *      무게는 사용자 본인 근력 + 룩업 light-boost 로 처리" 정책.
 * compensation_scale: gravity 토크 중 실제 출력 비율 (0~1). */
float gravity_mgl_nm               = 8.0f;
float compensation_scale           = 0.50f;
float coulomb_friction_nm          = 0.10f;
float viscous_friction_nms         = 0.01f;
float velocity_deadzone_rads       = 0.10f;
float squat_max_torque_nm          = 6.0f;   /* ASCENDING=고정 적용, DESCENDING=EMG 비례 스케일. 최대 10 Nm */
float assist_torque_limit_nm       = 10.0f;  /* 최대 허용 (HARD_MAX_ASSIST_TORQUE_NM 과 동일) */
float torque_sign                  = 1.0f;   /* set -1.0 in Live Expressions if torque direction wrong */
float squat_enter_threshold_deg    = 15.0f;
float squat_bottom_threshold_deg   = 45.0f;
float squat_stand_threshold_deg    = 5.0f;
/* EMG 기반 BOTTOM→ASCENDING 확정 임계값 (0~1).
 * avg EMG norm 이 이 값 이상이면 velocity 조건 없이 ASCENDING 진입.
 * 0.0 으로 설정 시 EMG 확정 비활성화 (velocity 조건만 사용). */
float emg_ascend_threshold         = 0.5f;
/* [v2] Ramp-in 지속시간 — Live Expressions 튜닝 대상.
 * 100~500ms 범위 권장. 너무 짧으면 jolt, 너무 길면 sticking 통과 후 토크 도달. */
uint16_t assist_ramp_time_ms       = ASSIST_RAMP_TIME_MS_DEFAULT;

/* [v2] Dwell-time BOTTOM 인식 파라미터 — Live Expressions 튜닝 가능.
 * dwell_angle_dead_deg : HOLD 진입 임계 (이 이내 변동은 정지로 간주)
 * dwell_threshold_ms   : dwell 윈도우 길이 (이 시간 동안 정지 유지하면 HOLD)
 * dwell_exit_angle_deg : HOLD 탈출 임계 (히스테리시스용, 진입보다 큰 값) */
float    dwell_angle_dead_deg      = DWELL_ANGLE_DEAD_DEG_DEFAULT;
uint16_t dwell_threshold_ms        = DWELL_THRESHOLD_MS_DEFAULT;
float    dwell_exit_angle_deg      = DWELL_EXIT_ANGLE_DEG_DEFAULT;

/* [v2-fix] 출력 토크 slew-rate (Nm 변화량 / 1ms 루프).
 * 0.02 = 20 Nm/s → 10 Nm step 을 500ms 에 걸쳐 완화.
 * 너무 작게 (예: 0.005) 하면 응답 느려서 보조 체감 떨어짐.
 * 너무 크게 (예: 0.1) 하면 step 다시 인가 → 모터 보호 트립 위험.
 * 권장 범위: 0.01 ~ 0.05 Nm/ms */
float    torque_slew_nm_per_ms     = 0.02f;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

static BtnEvents_t _ReadButtons(void);

/* EMG */
static void _SampleEmg(void);
static void _ResetEmgFilters(void);
static void _ProcessEmgSignals(void);
static void _UpdateEmgCal(void);
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

/* [v2] 사용자 룩업 / Dwell / Ramp */
static float _LookupUserAscend(float angle_deg);
static void  _UpdateUserIntent(float avg_angle);
static float _ComputeAssistRamp(bool base_active);

/* ============================================================================
 * PUBLIC ENTRY POINTS
 * ============================================================================ */

void User_Setup(void)
{
    /* EMG : PF3=DIO_1(RH)  PF4=DIO_2(LH) */

    /* ── 센서 전원 및 핀 설정 ─────────────────────────────────────────────────
     * [에러] EMG 전체 0V → XM_SetExtPowerVoltage 누락 또는 순서 오류.
     *        반드시 XM_SwitchDioToAdc 보다 먼저 호출해야 함.
     * [에러] 특정 채널만 0V → 해당 DIO 번호의 XM_SwitchDioToAdc 호출 누락.
     *        DIO↔핀 매핑: DIO_1=PF3(EMG RH), DIO_2=PF4(EMG LH)
     * [에러] 값이 3.3V 고정 → 5V 전원은 켜졌으나 센서 물리 연결 단선.
     * [에러] 값이 노이즈만 → 5V 전원 불안정 또는 GND 미연결. */
    XM_SetExtPowerVoltage(XM_EXT_PWR_5V);
    XM_SwitchDioToAdc(XM_EXT_DIO_1);
    XM_SwitchDioToAdc(XM_EXT_DIO_2);

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

    _ResetEmgCal();

    s_stream_tick = XM_GetTick();
    XM_SendUsbDebugMessage(
        "[SQUAT] ready | EMG: BTN1-CLICK=rest  BTN2-CLICK=effort  BTN3-CLICK=reset\r\n");
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
     * [중요] 이 함수들은 ASSIST 조건 분기 바깥에 있어야 함 (의도적 설계).
     *   이유 1: MONITOR 모드에서도 Live Expressions에서 센서 값이 보여야 캘 확인 가능.
     *   이유 2: EMG bias 누적(s_emg_cal_sum)이 _SampleEmg 내부에서 일어나므로
     *            반드시 매 루프 호출 필요.
     * [에러] 센서 값이 ASSIST 모드 진입 전까지 0으로 보임
     *   → _Sample 함수들이 is_assist_mode 체크 안쪽으로 이동된 것. */
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
        _ResetEmgCal();
        s_squat_phase          = SQUAT_STAND;
        s_squat_debounce_active = false;
        s_comp_prev_valid      = false;
        /* [v2] dwell/intent/ramp 상태 초기화 */
        s_user_intent              = USER_INTENT_DESCEND;
        s_dwell_window_start_tick  = XM_GetTick();
        s_dwell_window_start_angle = (left_control_angle_deg + right_control_angle_deg) * 0.5f;
        s_assist_ramp_active       = false;
        s_prev_base_active         = false;
        s_assist_session_active = true;
        XM_SendUsbDebugMessage("[SQUAT] session started\r\n");
    }

    /* ── 버튼 이벤트 캡처 ────────────────────────────────────────────────────
     * [중요] BTN 이벤트는 루프 내에서 단 한 번만 읽어야 함.
     *   XM_GetButtonEvent()를 _UpdateEmgCal에서 직접 호출하면
     *   하나의 CLICK이 두 함수에서 동시에 소비되어 의도치 않은 동작 발생.
     * [에러] BTN1 CLICK이 FSR 캘과 EMG 캘에 동시에 반응함 → 각 함수에서 직접
     *        XM_GetButtonEvent를 호출하고 있는 것. ev 구조체를 통해서만 전달해야 함. */
    ev = _ReadButtons();

    /* ── 버튼 → EMG cal request 변환 ────────────────────────────────────────
     * _ReadButtons() 가 매 루프 XM_GetButtonEvent() 로 이벤트를 소비하므로,
     * ev 를 어디서도 사용하지 않으면 버튼 입력이 그대로 버려진다.
     * Live Expressions 의 emg_cal_*_request 변수를 1 로 세팅하면 _UpdateEmgCal()
     * 이 동일한 캘 시퀀스를 시작하므로, 버튼 입력을 그 변수로 매핑해 둔다.
     * → 버튼·디버거 둘 다로 캘 트리거 가능. */
    if (ev.btn1 == XM_BTN_CLICK) { emg_cal_rest_request   = 1U; }
    if (ev.btn2 == XM_BTN_CLICK) { emg_cal_effort_request = 1U; }
    if (ev.btn3 == XM_BTN_CLICK) { emg_cal_reset_request  = 1U; }

    /* ── ASSIST 모드 제어 파이프라인 (실행 순서 절대 변경 금지) ──────────────
     *
     * 순서 의존성 요약:
     *   _SampleEmg()     → s_emg_raw_v 갱신 + rest 캘 누적 (위에서 이미 실행)
     *   _SampleEncoder() → 각도/속도 갱신 (위에서 이미 실행)
     *   _ProcessEmgSignals() → raw_v 이용해 envelope 계산 (SampleEmg 이후여야 함)
     *   _UpdateEmgCal()  → envelope 이용해 full_scale 산출 (ProcessEmg 이후여야 함)
     *   _UpdateCompensation() → 각도/속도 이용 (SampleEncoder 이후여야 함)
     *   _UpdateSquatFsm()  → 각도 이용 (SampleEncoder 이후여야 함)
     *   _ApplyTorque()   → norm, phase, compensation 모두 사용 (위 모두 완료 후)
     *   _UpdatePublicSignals() → 최종 내부값을 public으로 복사 (ApplyTorque 이후)
     *   _SendStream()    → public 변수로 CDC 전송 (UpdatePublicSignals 이후)
     *
     * [에러] 파이프라인 순서를 바꾸면 1루프 lag 또는 stale 데이터로 동작 이상 발생. */

    /* (1) EMG 신호 처리
     * [중요] REST 캘 진행 중에는 필터를 리셋(0으로) 하고 ProcessEmgSignals 호출 안 함.
     *   이유: 바이어스 측정 기준인 raw 전압이 필터로 왜곡되면 안 되기 때문.
     *   s_emg_cal_sum 에는 raw_v가 누적되고 있음 (_SampleEmg 에서).
     * [에러] Rest 캘 중 envelope 값이 올라감 → _ResetEmgFilters 호출이 빠진 것. */
    if (s_emg_cal_state == EMG_CAL_REST_RUNNING) {
        _ResetEmgFilters();
    } else {
        _ProcessEmgSignals();
    }

    /* (2) EMG 캘 상태머신 — Live Expressions 요청 플래그 + 3초 타이머 완료 판단 */
    _UpdateEmgCal();

    /* (3) 중력/마찰 보상 토크 계산 */
    _UpdateCompensation();

    /* (4) 스쿼트 FSM 단계 판정 */
    _UpdateSquatFsm();

    /* (5) 안전 게이트 + 토크 출력 */
    _ApplyTorque();

    /* (6) public 변수 동기화 (Live Expressions에 반영) */
    _UpdatePublicSignals();

    /* (7) CDC USB 스트림 전송 */
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
        /* 바이어스: 캘 전 EMG_OFFLINE_CAL_BIAS_V(1.592V), 캘 완료 후 실측 평균값 */
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

static void _UpdateEmgCal(void)
{
    bool busy;
    bool elapsed;
    int i;

    busy = (s_emg_cal_state == EMG_CAL_REST_RUNNING ||
            s_emg_cal_state == EMG_CAL_EFFORT_RUNNING);

    /* ── EMG 캘리브레이션 요청 처리 ──────────────────────────────────────────
     *
     * 트리거: Live Expressions 에서 요청 변수에 1 을 씀.
     *   emg_cal_reset_request  = 1 → 디폴트(offline cal 값) 복귀
     *   emg_cal_rest_request   = 1 → Rest 캘 시작 (3초 정지)
     *   emg_cal_effort_request = 1 → Effort 캘 시작 (3초 내 최대 수축)
     * 시작 처리 후 자동으로 0 으로 클리어됨 (edge-trigger).
     *
     * 필수 순서: Rest 캘 → Effort 캘.
     *
     * [현재 상태] 버튼 캘은 하드웨어 미연결로 사용 안 함.
     *   대신 #define EMG_OFFLINE_CAL_BIAS_V / EMG_OFFLINE_CAL_FULL_SCALE_V 에
     *   피험자 사전 측정값을 박아 offline cal 로 동작 중.
     *   런타임 재캘이 필요할 때만 위 요청 변수를 디버거에서 1 로 설정.
     *
     * [버튼 부활 시] _ReadButtons() 에서 BTN1 CLICK → emg_cal_rest_request=1,
     *   BTN2 CLICK → emg_cal_effort_request=1, BTN3 CLICK → emg_cal_reset_request=1
     *   로 매핑하면 이 함수 수정 없이 그대로 동작.
     *
     * [에러] emg_cal_done 이 0으로 남음:
     *   케이스 1: Effort 캘은 했는데 Rest 캘을 안 함 → Rest 먼저.
     *   케이스 2: Rest 캘 완료 후 ASSIST 모드를 이탈했다가 재진입 →
     *             _ResetEmgCal() 호출로 emg_cal_rest_done 이 0 으로 리셋.
     *             두 캘을 같은 ASSIST 세션 안에서 연속으로 수행해야 함.
     *
     * [에러] Rest 캘 중 몸이 움직여 바이어스가 이상하게 잡힘:
     *   → emg_cal_rest_request=1 직후 3초 동안 완전히 정지 상태 유지.
     *
     * [에러] Effort 캘 중 최대값이 너무 낮음 (norm이 여전히 작음):
     *   → 3초 내에 최대 수축을 했는지 확인.
     *   → 요청 후 0.5~1초 지연 후 수축 시작 권장 (필터 과도 구간 대기).
     *
     * [주의] busy 체크: 3초 진행 중에는 다른 요청 모두 무시됨. */

    /* Reset request -> 디폴트(offline cal) 값으로 복귀 */
    if (emg_cal_reset_request == 1U && !busy) {
        _ResetEmgCal();
        emg_cal_reset_request = 0U;
        XM_SendUsbDebugMessage("[EMG CAL] reset to defaults\r\n");
        return;
    }
    /* Rest request -> rest calibration */
    if (emg_cal_rest_request == 1U && !busy) {
        _StartEmgRestCal();
        emg_cal_rest_request = 0U;
        return;
    }
    /* Effort request -> effort calibration */
    if (emg_cal_effort_request == 1U && !busy) {
        _StartEmgEffortCal();
        emg_cal_effort_request = 0U;
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
        s_emg_bias_v[i]       = EMG_OFFLINE_CAL_BIAS_V;
        s_emg_full_scale_v[i] = EMG_OFFLINE_CAL_FULL_SCALE_V;
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

    /* [중요] leftHipMotorAngle 은 전원 ON 이후 누적 각도(degree).
     *   → 처음 기립 자세에서 left_encoder_angle_deg 값을 읽어
     *     encoder_offset_lh_deg 에 그 값을 입력하면 control_angle 이 0이 됨.
     * [에러] 각도가 터무니없이 크게 시작하는 이유:
     *   → 이 offset 을 설정하지 않아서 누적값 그대로 노출되는 것.
     *      encoder_offset_lh_deg = (현재 left_encoder_angle_deg 값) 으로 설정. */
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

    /* [안전] 음수 중력 보상 차단.
     * 스쿼트 보조는 굽힘(+) 방향만 필요. 엔코더 영점이 어긋나 θ<0 으로 읽힐 때
     * 모터가 사용자를 뒤로 끌어당기는 토크가 인가되는 사고를 방지.
     * 영점 캘이 정확하면 직립에서 sinf≈0 이므로 클램프가 거의 작동 안 함. */
    if (left_gravity_torque_nm  < 0.0f) left_gravity_torque_nm  = 0.0f;
    if (right_gravity_torque_nm < 0.0f) right_gravity_torque_nm = 0.0f;

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
 * SQUAT FSM  [v2]
 * ============================================================================
 *
 * Phase transitions (based on avg hip angle + dwell-based user intent):
 *
 *   STAND      -> DESCENDING : avg_angle >= squat_enter_threshold   (150 ms debounce)
 *   DESCENDING -> BOTTOM     : intent == HOLD  &&  avg_angle >= squat_enter
 *                              (dwell-time 기반, 각도 임계값 미사용 — 사용자가
 *                               의도적으로 정지할 때만 BOTTOM 인정)
 *   DESCENDING -> RETURN     : avg_angle < squat_enter_threshold    (aborted rep)
 *   BOTTOM     -> ASCENDING  : intent == ASCEND  ||  avg_emg >= emg_ascend_threshold
 *                              (각속도 미분 사용 안 함, dwell 결과 + EMG push)
 *   ASCENDING  -> RETURN     : avg_angle < squat_enter_threshold
 *   RETURN     -> STAND      : avg_angle <= squat_stand_threshold    (150 ms debounce)
 *   (RETURN → ASCENDING 제거: 15° 근처 채터링 방지)
 *
 * Base 토크 (룩업 기반):
 *   ASCENDING        → lookup(angle) × ramp 적용
 *   BOTTOM(HOLD)     → lookup(angle) × ramp 적용 (정지 보조)
 *   BOTTOM(DESCEND)  → 0 (아직 내려가려는 의도 — 방해 안 함)
 *   그 외 (STAND/DESC/RETURN) → 0
 *
 * Compensation 토크는 phase 무관하게 active 상태에서 항상 인가됨 (각도 sin 함수).
 * 사용자 의도(USER_INTENT_DESCEND/HOLD/ASCEND)는 _UpdateUserIntent() 가 sliding
 * window dwell-time 으로 판정. 헬퍼 섹션 참조.
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
     * (v2): avg_velocity 는 계산만 하고 FSM 전이엔 사용 안 함. dwell intent 사용.
     *
     * [에러] avg_angle이 스쿼트해도 0 근처에서 변하지 않음:
     *   케이스 1: 두 엔코더 모두 freeze → CAN 문제.
     *   케이스 2: encoder_sign이 반대라 좌우가 서로 상쇄 → 한쪽만 확인.
     *   케이스 3: encoder_offset이 잘못 설정되어 부호 반전된 것.
     *   진단 방법: left_control_angle_deg 와 right_control_angle_deg 를
     *              각각 Live Expressions에서 스쿼트하며 변화 확인.
     *
     * [FSM 전환 조건 요약 — 문제 생기면 임계값/dwell 파라미터 조정]:
     *   STAND→DESC  : avg_angle >= squat_enter_threshold (기본 15°) + 150ms 지속
     *   DESC→BOTTOM : intent==HOLD && avg_angle>=enter_threshold  (각도 임계 없음)
     *   DESC→RETURN : avg_angle < squat_enter (조기 복귀)
     *   BOTTOM→ASC  : intent==ASCEND  ||  avg_emg >= emg_ascend_threshold
     *   ASC→RETURN  : avg_angle < squat_enter (거의 일어선 것)
     *   RETURN→STAND: avg_angle <= squat_stand_threshold (기본 5°) + 150ms
     *   (RETURN→ASC 제거: 15° 근처 채터링 방지)
     *
     * [에러] STAND→DESC 전환이 안 됨:
     *   → squat_enter_threshold(15°)가 너무 큼. 실제 시작 각도를 확인하고 낮출 것.
     *
     * [에러] DESC→BOTTOM 전환이 안 됨 (사용자 정지해도 DESCENDING 유지):
     *   → intent 가 HOLD 로 확정 안 됨. _UpdateUserIntent 의 윈도우 판정 확인.
     *     (a) dwell_threshold_ms (기본 200) 가 너무 길어 윈도우 만료 전 다시 움직임.
     *     (b) dwell_angle_dead_deg (기본 1.5°) 가 너무 좁아 미세 흔들림으로 탈락.
     *     (c) 사용자가 실제로 계속 천천히 내려가고 있음 — 의도대로면 정상.
     *
     * [에러] BOTTOM→ASC 전환이 안 됨 (상승 시작해도 BOTTOM 유지):
     *   → intent 가 ASCEND 로 안 바뀜. 두 가지 확인:
     *     (a) dwell_exit_angle_deg (기본 2.5°) 가 너무 큼 → 작은 상승을 못 잡음. 낮출 것.
     *     (b) emg_ascend_threshold 를 활용해 EMG 강제 트리거 (기본 0.5, 0=비활성).
     *
     * [에러] BOTTOM→ASC 전환이 너무 빨리 일어남 (BOTTOM 머무르려 했는데 ASC로 튐):
     *   → emg_ascend_threshold 가 너무 낮음. 0.7~1.0 으로 올리거나 0 으로 비활성화.
     *
     * [에러] ASC→RETURN 전환이 안 됨 (torque가 끝나지 않음):
     *   → 일어선 후에도 avg_angle이 enter_threshold(15°) 아래로 안 내려옴.
     *   → encoder_offset이 과도하게 설정되거나 기립 자세가 완전하지 않은 것.
     *   → squat_enter_threshold 를 더 크게 설정해서 조건을 쉽게 빠져나오게 할 것. */
    avg_angle    = (left_control_angle_deg      + right_control_angle_deg)      * 0.5f;
    avg_velocity = (left_angular_velocity_rads  + right_angular_velocity_rads)  * 0.5f;
    now          = XM_GetTick();
    (void)avg_velocity;   /* [v2] BOTTOM→ASC 전환에 더 이상 사용 안 함 (intent 기반) */

    /* [v2] 사용자 의도(Dwell-time) 갱신 — 이 결과를 FSM 전이에 사용 */
    _UpdateUserIntent(avg_angle);

    enter_zone  = (avg_angle >= squat_enter_threshold_deg);
    bottom_zone = (avg_angle >= squat_bottom_threshold_deg);
    stand_zone  = (avg_angle <= squat_stand_threshold_deg);
    (void)bottom_zone;    /* [v2] BOTTOM 진입은 intent 기반 (각도 임계 미사용) */

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
        /* [v2] BOTTOM 진입을 각도 임계 대신 Dwell-time 기반으로 변경.
         *   사용자가 의도적으로 정지(dwell)할 때만 BOTTOM 인정.
         *   각도가 45° 넘어도 사용자가 더 내려가려는 의도면 DESCENDING 유지.
         *   → 모터가 사용자 의도 방해 X.
         *
         * 안전 조건: 의도가 HOLD 로 확정되었고 어느 정도 깊이(enter_zone) 도달했을 때만. */
        if ((s_user_intent == USER_INTENT_HOLD) && enter_zone) {
            s_squat_phase           = SQUAT_BOTTOM;
            s_squat_debounce_active = false;
        } else if (!enter_zone) {
            /* Aborted before reaching bottom */
            s_squat_phase           = SQUAT_RETURN;
            s_squat_debounce_active = false;
        }
        break;

    case SQUAT_BOTTOM:
        /* [v2] BOTTOM → ASCENDING 전환을 Dwell intent 기반으로 변경.
         *   사용자 의도가 ASCEND 로 바뀌면(각도 감소 EXIT_ANGLE 초과) 전환.
         *   각속도 미분 안 씀 → 노이즈 영향 ↓.
         *   기존 emg_ascend_threshold 조건은 보조로 유지(매우 강한 EMG 시 즉시 전환). */
        {
            float avg_emg = (s_emg_norm_priv[EMG_LH] + s_emg_norm_priv[EMG_RH]) * 0.5f;
            bool  emg_push = (emg_ascend_threshold > 0.0f) &&
                             (avg_emg >= emg_ascend_threshold);
            if (s_user_intent == USER_INTENT_ASCEND || emg_push) {
                s_squat_phase = SQUAT_ASCENDING;
            }
        }
        break;

    case SQUAT_ASCENDING:
        if (!enter_zone) {
            s_squat_phase           = SQUAT_RETURN;
            s_squat_debounce_active = false;
        }
        break;

    case SQUAT_RETURN:
        /* [v2] RETURN → ASC 전이 제거.
         *   기존: 15° 근처에서 각도 흔들리면 RETURN↔ASC 채터링 (4→3→4→3) 발생.
         *   정책: RETURN 진입 후엔 무조건 STAND 만 갈 수 있음.
         *   사용자가 다 일어서기 전에 다시 squat 시작하면 한 번 완전히 일어선 뒤
         *   다시 앉으면 됨 (실험 상 거의 발생 안 하는 시나리오). */
        if (stand_zone) {
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
     * 2개 조건 AND:
     *   [1] squat_control_ON == 1  : 사용자가 Live Expressions에서 명시적으로 1 설정
     *   [2] H10 mode == ASSIST     : 하드웨어가 실제로 ASSIST 상태
     *
     * [에러] assist_enable = 0 → 토크 없음. 진단 순서:
     *   step1: assist_mode_active 확인 → 0이면 H10 모드 전환 안 된 것.
     *   step2: squat_control_ON 확인 → 0이면 Live Expressions에서 1로 설정.
     *
     * [에러] assist_enable = 1 인데 토크가 전혀 안 느껴짐:
     *   케이스 1: squat_phase 가 ASCENDING(3) 도 BOTTOM(2, HOLD intent) 도 아님
     *     → base 토크 0. compensation_ON=1 이면 보상 토크는 모든 active phase 에서 나옴.
     *   케이스 2: squat_max_torque_nm 이 0 으로 설정 → 룩업 비활성화. 6.0 이 기본.
     *   케이스 3: assist_ramp_time_ms 가 매우 큼 + ASCEND 가 빠름 → ramp 0~50% 만 도달.
     *   케이스 4: torque_sign 부호 반전으로 기계 저항으로 작용 (H10 임피던스 모드).
     *   케이스 5: assist_torque_limit_nm 가 너무 낮음 → clamp 후 출력 작음.
     *
     * [에러] 안전 게이트 통과했는데 XM_SetAssistTorqueLH 가 실제 모터 출력 안 됨:
     *   → XM_SetControlMode(XM_CTRL_TORQUE) 가 설정됐는지 확인.
     *     s_torque_mode_active=false 상태에서 첫 진입 시에만 설정. */
    assist_requested = (squat_control_ON == 1U) &&
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
        /* [v2] dwell/intent/ramp 상태 리셋 — 재진입 시 깨끗한 상태로 시작 */
        s_user_intent               = USER_INTENT_DESCEND;
        s_dwell_window_start_tick   = XM_GetTick();
        s_dwell_window_start_angle  = (left_control_angle_deg + right_control_angle_deg) * 0.5f;
        s_assist_ramp_active        = false;
        s_prev_base_active          = false;
        s_torque_lh_out             = 0.0f;
        s_torque_rh_out             = 0.0f;
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

    /* ── [v2] 보조 토크 계산 — 사용자 룩업 + 500ms Ramp ────────────────────
     *
     * 핵심 변경:
     *   - DESCENDING: base = 0 (EMG 비례 제거).
     *     이유: MaxTorque0 분석에서 DESC EMG 매우 낮음(평균 0.24V),
     *          compensation 만으로 사용자 부담 이미 충분히 받침.
     *          EMG 비례 토크는 노이즈/지연으로 사용자 동시수축 유발.
     *
     *   - BOTTOM(HOLD intent): base = lookup(angle) × ramp.
     *     Dwell-time 으로 사용자 의도가 HOLD 로 확정된 후 500ms ramp 으로 적용.
     *     사용자가 아직 내려가려는 의도(DESCEND)면 base = 0.
     *
     *   - ASCENDING: base = lookup(angle) × ramp.
     *     사용자 baseline EMG 프로파일 기반. sticking point(~15-25°)에서 자동 최대.
     *     compensation 몫은 룩업이 미리 차감해 둠.
     *
     * 룩업 값 (Nm) = max(0, EMG_baseline(θ) × scale - compensation(θ))
     *   compensation = 0.5 × 8 × sin(θ) 가정. gravity_mgl_nm/compensation_scale
     *   변경 시 룩업 테이블 재계산 필요.
     *
     * Ramp 동작: base 가 0→양수로 바뀌는 순간 assist_ramp_time_ms 동안 선형 증가.
     *   기본 500ms (Reinkensmeyer 2009 인간 응답 지연 매치).
     *   ASCEND 가 1초로 짧을 땐 200~300ms 로 낮춰서 sticking point 에 토크 도달 보장. */
    {
        float avg_angle_deg = (left_control_angle_deg + right_control_angle_deg) * 0.5f;
        float target_base;
        float ramp_mul;
        bool  base_active;

        if (s_squat_phase == SQUAT_ASCENDING) {
            target_base = _LookupUserAscend(avg_angle_deg);
        } else if (s_squat_phase == SQUAT_BOTTOM) {
            if (s_user_intent == USER_INTENT_HOLD) {
                target_base = _LookupUserAscend(avg_angle_deg);
            } else {
                /* 아직 내려가려는 의도 → 방해 안 함 */
                target_base = 0.0f;
            }
        } else {
            /* DESCENDING / STAND / RETURN: lookup base 0 (compensation 만) */
            target_base = 0.0f;
        }

        /* squat_max_torque_nm 을 전역 게인으로 활용 (Live Expr 튜닝).
         * 6.0 (=룩업 설계 기준값) → 룩업 값 그대로 출력.
         * 3.0 → 절반, 0.0 → lookup 비활성 (compensation 만). */
        target_base *= (squat_max_torque_nm / 6.0f);  /* 6.0 = 룩업 설계 기준값 */

        base_active = (target_base > 0.05f);
        ramp_mul    = _ComputeAssistRamp(base_active);

        base_lh = target_base * ramp_mul;
        base_rh = target_base * ramp_mul;
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
     * [에러] 토크가 limit(기본 10.0 Nm, hard cap 10.0)으로 고정:
     *   → 입력 합계가 limit을 초과. squat_max_torque_nm 또는 gravity_mgl_nm 낮출 것.
     *   → 기본값에선 발생 안 함 (peak ~5 Nm vs limit 10 Nm). 사용자가 limit 낮추면 가능.
     *
     * [에러] 토크 방향 반대 (저항으로 느껴짐):
     *   케이스 1: torque_sign = -1.0 으로 Live Expressions 수정.
     *   케이스 2: compensation 토크 방향 문제 → compensation_ON=0 해서 base 토크만 확인.
     *   케이스 3: ASCENDING phase 에서 compensation_ON=0 + squat_max_torque_nm 만 켠 상태로
     *             base/comp 어느 쪽이 문제인지 격리해서 확인.
     *
     * [중요] torque_sign은 left/right 동시에 적용됨.
     *   좌우가 각각 반대 방향이면 (비대칭 장착) torque_sign으로는 해결 안 됨.
     *   → encoder_sign_lh/rh 로 각 채널 개별 보정 필요.
     *
     * [에러] XM_SetAssistTorqueLH 호출 후 실제 모터에서 응답 없음:
     *   → s_torque_mode_active=true + XM_CTRL_TORQUE 상태 확인.
     *   → H10 통신 상태 확인 (CAN 연결). */
    /* ── 토크 출력 (slew-rate limit) ────────────────────────────────────────
     * Phase 전이 시 base 토크가 즉시 0이 되며 발생하는 step 차단.
     * 직전 출력값에서 ±torque_slew_nm_per_ms 이내로만 변화 허용.
     * → 모터 di/dt 스파이크 차단 → 과전류 보호 트립 방지.
     * Ramp-up 은 _ComputeAssistRamp() 가, ramp-down 은 이 slew 가 담당. */
    {
        float target_lh = torque_sign * left_assist_torque_nm;
        float target_rh = torque_sign * right_assist_torque_nm;
        float slew      = torque_slew_nm_per_ms;
        if (slew < 0.001f) slew = 0.001f;    /* 안전 하한 (너무 작으면 stuck) */

        float delta_lh = target_lh - s_torque_lh_out;
        if (delta_lh >  slew) delta_lh =  slew;
        if (delta_lh < -slew) delta_lh = -slew;
        s_torque_lh_out += delta_lh;

        float delta_rh = target_rh - s_torque_rh_out;
        if (delta_rh >  slew) delta_rh =  slew;
        if (delta_rh < -slew) delta_rh = -slew;
        s_torque_rh_out += delta_rh;

        XM_SetAssistTorqueLH(s_torque_lh_out);
        XM_SetAssistTorqueRH(s_torque_rh_out);
    }
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
     */

    /* EMG raw voltages (PF3/PF4) */
    pf3_volt = s_emg_raw_v[EMG_RH];
    pf4_volt = s_emg_raw_v[EMG_LH];

    /* EMG processed */
    emg_rh_raw_v      = s_emg_raw_v[EMG_RH];
    emg_lh_raw_v      = s_emg_raw_v[EMG_LH];
    emg_rh_envelope_v = s_emg_envelope_v[EMG_RH];
    emg_lh_envelope_v = s_emg_envelope_v[EMG_LH];
    emg_rh_norm       = s_emg_norm_priv[EMG_RH];
    emg_lh_norm       = s_emg_norm_priv[EMG_LH];

    /* System */
    assist_mode_active = (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) ? 1U : 0U;
    squat_phase        = (uint16_t)s_squat_phase;
    user_intent        = (uint16_t)s_user_intent;
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

/* ============================================================================
 * [v2] USER LOOKUP / DWELL / RAMP HELPERS
 * ============================================================================ */

/* 사용자 ASCEND 룩업 — 각도(deg)에 대해 추가 토크(Nm) 반환.
 * 5°~70° 14점 룩업, 선형 보간. 범위 밖은 끝값으로 고정 + 양 끝 페이드.
 * 영점 ±5° 어긋남 흡수용으로 5° 미만 / 65° 초과 영역에서 점진 페이드 적용. */
static float _LookupUserAscend(float angle_deg)
{
    float idx_f;
    int   idx_low;
    float frac;
    float v;

    if (angle_deg <= USER_LOOKUP_BASE_DEG) {
        /* 5° 미만: 0° → 0 으로 페이드 인 */
        if (angle_deg <= 0.0f) { return 0.0f; }
        return s_user_lookup_ascend[0] * (angle_deg / USER_LOOKUP_BASE_DEG);
    }

    idx_f   = (angle_deg - USER_LOOKUP_BASE_DEG) / USER_LOOKUP_STEP_DEG;
    idx_low = (int)idx_f;

    if (idx_low >= (USER_LOOKUP_ASCEND_POINTS - 1)) {
        /* 70° 초과: 끝값 유지 (이미 0인 구간) */
        return s_user_lookup_ascend[USER_LOOKUP_ASCEND_POINTS - 1];
    }

    frac = idx_f - (float)idx_low;
    v = s_user_lookup_ascend[idx_low] * (1.0f - frac) +
        s_user_lookup_ascend[idx_low + 1] * frac;

    if (v < 0.0f) { v = 0.0f; }
    return v;
}

/* Windowed Dwell-time 기반 사용자 의도 갱신.
 * 호출 주기: 1ms (User_Loop ISR).
 *
 * 알고리즘 (sliding window 단순 구현):
 *   - 매 dwell_threshold_ms 마다 윈도우 종료 시점에 angle 변화량 측정.
 *   - 윈도우 시작↔현재 변화량 |Δ| < ANGLE_DEAD → HOLD 확정.
 *   - HOLD 중에는 EXIT 임계로 탈출 검사 (히스테리시스).
 *   - 윈도우 끝마다 시작 각도/시점 재설정.
 *
 * 이유: 단순 "ref 갱신" 방식은 천천히 내려가는 사용자(< DEAD/THRESHOLD 속도)에서
 *       false HOLD 발생. 윈도우 방식은 전체 윈도우 동안 누적 변화를 보므로
 *       속도와 무관하게 정확히 "지난 N ms 동안 멈춰있었는가" 판정.
 *
 * 검출 지연: 최대 ~THRESHOLD_MS×2 (worst case). 200ms 설정 시 ~400ms.
 *           인간 응답 지연(500ms)보다 짧음 → 사용자 체감 문제 없음.
 *
 * 안전: 각속도 미분 안 씀 → 노이즈 강건성 ↑ */
static void _UpdateUserIntent(float avg_angle)
{
    uint32_t now;
    uint32_t window_elapsed;
    float    window_change;
    float    abs_change;

    /* Squat phase 가 STAND/RETURN 이면 의도 추적 의미 없음 — 리셋 */
    if (s_squat_phase == SQUAT_STAND || s_squat_phase == SQUAT_RETURN) {
        s_user_intent              = USER_INTENT_DESCEND;  /* 다음 진입 대비 기본값 */
        s_dwell_window_start_tick  = XM_GetTick();
        s_dwell_window_start_angle = avg_angle;
        return;
    }

    now            = XM_GetTick();
    window_elapsed = now - s_dwell_window_start_tick;

    /* 윈도우 진행 중 — 아직 평가 안 함 (단, HOLD 탈출은 즉시 감지) */
    if (s_user_intent == USER_INTENT_HOLD) {
        /* HOLD 중에는 윈도우 무관하게 큰 움직임 즉시 감지 (히스테리시스) */
        window_change = avg_angle - s_dwell_window_start_angle;
        abs_change    = _AbsFloat(window_change);
        if (abs_change > dwell_exit_angle_deg) {
            if (window_change > 0.0f) {
                s_user_intent = USER_INTENT_DESCEND;
            } else {
                s_user_intent = USER_INTENT_ASCEND;
            }
            s_dwell_window_start_tick  = now;
            s_dwell_window_start_angle = avg_angle;
        }
        return;
    }

    /* DESCEND/ASCEND 상태 — 윈도우 만료까지 대기 */
    if (window_elapsed < (uint32_t)dwell_threshold_ms) {
        return;
    }

    /* 윈도우 만료 — 누적 변화량 평가 */
    window_change = avg_angle - s_dwell_window_start_angle;
    abs_change    = _AbsFloat(window_change);

    if (abs_change < dwell_angle_dead_deg) {
        /* 윈도우 동안 거의 안 움직였음 → HOLD 확정 */
        s_user_intent = USER_INTENT_HOLD;
    } else if (window_change > 0.0f) {
        /* 윈도우 동안 의미 있게 더 내려감 */
        s_user_intent = USER_INTENT_DESCEND;
    } else {
        /* 윈도우 동안 의미 있게 올라감 */
        s_user_intent = USER_INTENT_ASCEND;
    }

    /* 다음 윈도우 시작 */
    s_dwell_window_start_tick  = now;
    s_dwell_window_start_angle = avg_angle;
}

/* 보조 토크 ramp-in (0 → 1) 계산.
 * base_active 가 false → true 로 바뀌는 순간 ramp 시작.
 * 500ms 동안 0 → 1 선형 증가, 이후 1 유지.
 * base_active=false 면 즉시 0 반환 (ramp 리셋).
 *
 * 인간 응답 지연(~500ms)에 맞춰 사용자가 부담 없이 토크에 적응할 시간 부여. */
static float _ComputeAssistRamp(bool base_active)
{
    uint32_t now;
    uint32_t elapsed;
    float    mul;

    if (!base_active) {
        s_assist_ramp_active = false;
        s_prev_base_active   = false;
        return 0.0f;
    }

    /* base_active = true */
    if (!s_prev_base_active) {
        /* edge: 0 → 1 — ramp 시작 */
        s_assist_ramp_tick   = XM_GetTick();
        s_assist_ramp_active = true;
    }
    s_prev_base_active = true;

    if (!s_assist_ramp_active) { return 1.0f; }

    now     = XM_GetTick();
    elapsed = now - s_assist_ramp_tick;
    /* assist_ramp_time_ms 가 0 이면 ramp 비활성 (즉시 1.0). 하한 1ms 로 안전 보장. */
    {
        uint32_t ramp_ms = (assist_ramp_time_ms == 0U) ? 1U : (uint32_t)assist_ramp_time_ms;
        if (elapsed >= ramp_ms) {
            s_assist_ramp_active = false;
            return 1.0f;
        }
        mul = (float)elapsed / (float)ramp_ms;
    }
    return mul;
}
