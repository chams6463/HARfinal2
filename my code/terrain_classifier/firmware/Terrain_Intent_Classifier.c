/**
 ******************************************************************************
 * @file    Terrain_Intent_Classifier.c
 * @brief   [고급] 보행 의도 분류 — 정지 / 평지 / 오르막(경사로) 실시간 구분
 *          (FSR/발접촉 센서 미사용 — 순수 고관절 IMU 기반)
 * @details
 * ============================================================================
 * 1. 배경 및 근거
 * ============================================================================
 * H10은 별도의 깊이 카메라, FSR 발압력 센서, 지형 센서 없이, **고관절
 * IMU(Xsens MTi-630) + 모터 엔코더 + CM 내부 역기구학**만으로 아래 3가지
 * 지형 의도를 구분합니다. isLeftFootContact/isRightFootContact를 포함한
 * 어떤 발-접촉/압력 계열 신호도 사용하지 않습니다.
 *
 * [왜 이 3개 feature인가 — 생체역학 근거]
 * 1) 골반(trunk) 전방 기울기 baseline 증가
 *    McIntosh, Beatty, Dwan & Vickers (2006) "Gait dynamics on an inclined
 *    walkway," J. Biomech. 39(13):2491-502 — 경사각이 커질수록 heel-strike
 *    시점의 hip flexion 및 trunk lean이 경사 방향에 비례해 지속적으로
 *    증가함(오름 방향에서만). 평지에서는 이 baseline이 0 부근에서 진동.
 * 2) 허벅지(thigh) 각도 ROM(Range of Motion) 증가
 *    McIntosh(2006) 동일 논문 — heel-strike 시 hip/knee flexion이 경사에서
 *    커짐 → thigh segment의 stride당 최대-최소 각도 폭이 넓어짐.
 * 3) 고관절 각속도(gyroY) ROM 변화
 *    Qian et al. (2022, 아래 참고문헌)에서 각속도가 지형별로 각도보다
 *    파형 형태 변화가 작은 안정적 주기 신호임을 이용하되, 진폭(ROM) 자체는
 *    보행 강도(경사에서 더 큰 추진력 필요)에 따라 증가하는 경향을 보조
 *    지표로 사용합니다. footContact 없이도 얻을 수 있는 순수 IMU 신호.
 *
 * [센서 융합 이론]
 * pelvicAngle / leftThighAngle / rightThighAngle / hip gyroY 등은 H10 CM
 * 내부에서 IMU(가속도+자이로) 상보/칼만 필터로 이미 융합된 값입니다.
 * 근거: Ch.5-1 Mechanical Sensors and Posture Estimation (PDF)
 *   - 5-2/5-3장 "Complementary Filter" / "Kalman Filter" (p.402-590):
 *     자이로 기반 각도(적분, 저주파 드리프트)와 가속도 기반 각도(고주파
 *     노이즈, 정적 기준 有)를 상보 필터로 융합해 안정적인 orientation 추정.
 *   - 5-4장 "Whole Body Posture Estimation by IMUs" (p.1117-1150):
 *     쿼터니언 회전행렬을 세그먼트 체인에 곱해 관절 위치/각도를 복원하는
 *     방식 — leftThighAngle/rightThighAngle이 바로 이 결과물입니다.
 * 따라서 이 알고리즘은 원시 IMU를 다시 필터링하지 않고, CM이 이미 융합한
 * 값을 그대로 사용합니다(중복 필터링 불필요).
 *
 * [Stride 세그멘테이션 — FSR/발접촉 미사용]
 * 일반적인 Heel-Strike 기반 stride 분할(Ex.23, Ex.32)은 isLeftFootContact
 * (발 접촉/압력 계열) rising edge를 이벤트로 씁니다. 이 알고리즘은 그 대신
 * **Maximum Hip Extension(MHE)** — 허벅지 각도가 가장 신전된(최솟값)
 * 시점 — 을 stride 경계 이벤트로 사용합니다.
 *   근거: Qian, Y. et al. (2022) "Predictive Locomotion Mode Recognition and
 *   Accurate Gait Phase Estimation for Hip Exoskeleton on Various Terrains,"
 *   IEEE RA-L 7(3):6439-6446, Sec.II-C-2 — "Heel Strike는 무릎 위 착용 센서
 *   만으로는 안정적으로 검출하기 어렵다. 따라서 MHF/MHE(최대 고관절 굴곡/
 *   신전)를 고관절 외골격의 특징적 보행 이벤트로 널리 채택한다."
 * thigh_avg(t)의 국소 최솟값(감소->증가 전환)을 디바운스와 함께 검출합니다.
 *
 * [분류 방식 — Fuzzy Weighted Score + 다수결 필터]
 * XM_Apps/Final_FSR_Fuzzy_Logic.c와 동일하게 tanh 기반 fuzzy score를 쓰되,
 * Qian et al. (2022)에서 쓰인 "majority filter(다수결 스무딩)" 개념을
 * stride 단위 투표 버퍼로 재구현해 순간적인 오분류로 인한 모드 flickering을
 * 방지합니다.
 *
 * ============================================================================
 * 2. 알고리즘 개요
 * ============================================================================
 *   [매 1ms]  forwardVelocity로 정지 여부 판정 (즉시 반영, FSR 아님 —
 *             CM의 IMU 기반 보행속도 추정치)
 *   [Stride 중] pelvicAngle 누적(평균용), thigh angle/hip gyroY min-max 갱신
 *   [MHE 이벤트 시] stride 종료 → 3개 feature 확정
 *     -> 캘리브레이션 중이면: 평지 baseline 누적
 *     -> 캘리브레이션 완료 후: baseline 대비 편차로 fuzzy score 계산
 *        -> score를 다수결 투표 버퍼에 push
 *        -> 버퍼 과반 이상이 INCLINE 판정이면 상태 전환(히스테리시스)
 *
 * ============================================================================
 * 3. 필수 사전 조건 (Body Data)
 * ============================================================================
 * leftThighAngle/rightThighAngle/pelvicAngle/hip gyroY/forwardVelocity는
 * 모두 H10 CM의 실시간 보행 분석(1kHz) 출력입니다. XM_SendUserBodyData()로
 * 신체 정보(체중/키/분절 길이)가 설정되어 있지 않으면 이 값들이 부정확할
 * 수 있습니다. (Ex.23, Ex.32와 동일한 제약 — 단, footContact는 사용하지
 * 않으므로 발-접촉 관련 Body Data 의존성은 없습니다.)
 *
 * ============================================================================
 * 4. 버튼 조작
 * ============================================================================
 *   BTN1 클릭: 캘리브레이션 시작/재시작
 *     -> 평지에서 실제로 걸으며 누르세요. CALIB_STRIDE_COUNT(5) stride
 *        동안 평지 baseline(trunk lean, thigh ROM, hip gyroY ROM)을 수집.
 *   BTN2 클릭: 투표 버퍼 초기화 (지형이 갑자기 바뀌어 판정이 꼬였을 때)
 *   BTN3 클릭: 분류 민감도(SENSITIVITY) 3단계 순환 (둔감 <-> 민감)
 *
 * [LED 표시]
 *   LED1: ACTIVE 상태 heartbeat
 *   LED2: 캘리브레이션 진행 중 점멸, 완료 시 상시 점등
 *   LED3: 현재 판정 — OFF(정지) / 상시점등(평지) / 빠른 점멸(오르막)
 *
 * @see     Ex.15 inverted_pendulum_control.c (pelvicAngle + hip gyroY 사용 패턴)
 * @see     Ex.23 gait_phase_adaptive_torque.c (stride 추정 구조 참고, 단 본
 *          파일은 footContact 대신 MHE 이벤트로 대체)
 * @see     XM_Apps/User_Algorithm/Final_FSR_Fuzzy_Logic.c (tanh fuzzy score 패턴)
 * @see     Riener, R., Rabuffetti, M., Frigo, C. (2002) "Stair ascent and
 *          descent at different inclinations," Gait & Posture 15(1):32-44.
 * @see     McIntosh, A.S. et al. (2006) "Gait dynamics on an inclined
 *          walkway," J. Biomech. 39(13):2491-502.
 * @see     Qian, Y. et al. (2022) "Predictive Locomotion Mode Recognition and
 *          Accurate Gait Phase Estimation for Hip Exoskeleton on Various
 *          Terrains," IEEE Robotics and Automation Letters, 7(3):6439-6446.
 * @version 1.1
 * @date    Aug 02, 2026
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api.h"
#include <math.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/* --- 정지 판정 (forwardVelocity는 CM의 IMU 기반 추정치 — FSR 아님) --- */
#define MIN_FWD_VELOCITY        0.1f    /* m/s — 이하이면 정지로 판단(Ex.23과 동일 임계값) */

/* --- Stride 유효성 방어 --- */
#define STRIDE_MIN_MS           400U    /* 최소 stride 주기(잡음 방지) */
#define STRIDE_MAX_MS           3000U   /* 최대 stride 주기(타임아웃) */

/* --- MHE(Maximum Hip Extension) 이벤트 검출 --- */
#define MHE_MIN_SWING_DEG       3.0f    /* 국소 최솟값 확정 전 최소 반등 폭(deg) — 잡음 디바운스 */
#define MHE_REFRACTORY_MS       250U    /* 이벤트 후 최소 불응기간(ms) — 중복 검출 방지 */

/* --- 캘리브레이션 --- */
#define CALIB_STRIDE_COUNT      5U      /* 평지 baseline 수집 stride 수 */

/* --- Fuzzy score 가중치 (경험적 스케일: '이 정도 편차면 score가 포화') --- */
#define TRUNK_LEAN_SCALE_DEG    6.0f    /* trunk lean 편차 6deg -> 포화 근접 */
#define THIGH_ROM_SCALE_DEG     10.0f   /* thigh ROM 편차 10deg -> 포화 근접 */
#define GYRO_ROM_SCALE_DPS      40.0f   /* hip gyroY ROM 편차 40deg/s -> 포화 근접 */

#define W_TRUNK_LEAN            0.5f
#define W_THIGH_ROM             0.3f
#define W_GYRO_ROM              0.2f

/* --- 판정 히스테리시스 (score: -1~+1) --- */
#define INCLINE_SCORE_TH        0.35f   /* 이 값 이상 -> "오르막 성향" 투표 */
#define LEVEL_SCORE_TH          0.15f   /* 이 값 이하 -> "평지 성향" 투표 (완충지대: 0.15~0.35) */

/* --- 다수결 투표 버퍼 --- */
#define VOTE_WINDOW              5U     /* 최근 5 stride로 다수결 (IFAC majority filter 개념) */
#define VOTE_INCLINE_MAJORITY    3U     /* 5개 중 3개 이상 INCLINE 성향 -> 상태 전환 */
#define VOTE_LEVEL_MAJORITY      3U     /* 5개 중 3개 이상 LEVEL 성향 -> 상태 전환 */

/* --- 민감도 프리셋 (BTN3) --- */
#define NUM_SENSITIVITY_LEVELS   3U
static const float k_sensitivity_scale[NUM_SENSITIVITY_LEVELS] = { 0.7f, 1.0f, 1.4f }; /* 둔감/기본/민감 */

/* --- 제어 루프 타이밍 --- */
#define CONTROL_DT               0.001f
#define USB_DEBUG_PERIOD_MS      300U

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/** @brief 보행 의도(지형) 분류 결과 */
typedef enum {
    TERRAIN_STATIONARY = 0,   /* 정지 */
    TERRAIN_LEVEL,             /* 평지 보행 */
    TERRAIN_INCLINE,           /* 오르막(경사로) 보행 */
} TerrainIntent_t;

/** @brief 진행 중인 stride의 누적 accumulator (모두 IMU/엔코더 파생값) */
typedef struct {
    float    trunk_sum_deg;     /* pelvicAngle 누적합 (평균용) */
    uint32_t sample_count;      /* 누적 샘플 수 */
    float    thigh_max_deg;     /* stride 내 thigh angle 최대값 */
    float    thigh_min_deg;     /* stride 내 thigh angle 최소값 */
    float    gyro_max_dps;      /* stride 내 hip gyroY 최대값 */
    float    gyro_min_dps;      /* stride 내 hip gyroY 최소값 */
    uint32_t stride_start_tick; /* stride 시작 tick */
} StrideAccumulator_t;

/** @brief 완성된 1 stride의 feature */
typedef struct {
    float trunk_lean_mean_deg;
    float thigh_rom_deg;
    float gyro_rom_dps;
    bool  valid;                /* stride 유효성 */
} StrideFeature_t;

/** @brief 평지 baseline (캘리브레이션 결과) */
typedef struct {
    float trunk_lean_deg;
    float thigh_rom_deg;
    float gyro_rom_dps;
    bool  is_done;
} TerrainBaseline_t;

/** @brief USB 스트리밍용 데이터 구조체 */
typedef struct {
    float trunk_lean_dev_deg;   /* baseline 대비 trunk lean 편차 */
    float thigh_rom_dev_deg;    /* baseline 대비 thigh ROM 편차 */
    float gyro_rom_dev_dps;     /* baseline 대비 hip gyroY ROM 편차 */
    float fuzzy_score;          /* -1~+1 */
    float terrain_intent;       /* 0=정지,1=평지,2=오르막 (열거형을 float로) */
} TerrainStreamData_t;

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

static XmTsmHandle_t s_tsm;

/* --- Stride 세그멘테이션 상태 (MHE 이벤트 검출용) --- */
static StrideAccumulator_t  s_acc;
static bool                 s_has_first_mhe = false;
static float                s_prev_thigh_avg = 0.0f;
static bool                 s_thigh_was_falling = false;   /* 직전 샘플 대비 하강 중이었는지 */
static float                s_swing_peak_before_min_deg = 0.0f; /* 국소 최솟값 확정용 디바운스 */
static uint32_t              s_last_mhe_tick = 0;

/* --- 캘리브레이션 상태 --- */
static bool               s_calibrating = false;
static uint8_t             s_calib_stride_idx = 0;
static float               s_calib_trunk_sum = 0.0f;
static float               s_calib_rom_sum = 0.0f;
static float               s_calib_gyro_sum = 0.0f;
static TerrainBaseline_t   s_baseline = { 0 };

/* --- 다수결 투표 버퍼 (true = INCLINE 성향 투표) --- */
static bool     s_vote_buf[VOTE_WINDOW];
static uint8_t  s_vote_idx = 0;
static uint8_t  s_vote_filled = 0;

/* --- 민감도 --- */
static uint8_t s_sensitivity_idx = 1;  /* 기본: 1.0 배율 */

/* --- 현재 판정 결과 --- */
static TerrainIntent_t s_current_intent = TERRAIN_STATIONARY;
static float            s_last_fuzzy_score = 0.0f;
static float            s_last_trunk_dev = 0.0f;
static float            s_last_rom_dev = 0.0f;
static float            s_last_gyro_dev = 0.0f;

/* --- USB 디버그 / 스트리밍 --- */
static uint32_t            s_usb_debug_timer = 0;
static TerrainStreamData_t s_stream_data;

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static void Off_Loop(void);
static void Standby_Loop(void);
static void Active_Entry(void);
static void Active_Loop(void);
static void Active_Exit(void);

static void  _ResetStrideAccumulator(uint32_t tick);
static void  _AccumulateStrideSample(float thigh_avg, float gyro_avg);
static bool  _DetectMheEvent(float thigh_avg, uint32_t tick);
static StrideFeature_t _FinalizeStride(uint32_t tick);
static void  _ProcessCompletedStride(const StrideFeature_t *feat);
static float _ComputeFuzzyScore(const StrideFeature_t *feat);
static void  _PushVote(bool incline_leaning);
static void  _UpdateIntentFromVotes(void);
static void  _StartCalibration(void);

static void  _HandleButtonInput(void);
static void  _UpdateLedIndicators(void);
static void  _UpdateUsbDebug(void);
static void  _UpdateStreamData(void);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

void User_Setup(void)
{
    s_tsm = XM_TSM_Create(XM_STATE_OFF);

    XmStateConfig_t off_conf = { .id = XM_STATE_OFF, .on_loop = Off_Loop };
    XM_TSM_AddState(s_tsm, &off_conf);

    XmStateConfig_t sb_conf = { .id = XM_STATE_STANDBY, .on_loop = Standby_Loop };
    XM_TSM_AddState(s_tsm, &sb_conf);

    XmStateConfig_t act_conf = {
        .id       = XM_STATE_ACTIVE,
        .on_entry = Active_Entry,
        .on_loop  = Active_Loop,
        .on_exit  = Active_Exit
    };
    XM_TSM_AddState(s_tsm, &act_conf);

    XM_SetUsbCustomMeta(0xF3,
        "[{\"name\":\"Trunk Dev\",\"unit\":\"deg\"},"
        "{\"name\":\"ROM Dev\",\"unit\":\"deg\"},"
        "{\"name\":\"Gyro ROM Dev\",\"unit\":\"deg/s\"},"
        "{\"name\":\"Fuzzy Score\",\"unit\":\"-\"},"
        "{\"name\":\"Terrain Intent\",\"unit\":\"-\"}]");

    /* 토크를 인가하지 않는 순수 분류기 — 모니터링 모드로 충분 */
    XM_SetControlMode(XM_CTRL_MONITOR);
}

void User_Loop(void)
{
    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF);
    }
    XM_TSM_Run(s_tsm);
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS
 *-----------------------------------------------------------
 */

// ==================== Task State Machine 콜백 ====================

static void Off_Loop(void)
{
    if (XM_IsCmConnected()) {
        XM_SendUsbDebugMessage("[TIC] CM 연결됨 -> STANDBY\r\n");
        XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
    }
}

static void Standby_Loop(void)
{
    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_SendUsbDebugMessage("[TIC] ASSIST 감지 -> ACTIVE\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_ACTIVE);
    }
}

static void Active_Entry(void)
{
    uint32_t tick = XM_GetTick();

    s_has_first_mhe = false;
    s_thigh_was_falling = false;
    s_swing_peak_before_min_deg = 0.0f;
    s_last_mhe_tick = tick;
    s_prev_thigh_avg = (XM.status.h10.leftThighAngle + XM.status.h10.rightThighAngle) * 0.5f;
    _ResetStrideAccumulator(tick);

    s_calibrating = false;
    s_calib_stride_idx = 0;
    s_calib_trunk_sum = 0.0f;
    s_calib_rom_sum = 0.0f;
    s_calib_gyro_sum = 0.0f;
    /* baseline은 유지 — 이전 세션에서 캘리브레이션했다면 재사용 가능 */

    s_vote_idx = 0;
    s_vote_filled = 0;
    s_current_intent = TERRAIN_STATIONARY;
    s_last_fuzzy_score = 0.0f;

    s_usb_debug_timer = tick;

    _UpdateLedIndicators();

    XM_SendUsbDebugMessage("[TIC] ACTIVE — 지형 의도 분류 시작 (FSR/발접촉 센서 미사용)\r\n");
    if (!s_baseline.is_done) {
        XM_SendUsbDebugMessage("[TIC] 캘리브레이션 필요: 평지에서 걸으며 BTN1을 누르세요\r\n");
    }
}

/**
 * @brief ACTIVE 루프 — 정지 판정 -> stride 누적/완결(MHE 이벤트) -> 분류 (1ms)
 */
static void Active_Loop(void)
{
    if (XM.status.h10.h10Mode != XM_H10_MODE_ASSIST) {
        XM_SendUsbDebugMessage("[TIC] ASSIST 해제 -> STANDBY\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
        return;
    }

    _HandleButtonInput();

    uint32_t tick = XM_GetTick();
    float fwd_vel = XM.status.h10.forwardVelocity;

    float thigh_avg = (XM.status.h10.leftThighAngle + XM.status.h10.rightThighAngle) * 0.5f;
    float gyro_avg  = (XM.status.h10.leftHipImuGlobalGyrY + XM.status.h10.rightHipImuGlobalGyrY) * 0.5f;

    /* --- 정지 판정: 즉시 반영, stride 누적은 보류 --- */
    if (fwd_vel < MIN_FWD_VELOCITY) {
        s_current_intent = TERRAIN_STATIONARY;
        s_prev_thigh_avg = thigh_avg;      /* 재개 시 오검출 방지 */
        s_thigh_was_falling = false;
        _ResetStrideAccumulator(tick);
        _UpdateLedIndicators();
        _UpdateUsbDebug();
        _UpdateStreamData();
        return;
    }

    /* --- Stride 진행 중 누적 --- */
    _AccumulateStrideSample(thigh_avg, gyro_avg);

    /* --- MHE(허벅지 최대 신전) 이벤트 검출: footContact 미사용 --- */
    if (_DetectMheEvent(thigh_avg, tick)) {
        StrideFeature_t feat = _FinalizeStride(tick);
        if (s_has_first_mhe && feat.valid) {
            _ProcessCompletedStride(&feat);
        }
        s_has_first_mhe = true;
        s_last_mhe_tick = tick;
        _ResetStrideAccumulator(tick);
    }

    _UpdateLedIndicators();
    _UpdateUsbDebug();
    _UpdateStreamData();
}

static void Active_Exit(void)
{
    XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
    XM_SetLedState(XM_LED_2, XM_OFF);
    XM_SetLedState(XM_LED_3, XM_OFF);
    XM_SendUsbDebugMessage("[TIC] 제어 종료\r\n");
}

// ==================== Stride 세그멘테이션 (MHE 기반, FSR/발접촉 미사용) ====================

static void _ResetStrideAccumulator(uint32_t tick)
{
    s_acc.trunk_sum_deg = 0.0f;
    s_acc.sample_count = 0;
    s_acc.thigh_max_deg = -1000.0f;
    s_acc.thigh_min_deg = 1000.0f;
    s_acc.gyro_max_dps = -1000.0f;
    s_acc.gyro_min_dps = 1000.0f;
    s_acc.stride_start_tick = tick;
}

/**
 * @brief 매 1ms 호출 — 현재 stride의 feature 원재료를 누적
 * @details thigh angle/hip gyroY 모두 좌우 평균을 사용해 좌우 비대칭 노이즈를 줄입니다.
 *          발 접촉/압력 신호는 일절 참조하지 않습니다.
 */
static void _AccumulateStrideSample(float thigh_avg, float gyro_avg)
{
    float trunk = XM.status.h10.pelvicAngle;

    s_acc.trunk_sum_deg += trunk;
    s_acc.sample_count++;

    if (thigh_avg > s_acc.thigh_max_deg) { s_acc.thigh_max_deg = thigh_avg; }
    if (thigh_avg < s_acc.thigh_min_deg) { s_acc.thigh_min_deg = thigh_avg; }

    if (gyro_avg > s_acc.gyro_max_dps) { s_acc.gyro_max_dps = gyro_avg; }
    if (gyro_avg < s_acc.gyro_min_dps) { s_acc.gyro_min_dps = gyro_avg; }
}

/**
 * @brief Maximum Hip Extension(MHE) 이벤트 검출 — thigh_avg의 국소 최솟값
 * @details
 * thigh_avg가 하강하다가(신전 진행) 다시 상승으로 전환되는 순간을 MHE로
 * 간주합니다. 잡음으로 인한 미세한 반등을 걸러내기 위해:
 *   1) 하강 -> 상승 전환 후 MHE_MIN_SWING_DEG 이상 반등해야 확정
 *   2) 직전 이벤트로부터 MHE_REFRACTORY_MS 이상 경과해야 재검출 허용
 * (Qian et al. 2022, Sec.II-C-2 — 발 위 센서 없이도 검출 가능한 고관절
 *  특징 이벤트로 MHF/MHE를 채택한 근거와 동일한 아이디어)
 * @return true면 이번 호출에서 MHE 이벤트 확정
 */
static bool _DetectMheEvent(float thigh_avg, uint32_t tick)
{
    bool event_fired = false;
    float delta = thigh_avg - s_prev_thigh_avg;

    if (delta < 0.0f) {
        /* 하강 중(신전 진행) */
        s_thigh_was_falling = true;
        s_swing_peak_before_min_deg = 0.0f; /* 반등 누적 리셋 */
    } else if (delta > 0.0f && s_thigh_was_falling) {
        /* 하강 이후 반등 중 — 반등 폭 누적 */
        s_swing_peak_before_min_deg += delta;

        bool refractory_ok = (tick - s_last_mhe_tick) >= MHE_REFRACTORY_MS;
        if (s_swing_peak_before_min_deg >= MHE_MIN_SWING_DEG && refractory_ok) {
            event_fired = true;
            s_thigh_was_falling = false;
            s_swing_peak_before_min_deg = 0.0f;
        }
    }

    s_prev_thigh_avg = thigh_avg;
    return event_fired;
}

/**
 * @brief MHE 이벤트 시점에 stride를 마감하고 feature를 계산
 * @details stride 주기가 비정상(너무 짧거나 김)이면 invalid 처리
 */
static StrideFeature_t _FinalizeStride(uint32_t tick)
{
    StrideFeature_t feat = { 0 };
    uint32_t stride_ms = tick - s_acc.stride_start_tick;

    if (stride_ms < STRIDE_MIN_MS || stride_ms > STRIDE_MAX_MS || s_acc.sample_count == 0U) {
        feat.valid = false;
        return feat;
    }

    feat.trunk_lean_mean_deg = s_acc.trunk_sum_deg / (float)s_acc.sample_count;
    feat.thigh_rom_deg       = s_acc.thigh_max_deg - s_acc.thigh_min_deg;
    feat.gyro_rom_dps        = s_acc.gyro_max_dps - s_acc.gyro_min_dps;
    feat.valid = true;
    return feat;
}

/**
 * @brief 완성된 stride를 캘리브레이션 또는 분류에 반영
 */
static void _ProcessCompletedStride(const StrideFeature_t *feat)
{
    if (s_calibrating) {
        s_calib_trunk_sum += feat->trunk_lean_mean_deg;
        s_calib_rom_sum   += feat->thigh_rom_deg;
        s_calib_gyro_sum  += feat->gyro_rom_dps;
        s_calib_stride_idx++;

        char buf[64];
        snprintf(buf, sizeof(buf), "[TIC] 캘리브레이션 %u/%u stride 수집\r\n",
                 s_calib_stride_idx, CALIB_STRIDE_COUNT);
        XM_SendUsbDebugMessage(buf);

        if (s_calib_stride_idx >= CALIB_STRIDE_COUNT) {
            s_baseline.trunk_lean_deg = s_calib_trunk_sum / (float)CALIB_STRIDE_COUNT;
            s_baseline.thigh_rom_deg  = s_calib_rom_sum / (float)CALIB_STRIDE_COUNT;
            s_baseline.gyro_rom_dps   = s_calib_gyro_sum / (float)CALIB_STRIDE_COUNT;
            s_baseline.is_done = true;
            s_calibrating = false;

            snprintf(buf, sizeof(buf), "[TIC] 캘리브레이션 완료: trunk=%.1fdeg rom=%.1fdeg gyroRom=%.1fdps\r\n",
                     (double)s_baseline.trunk_lean_deg,
                     (double)s_baseline.thigh_rom_deg,
                     (double)s_baseline.gyro_rom_dps);
            XM_SendUsbDebugMessage(buf);
        }
        return; /* 캘리브레이션 중에는 분류하지 않음 */
    }

    if (!s_baseline.is_done) {
        return; /* 캘리브레이션 전에는 항상 STATIONARY/미분류 상태 유지 */
    }

    float score = _ComputeFuzzyScore(feat);
    s_last_fuzzy_score = score;

    bool incline_leaning = (score >= INCLINE_SCORE_TH);
    bool level_leaning    = (score <= LEVEL_SCORE_TH);

    /* 완충지대(LEVEL_SCORE_TH < score < INCLINE_SCORE_TH)는 투표를 넣지 않고
     * 직전 다수결 상태를 유지 — 급격한 flip-flop 방지 */
    if (incline_leaning) {
        _PushVote(true);
    } else if (level_leaning) {
        _PushVote(false);
    }

    _UpdateIntentFromVotes();
}

/**
 * @brief baseline 대비 편차 3종을 가중합해 tanh로 압축한 fuzzy score(-1~+1) 계산
 * @details Final_FSR_Fuzzy_Logic.c의 tanh membership 패턴과 동일한 형태.
 *          score > 0: 오르막 성향, score < 0: 평지 성향(또는 반대 방향 편차)
 */
static float _ComputeFuzzyScore(const StrideFeature_t *feat)
{
    float trunk_dev = feat->trunk_lean_mean_deg - s_baseline.trunk_lean_deg;
    float rom_dev   = feat->thigh_rom_deg - s_baseline.thigh_rom_deg;
    float gyro_dev  = feat->gyro_rom_dps - s_baseline.gyro_rom_dps;

    s_last_trunk_dev = trunk_dev;
    s_last_rom_dev   = rom_dev;
    s_last_gyro_dev  = gyro_dev;

    float norm_trunk = trunk_dev / TRUNK_LEAN_SCALE_DEG;
    float norm_rom   = rom_dev / THIGH_ROM_SCALE_DEG;
    float norm_gyro  = gyro_dev / GYRO_ROM_SCALE_DPS;

    float weighted = (W_TRUNK_LEAN * norm_trunk)
                    + (W_THIGH_ROM * norm_rom)
                    + (W_GYRO_ROM * norm_gyro);

    float sensitivity = k_sensitivity_scale[s_sensitivity_idx];
    return tanhf(sensitivity * weighted);
}

/**
 * @brief 다수결 투표 버퍼(circular)에 이번 stride의 판정을 push
 */
static void _PushVote(bool incline_leaning)
{
    s_vote_buf[s_vote_idx] = incline_leaning;
    s_vote_idx = (uint8_t)((s_vote_idx + 1U) % VOTE_WINDOW);
    if (s_vote_filled < VOTE_WINDOW) { s_vote_filled++; }
}

/**
 * @brief 투표 버퍼 과반 이상으로 최종 상태 전환(히스테리시스)
 * @details 버퍼가 다 차지 않았으면(초반) 판단을 보류하고 현재 상태 유지
 */
static void _UpdateIntentFromVotes(void)
{
    if (s_vote_filled < VOTE_WINDOW) {
        return; /* 아직 다수결 판단에 충분한 데이터가 없음 */
    }

    uint8_t incline_votes = 0;
    for (uint8_t i = 0; i < VOTE_WINDOW; i++) {
        if (s_vote_buf[i]) { incline_votes++; }
    }
    uint8_t level_votes = VOTE_WINDOW - incline_votes;

    if (incline_votes >= VOTE_INCLINE_MAJORITY) {
        s_current_intent = TERRAIN_INCLINE;
    } else if (level_votes >= VOTE_LEVEL_MAJORITY) {
        s_current_intent = TERRAIN_LEVEL;
    }
    /* 어느 쪽도 과반이 아니면(3:2 미만 격차 등 이론상 없음) 이전 상태 유지 */
}

static void _StartCalibration(void)
{
    s_calibrating = true;
    s_calib_stride_idx = 0;
    s_calib_trunk_sum = 0.0f;
    s_calib_rom_sum = 0.0f;
    s_calib_gyro_sum = 0.0f;
    XM_SendUsbDebugMessage("[TIC] 캘리브레이션 시작 — 평지에서 5 stride 동안 걸어주세요\r\n");
}

// ==================== 버튼 / LED / 디버그 ====================

static void _HandleButtonInput(void)
{
    /* BTN1: 캘리브레이션 시작/재시작 (평지에서 눌러야 함) */
    if (XM_GetButtonEvent(XM_BTN_1) == XM_BTN_CLICK) {
        _StartCalibration();
    }

    /* BTN2: 투표 버퍼 초기화 (판정이 꼬였을 때 수동 리셋) */
    if (XM_GetButtonEvent(XM_BTN_2) == XM_BTN_CLICK) {
        s_vote_idx = 0;
        s_vote_filled = 0;
        XM_SendUsbDebugMessage("[TIC] BTN2: 투표 버퍼 리셋\r\n");
    }

    /* BTN3: 분류 민감도 3단계 순환 */
    if (XM_GetButtonEvent(XM_BTN_3) == XM_BTN_CLICK) {
        s_sensitivity_idx = (uint8_t)((s_sensitivity_idx + 1U) % NUM_SENSITIVITY_LEVELS);
        char buf[48];
        snprintf(buf, sizeof(buf), "[TIC] BTN3: 민감도 = %.1fx\r\n",
                 (double)k_sensitivity_scale[s_sensitivity_idx]);
        XM_SendUsbDebugMessage(buf);
    }
}

static void _UpdateLedIndicators(void)
{
    /* LED1: ACTIVE heartbeat */
    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 500);

    /* LED2: 캘리브레이션 상태 */
    if (s_calibrating) {
        XM_SetLedEffect(XM_LED_2, XM_LED_BLINK, 200);
    } else if (s_baseline.is_done) {
        XM_SetLedState(XM_LED_2, XM_ON);
    } else {
        XM_SetLedState(XM_LED_2, XM_OFF);
    }

    /* LED3: 현재 판정 결과 */
    switch (s_current_intent) {
        case TERRAIN_STATIONARY:
            XM_SetLedState(XM_LED_3, XM_OFF);
            break;
        case TERRAIN_LEVEL:
            XM_SetLedState(XM_LED_3, XM_ON);
            break;
        case TERRAIN_INCLINE:
            XM_SetLedEffect(XM_LED_3, XM_LED_BLINK, 150);
            break;
        default:
            XM_SetLedState(XM_LED_3, XM_OFF);
            break;
    }
}

static void _UpdateUsbDebug(void)
{
    uint32_t now = XM_GetTick();
    if (now - s_usb_debug_timer < USB_DEBUG_PERIOD_MS) { return; }
    s_usb_debug_timer = now;

    const char *intent_str = "STATIONARY";
    if (s_current_intent == TERRAIN_LEVEL)   { intent_str = "LEVEL"; }
    if (s_current_intent == TERRAIN_INCLINE) { intent_str = "INCLINE"; }

    char buf[128];
    snprintf(buf, sizeof(buf),
             "TIC | %-10s score=%.2f dTrunk=%.1f dRom=%.1f dGyroRom=%.1f calib=%d\r\n",
             intent_str,
             (double)s_last_fuzzy_score,
             (double)s_last_trunk_dev,
             (double)s_last_rom_dev,
             (double)s_last_gyro_dev,
             s_baseline.is_done ? 1 : 0);
    XM_SendUsbDebugMessage(buf);
}

static void _UpdateStreamData(void)
{
    s_stream_data.trunk_lean_dev_deg = s_last_trunk_dev;
    s_stream_data.thigh_rom_dev_deg  = s_last_rom_dev;
    s_stream_data.gyro_rom_dev_dps   = s_last_gyro_dev;
    s_stream_data.fuzzy_score        = s_last_fuzzy_score;
    s_stream_data.terrain_intent     = (float)s_current_intent;
    XM_SendUsbDataWithId(&s_stream_data, sizeof(s_stream_data), 0xF3);
}
