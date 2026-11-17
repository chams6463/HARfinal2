/**
 ******************************************************************************
 * @file    Locomotion_Mode_Classifier.c
 * @brief   7-모드 실시간 지형/보행의도 분류기
 *          STAND / WALK / TURN / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN
 *          (H10 right footContact + 고관절 IMU + 외부 ADC EMG 기반)
 *
 * @details
 * ============================================================================
 * 1. 아키텍처 — 3층 구조 (왜 7모드를 한 뱅크로 안 돌리는가)
 * ============================================================================
 * 이 파일은 Terrain_Intent_Classifier.c 의 3-class(정지/평지/오르막) 구조를
 * 7-class 로 확장하되, 7모드를 하나의 z-score 뱅크에 넣지 않는다.
 * Camargo(2021) 데이터셋 7명(AB06~AB12) 실측 검증 결과 STAND 와 TURN 은
 * 위상 기준곡선(reference curve) r^m(theta) 로 판별할 수 없음이 확인되었기
 * 때문이다.
 *
 *   [층 1] STAND 게이트 (기준곡선 미사용)
 *       forwardVelocity + 고관절 각속도 에너지로 즉시 판정.
 *       근거: Camargo 의 'idle' 라벨은 "정지"가 아니다. 라벨 run 단위로
 *       실측한 결과 idle 의 median |w| = 0.923 rad/s (53 deg/s) 로
 *       STAIRS_DOWN(0.872 rad/s)보다 오히려 크다. idle 구간은 보행로 끝에서
 *       방향을 바꾸고 자리를 잡는 동작을 포함한다. 게다가 idle run 길이가
 *       0.09 s ~ 9.51 s (100:1) 로 제각각이라, 이를 균등분할해 만든
 *       r^STAND(theta) 는 위상 의미가 없는 "상수 + 큰 분산"이다.
 *       => 기준곡선으로 쓸 수 없다. 속도/에너지 게이트가 정답.
 *       (이는 브리핑 Tier4 의 STAND_TO_WALK 결론과도 일치한다.)
 *
 *   [층 2] TURN 검출기 (기준곡선 미사용)
 *       고관절 IMU 의 TransverseYaw 변화량(heading change)으로 판정.
 *       근거: TURN vs WALK 는 41개 feature 전부에서 위상분해 Cohen's d 가
 *       최대 0.80 에 그치고, EMG 자리를 3개까지 늘려도 전혀 개선되지 않는다.
 *       회전은 본질적으로 yaw 현상인데 기준곡선의 IMU feature 는 sagittal
 *       atan2(AccX,AccZ) 와 gyro '크기(magnitude)' 뿐이라 방향 정보가 아예
 *       없기 때문이다. 반면 H10 CM 은 Camargo 에 없는 좌우 TransverseYaw 를
 *       실제로 제공한다 => 여기서 판별하는 것이 유일하게 타당한 경로.
 *
 *   [층 2.5] IDLE(전이/재배치) 전환 증거
 *       Camargo 의 'idle' 라벨은 원래 "보행로 끝에서 방향을 바꾸고 자리를 잡는"
 *       거쳐가는 상태다. 이 상태의 위상축 정보량(flatness = 위상간분산/전체분산)은
 *       1.7~7.1% 로 사실상 0 이다(보행모드는 11~59%). 즉 idle 은 위상 구조가
 *       없는 상태이며, 101점 곡선 대신 스칼라 (mean,std) 한 쌍으로 손실 없이
 *       표현된다 (ref_curve_tables.h 의 kRcIdleMean/kRcIdleStd, 32 B).
 *
 *       이 평탄 템플릿 대비 z_IDLE 을 계산하면, 보행 중에는 x(theta) 가 크게
 *       출렁여 평탄 템플릿에서 멀어지므로 z_IDLE 이 커지고, 전이 구간에는
 *       x(theta) 가 평탄해 z_IDLE 이 작아진다 — "파형이 있는가" 자체가 신호다.
 *
 *       기존 구현은 z_IDLE < z_best 이면 무조건 투표를 막았다. 그러나 연속
 *       보행 검증에서 이 조건이 새 지형의 첫 transition stride 까지 버려
 *       3/2 다수결에 필요한 첫 표가 사라지는 문제가 확인됐다.
 *       개선 구현은 IDLE 을 모드로 쓰지 않고 "전환 가능성"으로만 쓴다.
 *       안정 stride 는 기존 margin 0.35를 유지하고, IDLE이면서 현재 모드와
 *       다른 후보에 한해 margin 0.15부터 표를 허용한다. 3칸/2표 다수결은
 *       변경하지 않아 한 stride 하나만으로 모드가 바뀌지는 않는다.
 *
 *   [층 3] z-score 뱅크 (기준곡선 사용, 보행 5모드)
 *       WALK / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN 만 담당.
 *       우측 heel-strike 위상 theta(0~1) 에서 r^m(theta) 대비 z 를 5모드
 *       동시에 계산하고
 *       (뱅크 구조), z 가 가장 작은 모드를 채택한다. 2위와의 마진을 tanh 로
 *       압축해 dual-threshold 히스테리시스 + stride 단위 다수결에 넣는다.
 *       (tanh fuzzy / 다수결 / 캘리브레이션 / 버튼·LED 배치 / USB 스트리밍
 *        관례는 Terrain_Intent_Classifier.c 의 검증된 패턴을 그대로 재사용)
 *
 * ============================================================================
 * 2. feature 선정 근거 (41개 -> 4개)
 * ============================================================================
 *   IMU_trunk_TILT      채택. 판별력의 50%가 DC(자세 기울기)에서 나온다.
 *                       모드별 평균이 STAIRS_UP -93.4 / WALK -80.9 /
 *                       RAMP_DOWN -76.6 deg 로 뚜렷이 다르며, 이는 실제
 *                       체간 전후 경사다. H10 pelvicAngle은 좌우 tilt라 쓰지
 *                       않고 좌우 HipImuSagittalPitch 평균을 proxy로 사용한다.
 *   IMU_thigh_GYROMAG   채택. theta 0.10(입각 초기)과 0.60~0.70(swing 개시)에
 *                       두 개의 물리적으로 타당한 피크를 갖는 깨끗한 파형.
 *   EMG site0 RMS       채택. greedy 선택 1순위(gastrocnemius medialis,
 *                       15쌍 합계 gain +5.44 로 압도적 1위).
 *   EMG site1 RMS       채택. greedy 2순위(vastus medialis, +0.72).
 *
 *   IMU_thigh_TILT      [제외] 모드별 평균이 전부 -85~-87 deg 로 사실상 동일
 *                       (DC 기여 -2%)하고, 판별력이 오직 heel-strike/toe-off
 *                       충격 스파이크 파형에서만 나온다. H10 의 leftThighAngle
 *                       은 CM 이 상보/칼만 융합한 매끈한 값이라 그 스파이크가
 *                       없다 => 템플릿 자체가 불일치. 넣으면 오히려 해롭다.
 *   IMU_trunk_GYROMAG   [제외] d_subj 0.92 로 약하고, 유일한 강점이던 TURN 이
 *                       뱅크에서 빠졌다.
 *   IMU_shank / foot    [제외] 무릎/발목 엔코더가 없어 복원 불가.
 *                       단, 대체 손실은 15개 모드쌍 평균 1.7% 에 불과하다
 *                       (12쌍은 손실 0%, 최대 11.2%). 즉 이 하드웨어 제약은
 *                       실질적으로 문제가 되지 않는다.
 *
 * ============================================================================
 * 3. 런타임 도메인 보정 — 기준곡선을 H10 신호에 맞추는 방법
 * ============================================================================
 * 기준곡선은 Camargo 센서 단위/좌표계로 만들어졌고 H10 과 물리적으로 다르다:
 *   - Camargo gyro = rad/s, H10 = deg/s
 *   - Camargo TILT = atan2(AccX,AccZ) [deg], H10 bilateral sagittal pitch
 *     proxy와 센서 위치/offset/sign이 다름
 *   - Camargo EMG = mV 스케일, 우리 외부 ADC 전압과 무관
 *   - 기준곡선과 런타임 theta 원점은 모두 우측 heel-strike
 * 이 값을 그대로 z-score 에 넣으면 전부 쓰레기가 된다.
 *
 * 해결: BTN1 캘리브레이션(평지 보행 CALIB_STRIDE_COUNT stride)에서 우리 자신의
 * WALK 위상곡선을 만든 뒤, 기준곡선의 WALK 곡선에 맞추는 계수를 자동 추정한다.
 *   1) 잔여 위상 offset : 우리 곡선을 원형 시프트하며 r^WALK 과의 상관이
 *                    최대가 되는 시프트량을 찾는다. 같은 heel-strike 원점을
 *                    쓰고도 남는 센서 지연/접촉 검출 지연만 흡수한다.
 *   2) 부호 sign   : IMU는 상관계수 부호, RMS는 물리적으로 음수가 될 수 없어 +1
 *   3) 스케일 a    : std(r^WALK) / std(우리 WALK)  (단위 차이 흡수)
 *   4) 오프셋 b    : mean(r^WALK) - sign*a*mean(우리 WALK)
 * 이후 x_cal = sign*a*x + b 로 변환한 값을 z-score 에 넣는다.
 * => 캘리브레이션 전에는 분류하지 않는다(LED2 로 표시).
 *
 * ============================================================================
 * 4. 위상 theta 경계 — 왜 right footContact인가
 * ============================================================================
 * 기준곡선은 Camargo gcRight(오른쪽 heel-strike) 기준 1 stride다. 구버전은
 * rightThighAngle의 MHE를 경계로 쓰고 평지 캘리브레이션의 원형 shift로 두
 * 사건의 차이를 흡수했다. 그러나 MHE↔heel-strike 간격은 지형/보폭마다 달라
 * 평지에서 얻은 하나의 shift를 계단·경사로에 재사용하면 위상축이 흔들린다.
 * 따라서 H10의 isRightFootContact rising edge를 같은 물리 경계로 사용한다.
 * Body Data 8개 실측값이 없으면 footContact가 유효하지 않을 수 있으므로,
 * 상단 LMC_BODY_*가 0이면 표준체형을 임의 전송하지 않고 fail-closed 한다.
 *
 * ============================================================================
 * 5. 위상 리샘플링 — 왜 "직전 stride 길이로 정규화"를 버렸는가
 * ============================================================================
 * 최초 버전은 theta(t) = (t - t_boundary) / stride_period_직전 으로 매 ms 마다
 * causal 하게 위상을 추정하며 그 즉시 101 bin 에 누적했다. stride 길이가
 * 안정적인 정상상태 WALK 안에서는 그럭저럭 버티지만, 두 가지 구조적 결함이
 * 있었다:
 *   1) 모드 전환 stride 에서 가장 심하게 틀어진다. 예를 들어 WALK(~1.1s)에서
 *      RAMP_UP(~1.2~1.4s)으로 들어가는 stride 는 WALK 길이로 나눠지므로,
 *      실제 theta=0.7 지점인데 추정치는 이미 0.8~0.9 로 앞서간다. 정확히
 *      "잘 분류해야 하는" 전환 stride 에서 위상축이 제일 많이 왜곡되는
 *      구조였다.
 *   2) stride 가 길어지면(추정 대비) theta 가 1.0 에 먼저 도달해 버려서,
 *      진짜 heel-strike가 오기 전까지 남은 후반부 구간을 통째로 유실했다.
 *      몰려있는 theta=0.6~0.9 구간(thigh_GYROMAG 2차 피크)이 하필 후반부라
 *      타격이 컸다.
 *
 * 대안으로 "진짜 위상변수"(HZD 식 phase portrait, 예: thigh angle-velocity
 * 궤적의 atan2)도 검토했으나 채택하지 않았다. 기준곡선 자체가 Camargo 의
 * %GC(percent gait cycle, gcRight heel-strike 기준 시간정규화)로 만들어졌기
 * 때문이다. phase-portrait 각도를 %GC 인덱스에 그대로 대입하면, IMU_thigh_TILT
 * 를 폐기했던 것과 같은 종류의 오류(다른 좌표계로 만든 템플릿에 다른 좌표계
 * 신호를 억지로 대입)가 재발한다. 진짜 위상변수를 쓰려면 기준곡선 자체를
 * phase-portrait 공간에서 다시 만들어야 하는데, 이는 이번 스코프를 벗어나는
 * 훨씬 큰 재구축이다(모드별 궤적이 단조 폐곡선을 이루는지도 새로 검증해야
 * 한다 — STAND 는 애초에 폐곡선이 없다는 사실이 위 STAND 게이트 결정을
 * 뒷받침하기도 한다).
 *
 * 채택한 해법: causal 추정을 완전히 버리고, stride 가 끝나 실측 길이를 아는
 * 시점(=heel-strike)에 사후적으로 101 bin 에 리샘플링한다.
 *   - stride 진행 중에는 위상을 전혀 추정하지 않고, RAW_SAMPLE_PERIOD_MS(10ms)
 *     간격 창평균으로 원시 버퍼(s_raw_buf)에 그냥 쌓는다(_AccumulateRawSample).
 *   - heel-strike로 실측 stride_ms를 알게 된 순간, theta_i=i/100에 대응하는
 *     실제 시각 t_i = theta_i * stride_ms 를 원시 버퍼에서 선형보간한다
 *     (_ResamplePhaseCurve). 이는 build_h_d_reference_curves.m 이 오프라인
 *     에서 쓴 interp1(thetaImuU, ..., thetaGrid) 와 정확히 같은 연산이며,
 *     기준곡선을 만든 방식 그대로를 온라인에서 재현하는 것이다.
 *   - 레이턴시는 늘지 않는다. _ClassifyStride(stride_ms)는 heel-strike 시점에만
 *     호출되므로(1-stride 지연 구조), causal 추정을 사후 보간으로 바꿔도
 *     분류가 일어나는 타이밍 자체는 그대로다.
 * 메모리: RAW_BUF_LEN(300) x RC_FEATURE_COUNT(4) x 4 B = 4.8 KB (RAM, 원시
 * 버퍼 1개 인스턴스 — 진행 중인 stride 는 항상 하나뿐이므로 이거면 충분).
 *
 * ============================================================================
 * 6. 버튼 / LED
 * ============================================================================
 *   BTN1 클릭: 캘리브레이션 시작/재시작 (평지에서 걸으며 누를 것)
 *   BTN2 클릭: 투표 버퍼 + 모드 리셋
 *   BTN3 클릭: 판정 민감도 3단계 순환 (0.7 / 1.0 / 1.4배)
 *   LED1: ACTIVE heartbeat
 *   LED2: 캘리브레이션 진행 중 점멸 / 완료 시 점등
 *   LED3: 정지=소등, 평지보행=점등, 그 외 지형=점멸
 *
 * @note  ref_curve_tables.h 는 export_ref_curve_tables.m 이 자동 생성한다.
 *        재생성 시 Camargo_matlab 폴더에서 MATLAB 로 실행할 것.
 * @note  "Rc"(Reference Curve) 명명 — 이전 버전은 HZD(Hybrid Zero Dynamics)
 *        제어이론의 virtual constraint desired-output 표기 "h_d"를 빌려 썼으나,
 *        이 표는 그런 제어 대상이 아니라 순수 통계적 템플릿 매칭(z-score
 *        최근접 분류)에 쓰인다. virtual constraint 도, 이를 추종하는 토크
 *        제어 루프도 없다 — 그래서 이름을 정정했다(ref_curve_tables.h 참고).
 * @see   Qian, Y. et al. (2022) "Predictive Locomotion Mode Recognition and
 *        Accurate Gait Phase Estimation for Hip Exoskeleton on Various
 *        Terrains," IEEE RA-L 7(3):6439-6446. (majority filter 배경)
 * @see   Camargo, J. et al. (2021) "A comprehensive, open-source dataset of
 *        lower limb biomechanics in multiple conditions," J. Biomech. 119.
 * @version 1.2
 * @date    Aug 25, 2026
 ******************************************************************************
 */

#include "xm_api.h"
#include "ref_curve_tables.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/* --- EMG 근육 자리 수 --------------------------------------------------
 * 자리를 늘릴 때 이 상수 하나만 바꾸면 된다(배열/루프는 전부 이 값 기준).
 * 1자리당 좌우 2채널(L/R). 외부 ADC 는 PF3~PF6 = XM_EXT_ADC_5~8 로 4채널이므로
 * 최대 2자리까지 외부 배선 가능. 3자리는 H10 내장 EMG 를 추가로 동원해야 한다.
 *
 * 자리 선택은 Camargo 7명 데이터의 greedy 최적화 결과를 따랐다:
 *   site0 = gastrocnemius medialis (15쌍 합계 gain +5.44, 압도적 1위)
 *   site1 = vastus medialis        (+0.72, 2위)
 * (참고: soleus 는 총 gain 0.83 으로 gastrocnemius 보다 훨씬 약했다.)
 *
 * MUSCLE_SITE_COUNT 를 1로 줄이면 EMG_SITE1 feature 가 자동 비활성화되고
 * 나머지 3 feature 로 동작한다. 코드 구조 변경 불필요.
 * --------------------------------------------------------------------- */
#define MUSCLE_SITE_COUNT       2U
#define EMG_SIDE_COUNT          2U      /* R, L */
#define EMG_CH_COUNT            (MUSCLE_SITE_COUNT * EMG_SIDE_COUNT)
#define EMG_SIDE_R              0U
#define EMG_SIDE_L              1U
#define EMG_CH_INDEX(site, side)  (((site) * EMG_SIDE_COUNT) + (side))

/* EMG RMS 이동윈도우 — Camargo 기준곡선 생성 시 ENV_WINDOW_S=0.15 와 동일 */
#define EMG_RMS_WINDOW_MS       150U
#define EMG_BIAS_SETTLE_MS      1000U   /* ACTIVE 진입 후 DC bias 추정 구간 */

/* 기준표의 EMG는 MATLAB movmean 기본값인 중심 150 ms 창으로 생성됐다.
 * H10은 미래 샘플을 읽을 수 없으므로 과거 150 ms RMS를 유지하고, 분류 시
 * 중심창의 군지연(약 75 ms)을 기준곡선 theta 인덱스에서 보상한다. */
#if !defined(RC_EMG_REFERENCE_CENTERED) || !defined(RC_EMG_RMS_WINDOW_MS)
#error "ref_curve_tables.h에 EMG 전처리 계약이 없습니다. 생성기를 다시 실행하세요."
#endif
#if (RC_EMG_REFERENCE_CENTERED != 1U) || (RC_EMG_RMS_WINDOW_MS != EMG_RMS_WINDOW_MS)
#error "EMG 기준곡선과 H10 런타임 RMS 계약이 일치하지 않습니다."
#endif

/* --- 정지(STAND) 게이트 --- */
#define MIN_FWD_VELOCITY        0.1f    /* m/s — 이하이고 각속도도 낮으면 정지 */
#define STAND_GYRO_TH_DPS       30.0f   /* deg/s — 고관절 IMU 각속도 크기 임계 */

/* --- H10 Body Data ----------------------------------------------------
 * footContact/forwardVelocity는 이 8개 실측치를 H10에 보내야 유효해진다.
 * 0 기본값은 "미설정"이며, 이 상태에서는 분류기가 STAND/hold-last로 닫힌다.
 * 공식 예제의 표준체형 숫자를 실제 사용자 값인 것처럼 넣지 않는다. */
#ifndef LMC_BODY_WEIGHT_G
#define LMC_BODY_WEIGHT_G          0U
#endif
#ifndef LMC_BODY_HEIGHT_MM
#define LMC_BODY_HEIGHT_MM         0U
#endif
#ifndef LMC_BODY_THIGH_R_MM
#define LMC_BODY_THIGH_R_MM        0U
#endif
#ifndef LMC_BODY_THIGH_L_MM
#define LMC_BODY_THIGH_L_MM        0U
#endif
#ifndef LMC_BODY_SHANK_R_MM
#define LMC_BODY_SHANK_R_MM        0U
#endif
#ifndef LMC_BODY_SHANK_L_MM
#define LMC_BODY_SHANK_L_MM        0U
#endif
#ifndef LMC_BODY_ANKLE_R_MM
#define LMC_BODY_ANKLE_R_MM        0U
#endif
#ifndef LMC_BODY_ANKLE_L_MM
#define LMC_BODY_ANKLE_L_MM        0U
#endif

/* --- TURN 검출기 --- */
#define TURN_WINDOW_MS          800U    /* heading 변화를 볼 시간창 */
#define TURN_SAMPLE_PERIOD_MS   10U     /* yaw 링버퍼 샘플 주기 */
#define TURN_YAW_BUF_LEN        (TURN_WINDOW_MS / TURN_SAMPLE_PERIOD_MS)  /* 80 */
#define TURN_ENTER_YAW_DEG      30.0f   /* 이 이상 heading 변화 -> TURN 진입 */
#define TURN_EXIT_YAW_DEG       15.0f   /* 이 이하로 떨어져야 TURN 해제(히스테리시스) */

/* --- Stride 유효성 --- */
#define STRIDE_MIN_MS           500U
#define STRIDE_MAX_MS           3000U

/* --- 원시 버퍼 (사후 위상 리샘플링용) ---
 * stride 진행 중에는 "직전 stride 길이"로 위상을 causal 하게 추정하지 않는다.
 * 대신 시간축 그대로(10ms 간격 창평균)로 원시 버퍼에 쌓아두고, 우측
 * footContact rising edge로 stride가
 * 끝나 실측 길이(stride_ms)를 아는 시점에 그 실측값으로 101 위상 bin 에
 * 리샘플링한다(build_h_d_reference_curves.m 의 interp1 과 동일 연산). 이렇게
 * 하면 stride 길이를 추정할 필요가 아예 없어져서, 모드 전환처럼 stride-to-stride
 * 길이가 크게 바뀌는 구간에서도 위상축이 왜곡되지 않는다. 분류는 heel-strike
 * 시점(stride 완료 후)에만 일어나므로 레이턴시는 늘지 않는다. */
#define RAW_SAMPLE_PERIOD_MS     10U     /* 원시 버퍼 창 길이 */
#define RAW_BUF_LEN              (STRIDE_MAX_MS / RAW_SAMPLE_PERIOD_MS)  /* 300 */
#define RAW_MIN_SAMPLES          10U     /* 리샘플링 최소 원시 샘플 수(방어) */

/* --- 캘리브레이션 --- */
#define CALIB_STRIDE_COUNT      8U      /* 평지 보행 baseline 수집 stride 수 */

/* --- 분류 히스테리시스 (margin score: 0~1) ---
 * 3-class 였던 Terrain_Intent_Classifier.c 는 양방향 dual-threshold 였지만,
 * 여기는 5-way argmin 이라 "1위와 2위의 상대 마진"이 곧 확신도다.
 * 마진이 이 값에 못 미치는 stride 는 투표 자체를 하지 않아(완충지대)
 * 직전 다수결 상태가 유지된다 — 경계 지형에서의 flicker 방지.
 *
 * 0.35 -> 0.30 인하 근거 (validate_12_threshold_sweep.py)
 * ------------------------------------------------------
 * 0.35 에서는 완성 안정 stride 의 절반가량이 이 게이트에서 걸러져
 * (steady_event_vote_fraction 47~53%) 3/2 버퍼에 후보가 도달하지 못했다.
 * 즉 전환 억제의 병목은 다수결이 아니라 그 앞단 게이트였다.
 *
 * clean8 을 fit(AB06,07,08,09) / holdout(AB10,12,13,14) 으로 나눠 스윕한 결과:
 *      fit      0.30/0.15 : commit +2.74%p, wrong_mode +1.28%p (교환비 0.47)
 *      holdout  0.30/0.15 : commit +2.67%p, wrong_mode +2.49%p (교환비 0.93)
 * commit 이득은 재현되지만 wrong_mode 비용은 holdout 에서 약 2배로 커진다.
 * => fit 에서 보였던 "2배 남는 거래"는 그 4명의 특성이고, 처음 보는 사람에게는
 *    사실상 1:1 등가 교환이다. 공짜 이득이 아니므로 여기서 더 낮추지 않는다.
 *
 * 더 낮출 때의 실측 곡선(fit 기준, 교환비는 낮을수록 유리):
 *      0.25/0.12 -> 0.75 | 0.20/0.10 -> 0.91 | 0.15/0.08 -> 1.07(손익분기 돌파)
 *      0.00/0.00 (게이트 개방) 은 commit 이 오히려 정체하고 wrong_mode 만 급증한다.
 * 주의: 전환 실패는 "직전 지형 보조 유지"(보수적)지만 wrong_mode 는 "어느
 * 지형과도 안 맞는 제3 보조"라 두 오류의 실기 위험도가 대칭이 아니다. */
#define MODE_CONFIRM_TH         0.30f
/* IDLE 전환 증거가 있고 후보가 현재 모드와 다를 때만 쓰는 낮은 문턱.
 * 3/2 다수결은 그대로라서 같은 후보가 최소 두 번은 관측되어야 전환된다.
 * 위 스윕에서 이 값은 0.15 를 유지하는 쪽이 교환비가 좋아 그대로 둔다. */
#define MODE_TRANSITION_CONFIRM_TH  0.15f

/* --- 다수결 투표 ---
 * 5-way 경쟁(뱅크 5모드) 대비 window/majority 를 완화했다. 최초값(5/3, 3표
 * 필요)은 2-class(LEVEL/INCLINE) 뿐이던 Terrain_Intent_Classifier.c 에서
 * 재튜닝 없이 그대로 옮긴 값이었는데, 이 경우 확신 있는 stride 가 매번
 * 이어져도 최소 3 stride(~3~4초) 지연되고, 전환 초반 stride 가 margin 미달
 * (완충지대)이나 구 IDLE hard veto에 걸려 투표를 건너뛰면 5~10초까지도 늘어났다.
 * 3/2 로 낮추면 확신 있는 stride 2개 연속(~2~2.5초)이면 전환되고, 경계
 * 지형에서의 flicker 방지는 안정/전환 이중 margin과 이 다수결이 맡는다. */
#define VOTE_WINDOW             3U
#define VOTE_MAJORITY           2U

/* --- 민감도 프리셋 (BTN3) --- */
#define NUM_SENSITIVITY_LEVELS  3U

/* --- feature 가중치 (뱅크 z 계산 시) ---
 * IMU 는 항상 신뢰 가능하므로 1.0. EMG 는 전극 부착 상태/피로에 민감하므로
 * 약간 낮춘다. 자리를 늘리면 여기에 항목을 추가하면 된다. */
#define W_TRUNK_TILT            1.0f
#define W_THIGH_GYROMAG         1.0f
#define W_EMG                   0.8f

/* --- 타이밍 --- */
#define USB_DEBUG_PERIOD_MS     300U
#define USB_STREAM_ID           0xF4

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/** @brief 최종 출력 7-class (기준곡선 뱅크의 5모드 + STAND + TURN) */
typedef enum {
    LOCO_STAND = 0,
    LOCO_WALK,
    LOCO_TURN,
    LOCO_STAIRS_UP,
    LOCO_STAIRS_DOWN,
    LOCO_RAMP_UP,
    LOCO_RAMP_DOWN,
    LOCO_MODE_COUNT
} LocoMode_t;

/** @brief feature 별 런타임 도메인 보정 계수 (3장 참고) */
typedef struct {
    float sign;     /* +1 또는 -1 — 좌표계 반전 흡수 */
    float scale;    /* std(r^WALK) / std(측정 WALK) */
    float offset;   /* mean(r^WALK) - sign*scale*mean(측정 WALK) */
    bool  enabled;  /* MUSCLE_SITE_COUNT 에 따라 EMG feature 자동 비활성화 */
} FeatureCalib_t;

/** @brief USB 스트리밍 페이로드 */
typedef struct {
    float mode;         /* LocoMode_t 를 float 로 */
    float z_best;       /* 뱅크 최우수 모드의 z */
    float z_second;     /* 2위 모드의 z */
    float margin;       /* tanh 압축된 판정 마진 (0~1) */
    float theta;        /* 현재 위상 (0~1) */
    float yaw_delta;    /* TURN 검출용 heading 변화량 (deg) */
    float emg_asym0;    /* site0 좌우 비대칭 (R-L)/(R+L) — 진단용 */
    float calib_done;   /* 0/1 */
    float z_idle;       /* IDLE 평탄 템플릿 대비 z (작을수록 전이 상태) */
    float in_transition;/* 0/1 — 직전 stride 에 IDLE 전환 증거가 있었는가 */
} LocoStreamData_t;

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

static XmTsmHandle_t s_tsm;

/* --- 민감도 --- */
static const float k_sensitivity_scale[NUM_SENSITIVITY_LEVELS] = { 0.7f, 1.0f, 1.4f };
static uint8_t     s_sensitivity_idx = 1U;

/* --- feature 가중치 (인덱스는 RcFeature_t 순서) --- */

static const float k_feature_weight[RC_FEATURE_COUNT] = {
    W_TRUNK_TILT,       /* RC_FEAT_TRUNK_TILT */
    W_THIGH_GYROMAG,    /* RC_FEAT_THIGH_GYROMAG */
    W_EMG,              /* RC_FEAT_EMG_SITE0_RMS */
    W_EMG               /* RC_FEAT_EMG_SITE1_RMS */
};

/* --- EMG 파이프라인 --- */
static const XmAdcPin_t s_emg_pins[EMG_CH_COUNT] = {
    XM_EXT_ADC_5,   /* PF3 : site0 R */
    XM_EXT_ADC_6,   /* PF4 : site0 L */
#if (MUSCLE_SITE_COUNT >= 2U)
    XM_EXT_ADC_7,   /* PF5 : site1 R */
    XM_EXT_ADC_8,   /* PF6 : site1 L */
#endif
};
static float    s_emg_bias_v[EMG_CH_COUNT];
static float    s_emg_sq_buf[EMG_CH_COUNT][EMG_RMS_WINDOW_MS];
static float    s_emg_sq_sum[EMG_CH_COUNT];
static uint16_t s_emg_buf_idx = 0U;
static bool     s_emg_buf_filled = false;
static float    s_emg_rms_v[EMG_CH_COUNT];
static bool     s_emg_bias_ready = false;
static float    s_emg_bias_sum[EMG_CH_COUNT];
static uint32_t s_emg_bias_cnt = 0U;

/* --- 우측 footContact heel-strike 경계 --- */
static bool     s_body_data_configured = false;
static bool     s_prev_contact_r = false;
static bool     s_contact_r_armed = false;  /* 한 번 false를 본 뒤 rising edge만 허용 */
static uint32_t s_last_hs_tick = 0U;
static uint32_t s_stride_period_ms = 1000U;  /* 직전 stride 실측 길이 — 표시용 theta 분모.
                                               * 분류용 위상 리샘플링에는 쓰이지 않는다
                                               * (그 stride 의 실측 길이를 그대로 쓰므로). */
static bool     s_has_first_hs = false;

/* --- 원시 버퍼 (진행 중인 stride, 10ms 창평균, 사후 리샘플링 대기) ---
 * 위상을 추정하지 않고 시간축 그대로 쌓는다 — RAW_SAMPLE_PERIOD_MS 참고. */
static float    s_raw_buf[RC_FEATURE_COUNT][RAW_BUF_LEN];
static float    s_raw_window_sum[RC_FEATURE_COUNT];
static uint16_t s_raw_window_cnt = 0U;
static uint16_t s_raw_count = 0U;            /* 확정된 원시 버퍼 슬롯 수 */
static uint32_t s_raw_window_start_tick = 0U;

/* --- 완료된 stride 를 101 위상 bin 으로 리샘플링한 결과 ---
 * _ResamplePhaseCurve() 가 heel-strike 시점에 한 번에 채운다(사후 처리). 각 bin 은
 * 보간값 1개씩만 가지므로 s_obs_cnt 는 항상 0 또는 1 이다 — 다운스트림
 * (_ClassifyStride/_AccumulateCalibStride/_ComputeIdleZ)은 원래도 sum/cnt
 * 형태로 읽으므로 수정 없이 그대로 재사용된다. */
static float    s_obs_sum[RC_FEATURE_COUNT][RC_THETA_POINTS];
static uint16_t s_obs_cnt[RC_THETA_POINTS];

/* --- 캘리브레이션 --- */
static bool     s_calibrating = false;
static uint8_t  s_calib_stride_idx = 0U;
static float    s_calib_sum[RC_FEATURE_COUNT][RC_THETA_POINTS];
static uint16_t s_calib_cnt[RC_THETA_POINTS];
static FeatureCalib_t s_calib[RC_FEATURE_COUNT];
static int      s_phase_shift = 0;      /* 우리 theta -> 기준곡선 theta 원형 시프트량 */
static bool     s_calib_done = false;

/* --- TURN 검출 --- */
static float    s_yaw_buf[TURN_YAW_BUF_LEN];
static uint16_t s_yaw_idx = 0U;
static bool     s_yaw_filled = false;
static uint32_t s_yaw_last_tick = 0U;
static float    s_yaw_delta_deg = 0.0f;
static bool     s_turn_active = false;

/* --- 다수결 투표 --- */
static uint8_t  s_vote_buf[VOTE_WINDOW];
static uint8_t  s_vote_idx = 0U;
static uint8_t  s_vote_filled = 0U;
static RcMode_t s_bank_mode = RC_MODE_WALK;   /* 뱅크가 고른 보행 모드 */

/* --- 최종 결과 / 진단 --- */
static LocoMode_t s_current_mode = LOCO_STAND;
static float      s_last_z_best = 0.0f;
static float      s_last_z_second = 0.0f;
static float      s_last_z_idle = 0.0f;
static bool       s_in_transition = false;   /* 직전 stride 에 IDLE 전환 증거가 있음 */
static float      s_last_margin = 0.0f;
static float      s_theta = 0.0f;

/* --- USB --- */
static uint32_t         s_usb_debug_timer = 0U;
static LocoStreamData_t s_stream;

/* 7-class 이름 (디버그 출력용) */
static const char *const k_loco_name[LOCO_MODE_COUNT] = {
    "STAND", "WALK", "TURN", "STAIRS_UP", "STAIRS_DOWN", "RAMP_UP", "RAMP_DOWN"
};

/* 뱅크 모드(RcMode_t) -> 최종 7-class(LocoMode_t) 매핑 */
static const LocoMode_t k_bank_to_loco[RC_MODE_COUNT] = {
    LOCO_WALK,          /* RC_MODE_WALK */
    LOCO_STAIRS_UP,     /* RC_MODE_STAIRS_UP */
    LOCO_STAIRS_DOWN,   /* RC_MODE_STAIRS_DOWN */
    LOCO_RAMP_UP,       /* RC_MODE_RAMP_UP */
    LOCO_RAMP_DOWN      /* RC_MODE_RAMP_DOWN */
};

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

static void  _UpdateEmg(void);
static float _HipGyroMagDps(void);
static float _SampleFeature(uint8_t feat_idx);
static bool  _SetupBodyData(void);

static void  _ResetStrideAccumulator(uint32_t tick);
static void  _AccumulateRawSample(uint32_t tick);
static void  _ResamplePhaseCurve(uint32_t stride_ms);
static uint8_t _ReferenceThetaIndex(uint8_t feat_idx, uint8_t theta_idx,
                                    uint32_t stride_ms);

static void  _UpdateTurnDetector(uint32_t tick);
static float _WrapDeg180(float deg);

static void  _StartCalibration(void);
static void  _AccumulateCalibStride(void);
static void  _FinishCalibration(void);
static bool  _CurveStats(const float *curve, float *out_mean, float *out_std);

static void  _ClassifyStride(uint32_t stride_ms);
static float _ComputeIdleZ(void);
static void  _PushVote(uint8_t bank_mode);
static void  _UpdateModeFromVotes(void);

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
    s_body_data_configured = _SetupBodyData();

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

    XM_SetUsbCustomMeta(USB_STREAM_ID,
        "[{\"name\":\"Mode\",\"unit\":\"-\"},"
        "{\"name\":\"z Best\",\"unit\":\"-\"},"
        "{\"name\":\"z 2nd\",\"unit\":\"-\"},"
        "{\"name\":\"Margin\",\"unit\":\"-\"},"
        "{\"name\":\"Theta\",\"unit\":\"-\"},"
        "{\"name\":\"Yaw Delta\",\"unit\":\"deg\"},"
        "{\"name\":\"EMG Asym0\",\"unit\":\"-\"},"
        "{\"name\":\"Calib\",\"unit\":\"-\"},"
        "{\"name\":\"z Idle\",\"unit\":\"-\"},"
        "{\"name\":\"In Transition\",\"unit\":\"-\"}]");

    /* 외부 ADC 4채널(PF3~PF6) 활성화 — EMG 전용.
     * 이 핀들은 FSR 과 물리적으로 공유되지만 본 알고리즘은 FSR 을 쓰지 않는다. */
    XM_SwitchDioToAdc(XM_EXT_DIO_1);
    XM_SwitchDioToAdc(XM_EXT_DIO_2);
    XM_SwitchDioToAdc(XM_EXT_DIO_3);
    XM_SwitchDioToAdc(XM_EXT_DIO_4);

    /* 토크를 인가하지 않는 순수 분류기 */
    XM_SetControlMode(XM_CTRL_MONITOR);
}

void User_Loop(void)
{
    /* TSM 핸들 생성 실패 시 안전 정지 (NULL 역참조 HardFault 방지) */
    if (!s_tsm) {
        return;
    }

    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF);
    }

    XM_TSM_Run(s_tsm);

    /* [필수] 버튼 디바운싱 + LED 효과 타이머 틱.
     * 호출하지 않으면 XM_GetButtonEvent() 가 항상 NONE 이 되어 BTN1~3 이 죽는다. */
    XM_IO_Update();
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
        XM_SendUsbDebugMessage("[LMC] CM 연결됨 -> STANDBY\r\n");
        XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
    }
}

static void Standby_Loop(void)
{
    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_SendUsbDebugMessage("[LMC] ASSIST 감지 -> ACTIVE\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_ACTIVE);
    }
}

static void Active_Entry(void)
{
    uint32_t tick = XM_GetTick();

    /* --- EMG 파이프라인 초기화 (bias 재추정) --- */
    memset(s_emg_sq_buf, 0, sizeof(s_emg_sq_buf));
    memset(s_emg_sq_sum, 0, sizeof(s_emg_sq_sum));
    memset(s_emg_rms_v,  0, sizeof(s_emg_rms_v));
    memset(s_emg_bias_sum, 0, sizeof(s_emg_bias_sum));
    s_emg_buf_idx = 0U;
    s_emg_buf_filled = false;
    s_emg_bias_ready = false;
    s_emg_bias_cnt = 0U;

    /* --- right footContact / 위상 리샘플링 초기화 --- */
    s_prev_contact_r = XM.status.h10.isRightFootContact;
    s_contact_r_armed = !s_prev_contact_r;
    s_last_hs_tick = tick;
    s_stride_period_ms = 1000U;
    s_has_first_hs = false;
    _ResetStrideAccumulator(tick);

    /* --- TURN 검출기 초기화 --- */
    s_yaw_idx = 0U;
    s_yaw_filled = false;
    s_yaw_last_tick = tick;
    s_yaw_delta_deg = 0.0f;
    s_turn_active = false;

    /* --- 투표 초기화 (캘리브레이션 결과 s_calib 는 유지) --- */
    s_vote_idx = 0U;
    s_vote_filled = 0U;
    s_calibrating = false;
    s_calib_stride_idx = 0U;
    s_current_mode = LOCO_STAND;
    s_bank_mode = RC_MODE_WALK;
    s_last_z_idle = 0.0f;
    s_in_transition = false;

    s_usb_debug_timer = tick;

    _UpdateLedIndicators();

    XM_SendUsbDebugMessage("[LMC] ACTIVE — right footContact 경계 7모드 분류 시작\r\n");
    if (!s_body_data_configured) {
        XM_SendUsbDebugMessage(
            "[LMC] BODY DATA 미설정: footContact 분류 비활성(상단 LMC_BODY_* 입력 필요)\r\n");
    }
    if (!s_calib_done) {
        XM_SendUsbDebugMessage("[LMC] 캘리브레이션 필요: 평지에서 걸으며 BTN1\r\n");
    }
}

/**
 * @brief ACTIVE 루프 (1ms) — 신호취득 -> 게이트 -> 위상 -> stride 마감 -> 분류
 */
static void Active_Loop(void)
{
    if (XM.status.h10.h10Mode != XM_H10_MODE_ASSIST) {
        XM_SendUsbDebugMessage("[LMC] ASSIST 해제 -> STANDBY\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
        return;
    }

    _HandleButtonInput();

    uint32_t tick = XM_GetTick();

    _UpdateEmg();
    _UpdateTurnDetector(tick);

    float fwd_vel  = XM.status.h10.forwardVelocity;
    float gyro_mag = _HipGyroMagDps();

    /* ---------- 층 1: STAND 게이트 (기준곡선 미사용) ----------
     * 속도가 낮고 고관절 각속도도 작으면 즉시 정지로 판정하고 stride 누적을
     * 중단한다. Camargo 의 idle 라벨이 오염되어 r^STAND 를 만들 수 없으므로
     * 이 물리 게이트가 STAND 판정의 유일한 경로다. */
    if ((fwd_vel < MIN_FWD_VELOCITY) && (gyro_mag < STAND_GYRO_TH_DPS)) {
        s_current_mode = LOCO_STAND;
        s_prev_contact_r = XM.status.h10.isRightFootContact;
        if (!s_prev_contact_r) { s_contact_r_armed = true; }
        s_has_first_hs = false;
        _ResetStrideAccumulator(tick);
        _UpdateLedIndicators();
        _UpdateUsbDebug();
        _UpdateStreamData();
        return;
    }

    /* ---------- 표시용 theta (분류에는 미사용) ----------
     * USB 스트림/디버그 로그에 "지금 대략 어디쯤"을 보여주는 용도일 뿐이다.
     * 직전 stride 길이로 추정하므로 전환 구간에서는 부정확할 수 있으나,
     * 분류 자체는 아래에서 stride 완료 후 실측 길이로 사후 리샘플링하므로
     * 이 추정 오차가 결과에 영향을 주지 않는다. */
    uint32_t elapsed = tick - s_last_hs_tick;
    float theta = (float)elapsed / (float)s_stride_period_ms;
    if (theta < 0.0f) { theta = 0.0f; }
    if (theta > 1.0f) { theta = 1.0f; }
    s_theta = theta;

    /* ---------- 원시 버퍼 누적 (위상 추정과 무관, 시간축 그대로) ---------- */
    if (s_has_first_hs) {
        _AccumulateRawSample(tick);
    }

    /* ---------- stride 마감 (우측 footContact rising edge) ----------
     * ref_curve_tables.h는 gcRight heel-strike가 theta=0인 표다. 평지에서
     * 추정한 MHE↔heel-strike 오프셋을 모든 지형에 재사용하지 않고, H10이
     * 제공하는 같은 물리 사건으로 경계를 직접 맞춘다. */
    bool contact_r = XM.status.h10.isRightFootContact;
    if (!contact_r) { s_contact_r_armed = true; }
    bool heel_strike_r = s_body_data_configured
                         && s_contact_r_armed
                         && contact_r
                         && !s_prev_contact_r;
    s_prev_contact_r = contact_r;

    if (heel_strike_r) {
        s_contact_r_armed = false;
        uint32_t stride_ms = tick - s_last_hs_tick;

        /* STRIDE_MIN_MS보다 짧은 rising edge는 chatter로 보고 기존 stride를
         * 계속 누적한다. 너무 긴 간격은 경계 유실로 보고 현재 edge에서
         * 재동기화하지만 분류에는 사용하지 않는다. */
        if (!s_has_first_hs) {
            s_has_first_hs = true;
            s_last_hs_tick = tick;
            _ResetStrideAccumulator(tick);
        } else if (stride_ms >= STRIDE_MIN_MS) {
            bool valid = (stride_ms <= STRIDE_MAX_MS)
                         && (s_raw_count >= RAW_MIN_SAMPLES);

            /* stride가 유효하면 실측 stride_ms로 101 위상 bin에 사후
             * 리샘플링한다. 미래값이나 정답 전환 시각은 쓰지 않는다. */
            if (valid) {
                _ResamplePhaseCurve(stride_ms);
            }

            /* EMG 파이프라인 준비 전에는 0 feature가 z를 왜곡하므로 대기. */
            bool emg_ready = (s_emg_bias_ready && s_emg_buf_filled);

            if (valid && emg_ready) {
                if (s_calibrating) {
                    _AccumulateCalibStride();
                } else if (s_calib_done) {
                    _ClassifyStride(stride_ms);
                }
            }
            if (valid) {
                s_stride_period_ms = stride_ms;
            }

            s_last_hs_tick = tick;
            _ResetStrideAccumulator(tick);
        }
    }

    /* ---------- 층 2: TURN 이 뱅크 결과를 override ----------
     * 회전 중에는 지형 판별이 의미가 없고(양발이 지형이 아니라 방향에 반응),
     * 기준곡선으로는 TURN 을 구분할 수 없으므로 yaw 검출기가 최종 판정을 가진다. */
    if (s_turn_active) {
        s_current_mode = LOCO_TURN;
    } else if (s_calib_done) {
        s_current_mode = k_bank_to_loco[s_bank_mode];
    } else {
        s_current_mode = LOCO_WALK;   /* 캘리브레이션 전에는 보행 여부만 표시 */
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
    XM_SendUsbDebugMessage("[LMC] 제어 종료\r\n");
}

// ==================== 신호 취득 ====================

/**
 * @brief H10 footContact/forwardVelocity를 위한 사용자 신체정보 전송
 * @return 여덟 실측값이 모두 설정되어 H10에 전송되었으면 true
 * @note 0 기본값에서는 잘못된 표준체형을 보내지 않고 분류를 fail-closed 한다.
 */
static bool _SetupBodyData(void)
{
#if (LMC_BODY_WEIGHT_G == 0U) || (LMC_BODY_HEIGHT_MM == 0U) \
 || (LMC_BODY_THIGH_R_MM == 0U) || (LMC_BODY_THIGH_L_MM == 0U) \
 || (LMC_BODY_SHANK_R_MM == 0U) || (LMC_BODY_SHANK_L_MM == 0U) \
 || (LMC_BODY_ANKLE_R_MM == 0U) || (LMC_BODY_ANKLE_L_MM == 0U)
    XM_SendUsbDebugMessage(
        "[LMC] LMC_BODY_* 8개 실측값이 없어 Body Data를 보내지 않음\r\n");
    return false;
#else
    const uint32_t body_data[8] = {
        LMC_BODY_WEIGHT_G,
        LMC_BODY_HEIGHT_MM,
        LMC_BODY_THIGH_R_MM,
        LMC_BODY_THIGH_L_MM,
        LMC_BODY_SHANK_R_MM,
        LMC_BODY_SHANK_L_MM,
        LMC_BODY_ANKLE_R_MM,
        LMC_BODY_ANKLE_L_MM,
    };
    XM_SendUserBodyData(body_data);
    XM_SendUsbDebugMessage("[LMC] H10 Body Data 전송 완료\r\n");
    return true;
#endif
}

/**
 * @brief 고관절 IMU 3축 자이로 크기 [deg/s]
 * @details 기준곡선의 IMU_thigh_GYROMAG 는 sqrt(Gx^2+Gy^2+Gz^2) 로 정의되어 있고,
 *          H10 CM 은 좌우 각각 GlobalGyr X/Y/Z 6축을 모두 PDO 로 제공한다
 *          (cm_drv.c 의 _CM_CreatePDO 등록 확인). Camargo 는 오른쪽 다리만
 *          계측했으므로 위상/표와 정합하도록 오른쪽 값을 쓴다.
 * @note    Camargo 원본 단위는 rad/s, H10 은 deg/s 다. 이 차이는 캘리브레이션의
 *          scale 계수가 흡수한다(3장 참고).
 */
static float _HipGyroMagDps(void)
{
    float gx = XM.status.h10.rightHipImuGlobalGyrX;
    float gy = XM.status.h10.rightHipImuGlobalGyrY;
    float gz = XM.status.h10.rightHipImuGlobalGyrZ;
    return sqrtf((gx * gx) + (gy * gy) + (gz * gz));
}

/**
 * @brief EMG 파이프라인 (1ms) — bias 제거 -> 제곱 이동합 -> RMS
 * @details Final_EMG.c 의 ADC 샘플링/bias 제거 관례를 재사용하되, 토크 변환
 *          대신 과거 150ms만 쓰는 causal RMS를 만든다. 기준표는 중심창이므로
 *          _ReferenceThetaIndex()에서 보폭별 약 75ms 군지연을 보상한다.
 */
static void _UpdateEmg(void)
{
    float centered[EMG_CH_COUNT];

    for (uint8_t i = 0U; i < EMG_CH_COUNT; i++) {
        float raw_v = (float)XM_AnalogReadMillivolts(s_emg_pins[i]) * 0.001f;

        /* ACTIVE 진입 직후 EMG_BIAS_SETTLE_MS 동안 DC bias 추정 */
        if (!s_emg_bias_ready) {
            s_emg_bias_sum[i] += raw_v;
            centered[i] = 0.0f;
        } else {
            centered[i] = raw_v - s_emg_bias_v[i];
        }
    }

    if (!s_emg_bias_ready) {
        s_emg_bias_cnt++;
        if (s_emg_bias_cnt >= EMG_BIAS_SETTLE_MS) {
            for (uint8_t i = 0U; i < EMG_CH_COUNT; i++) {
                s_emg_bias_v[i] = s_emg_bias_sum[i] / (float)s_emg_bias_cnt;
            }
            s_emg_bias_ready = true;
            XM_SendUsbDebugMessage("[LMC] EMG bias 추정 완료\r\n");
        }
        return;
    }

    /* 제곱값 순환버퍼 이동합 -> RMS */
    for (uint8_t i = 0U; i < EMG_CH_COUNT; i++) {
        float sq = centered[i] * centered[i];
        s_emg_sq_sum[i] -= s_emg_sq_buf[i][s_emg_buf_idx];
        s_emg_sq_buf[i][s_emg_buf_idx] = sq;
        s_emg_sq_sum[i] += sq;

        uint16_t n = s_emg_buf_filled ? EMG_RMS_WINDOW_MS : (s_emg_buf_idx + 1U);
        float mean_sq = s_emg_sq_sum[i] / (float)n;
        if (mean_sq < 0.0f) { mean_sq = 0.0f; }   /* 부동소수 누적오차 방어 */
        s_emg_rms_v[i] = sqrtf(mean_sq);
    }

    s_emg_buf_idx++;
    if (s_emg_buf_idx >= EMG_RMS_WINDOW_MS) {
        s_emg_buf_idx = 0U;
        s_emg_buf_filled = true;
    }
}

/**
 * @brief feature 인덱스 -> 현재 측정값 (H10 원단위 그대로)
 * @details 기준곡선은 Camargo 단위이므로 여기서는 변환하지 않는다.
 *          단위/부호/오프셋 정합은 캘리브레이션 계수가 담당한다(3장).
 */
static float _SampleFeature(uint8_t feat_idx)
{
    switch (feat_idx) {
        case RC_FEAT_TRUNK_TILT:
            /* Camargo TRUNK_TILT는 전후 시상면 기울기다. H10 pelvicAngle은
             * 공식 API상 좌우 골반 tilt이므로 같은 feature가 아니다.
             * 좌우 proximal IMU의 SagittalPitch 평균을 H10 가용 proxy로 쓴다. */
            return 0.5f * (XM.status.h10.leftHipImuSagittalPitch
                           + XM.status.h10.rightHipImuSagittalPitch);

        case RC_FEAT_THIGH_GYROMAG:
            return _HipGyroMagDps();

        case RC_FEAT_EMG_SITE0_RMS:
            /* Camargo 는 오른쪽 다리만 계측 -> R 채널로 표와 정합 */
            return s_emg_rms_v[EMG_CH_INDEX(0U, EMG_SIDE_R)];

        case RC_FEAT_EMG_SITE1_RMS:
#if (MUSCLE_SITE_COUNT >= 2U)
            return s_emg_rms_v[EMG_CH_INDEX(1U, EMG_SIDE_R)];
#else
            return 0.0f;   /* s_calib[].enabled == false 라 사용되지 않음 */
#endif

        default:
            return 0.0f;
    }
}

// ==================== heel-strike 위상 / stride 누적 ====================

static void _ResetStrideAccumulator(uint32_t tick)
{
    memset(s_obs_sum, 0, sizeof(s_obs_sum));
    memset(s_obs_cnt, 0, sizeof(s_obs_cnt));

    memset(s_raw_window_sum, 0, sizeof(s_raw_window_sum));
    s_raw_window_cnt = 0U;
    s_raw_count = 0U;
    s_raw_window_start_tick = tick;
}

/**
 * @brief 원시 버퍼 누적 (1ms 호출) — 위상 추정 없이 시간축 그대로 쌓는다
 * @details RAW_SAMPLE_PERIOD_MS(10ms) 창 안의 표본을 평균해 슬롯 하나에 담는다.
 *          이 값이 "stride 의 몇 % 지점인가"는 여기서 신경 쓰지 않는다 — 그건
 *          stride 가 끝나 실측 길이를 아는 시점에 _ResamplePhaseCurve() 가
 *          결정한다. 즉 이 함수는 완전히 causal-estimation-free 하다.
 */
static void _AccumulateRawSample(uint32_t tick)
{
    for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
        s_raw_window_sum[f] += _SampleFeature(f);
    }
    s_raw_window_cnt++;

    if ((tick - s_raw_window_start_tick) < RAW_SAMPLE_PERIOD_MS) {
        return;
    }

    if (s_raw_count < RAW_BUF_LEN) {
        float inv = 1.0f / (float)s_raw_window_cnt;
        for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
            s_raw_buf[f][s_raw_count] = s_raw_window_sum[f] * inv;
        }
        s_raw_count++;
    }
    /* RAW_BUF_LEN 초과분은 버린다. 다음 footContact rising edge에서
     * STRIDE_MAX_MS를 넘긴 간격은 재동기화용 경계로만 쓰고 분류하지 않는다. */

    memset(s_raw_window_sum, 0, sizeof(s_raw_window_sum));
    s_raw_window_cnt = 0U;
    s_raw_window_start_tick = tick;
}

/**
 * @brief 완료된 stride 의 원시 버퍼를 실측 stride_ms 로 101 위상 bin 에 재표본화
 * @details raw_buf[j] 는 시간창 [j*PERIOD, (j+1)*PERIOD) 의 평균이며, 대표
 *          시각은 창 중앙 (j+0.5)*PERIOD 로 둔다. 각 위상점 theta_i(0~1)에
 *          대응하는 실제 시각 t_i = theta_i * stride_ms 를 raw_buf 축에서
 *          선형보간한다 — build_h_d_reference_curves.m 의
 *          interp1(thetaImuU, ..., thetaGrid) 와 동일한 연산이다.
 *          호출 시점(heel-strike)에는 이미 stride가 끝나 있으므로, 여기서 계산하는
 *          위상축은 causal 추정이 전혀 섞이지 않은 100% 사후값이다.
 */
static void _ResamplePhaseCurve(uint32_t stride_ms)
{
    memset(s_obs_cnt, 0, sizeof(s_obs_cnt));

    if (s_raw_count < 2U) {
        return;   /* 표본 부족 — 모든 bin 을 invalid(cnt=0) 로 남긴다 */
    }

    uint16_t last_j = (uint16_t)(s_raw_count - 2U);

    for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
        float t_target = ((float)i / (float)(RC_THETA_POINTS - 1)) * (float)stride_ms;

        float pos = (t_target / (float)RAW_SAMPLE_PERIOD_MS) - 0.5f;
        if (pos < 0.0f) { pos = 0.0f; }

        uint16_t j0 = (uint16_t)pos;
        if (j0 > last_j) { j0 = last_j; }
        float frac = pos - (float)j0;
        if (frac < 0.0f) { frac = 0.0f; }
        if (frac > 1.0f) { frac = 1.0f; }

        for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
            float a = s_raw_buf[f][j0];
            float b = s_raw_buf[f][j0 + 1U];
            s_obs_sum[f][i] = a + ((b - a) * frac);   /* cnt=1 이므로 sum == 보간값 */
        }
        s_obs_cnt[i] = 1U;
    }
}

// ==================== TURN 검출기 (층 2) ====================

/** @brief -180~+180 deg 로 정규화 */
static float _WrapDeg180(float deg)
{
    while (deg > 180.0f)  { deg -= 360.0f; }
    while (deg < -180.0f) { deg += 360.0f; }
    return deg;
}

/**
 * @brief heading(yaw) 변화량 기반 TURN 판정
 * @details 기준곡선으로는 TURN 을 구분할 수 없다(TURN vs WALK 는 41 feature
 *          전부에서 Cohen's d <= 0.80). 회전은 yaw 현상인데 기준곡선의 IMU feature 는
 *          sagittal tilt 와 gyro 크기뿐이라 방향 정보가 없기 때문이다.
 *          H10 은 Camargo 에 없는 좌우 TransverseYaw 를 제공하므로 여기서 판정한다.
 *
 *          절대 yaw 는 자기/적분 드리프트가 있으므로 절대값을 쓰지 않고
 *          TURN_WINDOW_MS 시간창 안에서의 "변화량"만 본다 -> 드리프트 내성.
 *          진입/해제 임계를 다르게 두어 경계에서의 flicker 를 막는다.
 */
static void _UpdateTurnDetector(uint32_t tick)
{
    if ((tick - s_yaw_last_tick) < TURN_SAMPLE_PERIOD_MS) {
        return;
    }
    s_yaw_last_tick = tick;

    /* 좌우 고관절 IMU yaw 의 평균 = 골반 heading 근사 */
    float yaw = 0.5f * (XM.status.h10.leftHipImuTransverseYaw
                        + XM.status.h10.rightHipImuTransverseYaw);

    /* 시간창의 가장 오래된 값과 비교 (버퍼가 다 차기 전엔 가장 앞의 값) */
    uint16_t oldest_idx = s_yaw_filled ? s_yaw_idx : 0U;
    if (s_yaw_filled || (s_yaw_idx > 0U)) {
        s_yaw_delta_deg = _WrapDeg180(yaw - s_yaw_buf[oldest_idx]);
    } else {
        s_yaw_delta_deg = 0.0f;
    }

    s_yaw_buf[s_yaw_idx] = yaw;
    s_yaw_idx++;
    if (s_yaw_idx >= TURN_YAW_BUF_LEN) {
        s_yaw_idx = 0U;
        s_yaw_filled = true;
    }

    float mag = fabsf(s_yaw_delta_deg);
    if (!s_turn_active && (mag >= TURN_ENTER_YAW_DEG)) {
        s_turn_active = true;
    } else if (s_turn_active && (mag <= TURN_EXIT_YAW_DEG)) {
        s_turn_active = false;
    }
}

// ==================== 캘리브레이션 (도메인 보정) ====================

static void _StartCalibration(void)
{
    s_calibrating = true;
    s_calib_done = false;
    s_calib_stride_idx = 0U;
    memset(s_calib_sum, 0, sizeof(s_calib_sum));
    memset(s_calib_cnt, 0, sizeof(s_calib_cnt));
    XM_SendUsbDebugMessage("[LMC] 캘리브레이션 시작 — 평지에서 8 stride 걸어주세요\r\n");
}

/** @brief 완료된 1 stride 의 위상곡선을 캘리브레이션 누적기에 더한다 */
static void _AccumulateCalibStride(void)
{
    for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
        if (s_obs_cnt[i] == 0U) { continue; }
        float inv = 1.0f / (float)s_obs_cnt[i];
        for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
            s_calib_sum[f][i] += s_obs_sum[f][i] * inv;
        }
        s_calib_cnt[i]++;
    }

    s_calib_stride_idx++;

    char buf[64];
    snprintf(buf, sizeof(buf), "[LMC] 캘리브레이션 %u/%u stride\r\n",
             (unsigned)s_calib_stride_idx, (unsigned)CALIB_STRIDE_COUNT);
    XM_SendUsbDebugMessage(buf);

    if (s_calib_stride_idx >= CALIB_STRIDE_COUNT) {
        _FinishCalibration();
    }
}

/** @brief 곡선의 평균/표준편차(위상축 방향) 계산. 유효하면 true */
static bool _CurveStats(const float *curve, float *out_mean, float *out_std)
{
    float sum = 0.0f;
    for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
        sum += curve[i];
    }
    float mean = sum / (float)RC_THETA_POINTS;

    float var = 0.0f;
    for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
        float d = curve[i] - mean;
        var += d * d;
    }
    var /= (float)RC_THETA_POINTS;

    *out_mean = mean;
    *out_std = sqrtf(var);
    return (*out_std > 1e-9f);
}

/**
 * @brief 캘리브레이션 마감 — 위상 offset / sign / scale / offset 추정 (3장)
 */
static void _FinishCalibration(void)
{
    static float own[RC_FEATURE_COUNT][RC_THETA_POINTS];

    /* --- 1) 우리 자신의 WALK 위상곡선 확정 --- */
    for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
        float inv = (s_calib_cnt[i] > 0U) ? (1.0f / (float)s_calib_cnt[i]) : 0.0f;
        for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
            own[f][i] = s_calib_sum[f][i] * inv;
        }
    }

    /* 빈 bin 보간 (stride 마다 샘플이 안 들어온 위상이 있을 수 있음) */
    for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
        for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
            if (s_calib_cnt[i] > 0U) { continue; }
            uint8_t prev = (i == 0U) ? (RC_THETA_POINTS - 1U) : (i - 1U);
            uint8_t next = (uint8_t)((i + 1U) % RC_THETA_POINTS);
            own[f][i] = 0.5f * (own[f][prev] + own[f][next]);
        }
    }

    /* --- 2) 잔여 위상 offset 추정 ---
     * 런타임과 기준곡선 모두 right heel-strike 원점을 쓰지만, H10 접촉 검출
     * 지연과 센서 필터 지연은 남을 수 있다. THIGH_GYROMAG 원형 상관으로 그
     * 잔여 시프트를 찾고 모든 feature가 공유한다. */
    const float *ref = &kRcMean[RC_MODE_WALK][RC_FEAT_THIGH_GYROMAG][0];
    float ref_mean, ref_std, own_mean, own_std;
    (void)_CurveStats(ref, &ref_mean, &ref_std);
    (void)_CurveStats(&own[RC_FEAT_THIGH_GYROMAG][0], &own_mean, &own_std);

    int best_shift = 0;
    float best_corr = -1e30f;
    for (int k = 0; k < RC_THETA_POINTS; k++) {
        float acc = 0.0f;
        for (int i = 0; i < RC_THETA_POINTS; i++) {
            int j = (i + k) % RC_THETA_POINTS;
            acc += (own[RC_FEAT_THIGH_GYROMAG][i] - own_mean) * (ref[j] - ref_mean);
        }
        if (acc > best_corr) {
            best_corr = acc;
            best_shift = k;
        }
    }
    s_phase_shift = best_shift;

    /* --- 3) feature 별 sign / scale / offset --- */
    for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
        s_calib[f].enabled = true;
        s_calib[f].sign = 1.0f;
        s_calib[f].scale = 1.0f;
        s_calib[f].offset = 0.0f;

#if (MUSCLE_SITE_COUNT < 2U)
        if (f == RC_FEAT_EMG_SITE1_RMS) {
            s_calib[f].enabled = false;   /* 자리 수가 모자라면 자동 비활성화 */
            continue;
        }
#endif

        const float *rc = &kRcMean[RC_MODE_WALK][f][0];
        float rc_mean, rc_std, ow_mean, ow_std;
        bool rc_ok = _CurveStats(rc, &rc_mean, &rc_std);
        bool ow_ok = _CurveStats(&own[f][0], &ow_mean, &ow_std);

        if (!rc_ok || !ow_ok) {
            /* 신호가 거의 평탄(전극 미부착 등) -> 이 feature 는 쓰지 않는다 */
            s_calib[f].enabled = false;
            continue;
        }

        if ((f == RC_FEAT_EMG_SITE0_RMS) || (f == RC_FEAT_EMG_SITE1_RMS)) {
            /* RMS envelope는 음수가 될 수 없으므로 sign 반전은 물리적으로 잘못이다.
             * 시간축 차이는 분류 시 _ReferenceThetaIndex()가 따로 보상한다. */
            s_calib[f].sign = 1.0f;
        } else {
            /* IMU 부호: 위상 정렬 상태에서의 상관 부호 */
            float acc = 0.0f;
            for (int i = 0; i < RC_THETA_POINTS; i++) {
                int j = (i + s_phase_shift) % RC_THETA_POINTS;
                acc += (own[f][i] - ow_mean) * (rc[j] - rc_mean);
            }
            s_calib[f].sign = (acc >= 0.0f) ? 1.0f : -1.0f;
        }
        s_calib[f].scale = rc_std / ow_std;
        s_calib[f].offset = rc_mean - (s_calib[f].sign * s_calib[f].scale * ow_mean);
    }

    s_calibrating = false;
    s_calib_done = true;

    int n_enabled = 0;
    for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
        if (s_calib[f].enabled) { n_enabled++; }
    }

    char buf[128];
    snprintf(buf, sizeof(buf),
             "[LMC] 캘리브레이션 완료: phaseShift=%d/%d, 활성 feature=%d/%d\r\n",
             s_phase_shift, (int)RC_THETA_POINTS,
             n_enabled, (int)RC_FEATURE_COUNT);
    XM_SendUsbDebugMessage(buf);

    for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
        snprintf(buf, sizeof(buf), "[LMC]   feat%u en=%d sign=%+.0f scale=%.4g offset=%.4g\r\n",
                 (unsigned)f, s_calib[f].enabled ? 1 : 0,
                 (double)s_calib[f].sign, (double)s_calib[f].scale,
                 (double)s_calib[f].offset);
        XM_SendUsbDebugMessage(buf);
    }
}

// ==================== 층 3: z-score 뱅크 분류 ====================

/**
 * @brief 완성된 stride 를 5개 보행모드 기준곡선 전부와 비교해 z 를 계산
 * @details z^m = sqrt( (1/W) * sum_f w_f * mean_theta [ (x_cal - r^m)/sigma^m ]^2 )
 *          가장 z 가 작은 모드가 후보. 2위와의 상대 마진을 tanh 로 압축해
 *          dual-threshold 히스테리시스에 넣고, 통과한 것만 다수결에 투표한다.
 */
static uint8_t _ReferenceThetaIndex(uint8_t feat_idx, uint8_t theta_idx,
                                    uint32_t stride_ms)
{
    int index = (int)theta_idx + s_phase_shift;

    if ((feat_idx == RC_FEAT_EMG_SITE0_RMS)
        || (feat_idx == RC_FEAT_EMG_SITE1_RMS)) {
        /* trailing RMS(t)는 centered RMS(t-75 ms)와 같은 시간 중심을 갖는다.
         * 보폭마다 길이가 다르므로 75 ms를 고정 theta가 아니라 현재 stride_ms로
         * 환산한다. 미래 샘플이나 정답 전환 시각은 사용하지 않는다. */
        uint32_t half_window_ms = RC_EMG_RMS_WINDOW_MS / 2U;
        uint32_t delay_bins = ((half_window_ms * (RC_THETA_POINTS - 1U))
                              + (stride_ms / 2U)) / stride_ms;
        index -= (int)delay_bins;
    }

    index %= (int)RC_THETA_POINTS;
    if (index < 0) { index += (int)RC_THETA_POINTS; }
    return (uint8_t)index;
}

static void _ClassifyStride(uint32_t stride_ms)
{
    float z[RC_MODE_COUNT];

    for (uint8_t m = 0U; m < RC_MODE_COUNT; m++) {
        float acc = 0.0f;
        float wsum = 0.0f;

        for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
            if (!s_calib[f].enabled) { continue; }

            float w = k_feature_weight[f];
            float feat_acc = 0.0f;
            uint16_t n = 0U;

            for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
                if (s_obs_cnt[i] == 0U) { continue; }   /* 샘플 없는 위상은 건너뜀 */

                float x = s_obs_sum[f][i] / (float)s_obs_cnt[i];
                float xc = (s_calib[f].sign * s_calib[f].scale * x) + s_calib[f].offset;

                uint8_t j = _ReferenceThetaIndex(f, i, stride_ms);
                float d = (xc - kRcMean[m][f][j]) / kRcStd[m][f][j];
                feat_acc += d * d;
                n++;
            }

            if (n > 0U) {
                acc += w * (feat_acc / (float)n);
                wsum += w;
            }
        }

        z[m] = (wsum > 0.0f) ? sqrtf(acc / wsum) : 1e30f;
    }

    /* 1위 / 2위 찾기 */
    uint8_t best = 0U;
    for (uint8_t m = 1U; m < RC_MODE_COUNT; m++) {
        if (z[m] < z[best]) { best = m; }
    }
    float second = 1e30f;
    for (uint8_t m = 0U; m < RC_MODE_COUNT; m++) {
        if ((m != best) && (z[m] < second)) { second = z[m]; }
    }

    s_last_z_best = z[best];
    s_last_z_second = second;

    /* ---------- 층 2.5: IDLE 전환 증거 ----------
     * 위상무관(평탄) 템플릿 대비 z 를 같은 방식으로 계산한다. 보행 중이면
     * x(theta) 가 크게 출렁여 평탄 템플릿에서 멀어지므로 z_idle 이 커지고,
     * 전이/재배치 구간이면 x(theta) 가 평탄해 z_idle 이 작아진다. */
    s_last_z_idle = _ComputeIdleZ();
    s_in_transition = (s_last_z_idle < z[best]);

    /* 상대 마진 -> tanh 압축 (Terrain_Intent_Classifier.c 의 fuzzy score 패턴) */
    float denom = (z[best] > 1e-6f) ? z[best] : 1e-6f;
    float margin_raw = (second - z[best]) / denom;
    float sensitivity = k_sensitivity_scale[s_sensitivity_idx];
    s_last_margin = tanhf(sensitivity * margin_raw);

    /* 안정 보폭은 기존 0.35 문턱을 유지한다. IDLE 전환 증거가 나타난 경우에는
     * 현재 상태와 다른 후보만 0.15 문턱으로 3/2 버퍼에 넣는다. 따라서 첫
     * transition stride를 통째로 버리지 않으면서도 한 표만으로 전환되지는 않는다. */
    bool stable_vote = !s_in_transition && (s_last_margin >= MODE_CONFIRM_TH);
    bool transition_vote = s_in_transition
                           && (best != (uint8_t)s_bank_mode)
                           && (s_last_margin >= MODE_TRANSITION_CONFIRM_TH);
    if (stable_vote || transition_vote) {
        _PushVote(best);
        _UpdateModeFromVotes();
    }
}

/**
 * @brief IDLE(전이/재배치) 평탄 템플릿 대비 z 계산
 * @details kRcIdleMean/kRcIdleStd 는 위상 의존성이 없는 스칼라 1쌍이다.
 *          idle 곡선의 flatness 가 1.7~7.1% 로 사실상 0 이라, 101점 곡선으로
 *          두어도 정보가 늘지 않기 때문이다(analyze_idle_as_class.py).
 *          보행모드 z 와 동일한 가중/정규화를 써야 크기 비교가 성립한다.
 */
static float _ComputeIdleZ(void)
{
    float acc = 0.0f;
    float wsum = 0.0f;

    for (uint8_t f = 0U; f < RC_FEATURE_COUNT; f++) {
        if (!s_calib[f].enabled) { continue; }

        float w = k_feature_weight[f];
        float feat_acc = 0.0f;
        uint16_t n = 0U;

        for (uint8_t i = 0U; i < RC_THETA_POINTS; i++) {
            if (s_obs_cnt[i] == 0U) { continue; }

            float x = s_obs_sum[f][i] / (float)s_obs_cnt[i];
            float xc = (s_calib[f].sign * s_calib[f].scale * x) + s_calib[f].offset;

            /* 위상 무관 — 모든 theta 에서 같은 평균/표준편차와 비교 */
            float d = (xc - kRcIdleMean[f]) / kRcIdleStd[f];
            feat_acc += d * d;
            n++;
        }

        if (n > 0U) {
            acc += w * (feat_acc / (float)n);
            wsum += w;
        }
    }

    return (wsum > 0.0f) ? sqrtf(acc / wsum) : 1e30f;
}

static void _PushVote(uint8_t bank_mode)
{
    s_vote_buf[s_vote_idx] = bank_mode;
    s_vote_idx = (uint8_t)((s_vote_idx + 1U) % VOTE_WINDOW);
    if (s_vote_filled < VOTE_WINDOW) { s_vote_filled++; }
}

/** @brief 투표 버퍼에서 과반을 얻은 모드가 있으면 전환 (Qian 2022 majority filter) */
static void _UpdateModeFromVotes(void)
{
    if (s_vote_filled < VOTE_WINDOW) {
        return;   /* 아직 판단에 충분한 stride 가 모이지 않음 */
    }

    uint8_t count[RC_MODE_COUNT] = { 0 };
    for (uint8_t i = 0U; i < VOTE_WINDOW; i++) {
        count[s_vote_buf[i]]++;
    }

    for (uint8_t m = 0U; m < RC_MODE_COUNT; m++) {
        if (count[m] >= VOTE_MAJORITY) {
            s_bank_mode = (RcMode_t)m;
            return;
        }
    }
    /* 과반이 없으면 직전 상태 유지 */
}

// ==================== 버튼 / LED / USB ====================

static void _HandleButtonInput(void)
{
    if (XM_GetButtonEvent(XM_BTN_1) == XM_BTN_CLICK) {
        _StartCalibration();
    }

    if (XM_GetButtonEvent(XM_BTN_2) == XM_BTN_CLICK) {
        s_vote_idx = 0U;
        s_vote_filled = 0U;
        s_bank_mode = RC_MODE_WALK;
        s_turn_active = false;
        XM_SendUsbDebugMessage("[LMC] BTN2: 투표/모드 리셋\r\n");
    }

    if (XM_GetButtonEvent(XM_BTN_3) == XM_BTN_CLICK) {
        s_sensitivity_idx = (uint8_t)((s_sensitivity_idx + 1U) % NUM_SENSITIVITY_LEVELS);
        char buf[48];
        snprintf(buf, sizeof(buf), "[LMC] BTN3: 민감도 = %.1fx\r\n",
                 (double)k_sensitivity_scale[s_sensitivity_idx]);
        XM_SendUsbDebugMessage(buf);
    }
}

static void _UpdateLedIndicators(void)
{
    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 500);

    if (s_calibrating) {
        XM_SetLedEffect(XM_LED_2, XM_LED_BLINK, 200);
    } else if (s_calib_done) {
        XM_SetLedState(XM_LED_2, XM_ON);
    } else {
        XM_SetLedState(XM_LED_2, XM_OFF);
    }

    /* LED3: 정지=소등, 평지보행=상시점등, 그 외 지형/회전=점멸 */
    if (s_current_mode == LOCO_STAND) {
        XM_SetLedState(XM_LED_3, XM_OFF);
    } else if (s_current_mode == LOCO_WALK) {
        XM_SetLedState(XM_LED_3, XM_ON);
    } else {
        XM_SetLedEffect(XM_LED_3, XM_LED_BLINK, 150);
    }
}

static void _UpdateUsbDebug(void)
{
    uint32_t now = XM_GetTick();
    if ((now - s_usb_debug_timer) < USB_DEBUG_PERIOD_MS) { return; }
    s_usb_debug_timer = now;

    char buf[176];
    snprintf(buf, sizeof(buf),
             "LMC | %-11s%s z1=%.2f z2=%.2f zIdle=%.2f mgn=%.2f th=%.2f dYaw=%+6.1f calib=%d\r\n",
             k_loco_name[s_current_mode],
             s_in_transition ? "*" : " ",   /* * = IDLE 전환 증거 */
             (double)s_last_z_best, (double)s_last_z_second, (double)s_last_z_idle,
             (double)s_last_margin, (double)s_theta,
             (double)s_yaw_delta_deg, s_calib_done ? 1 : 0);
    XM_SendUsbDebugMessage(buf);
}

static void _UpdateStreamData(void)
{
    /* site0 좌우 EMG 비대칭 — Camargo 에는 없는 신호라 기준곡선 판정에는 쓰지 않고
     * 진단용으로만 스트리밍한다. TURN 개선 여지를 실측으로 조사할 때 사용. */
    float r = s_emg_rms_v[EMG_CH_INDEX(0U, EMG_SIDE_R)];
    float l = s_emg_rms_v[EMG_CH_INDEX(0U, EMG_SIDE_L)];
    float sum = r + l;
    s_stream.emg_asym0 = (sum > 1e-9f) ? ((r - l) / sum) : 0.0f;

    s_stream.mode       = (float)s_current_mode;
    s_stream.z_best     = s_last_z_best;
    s_stream.z_second   = s_last_z_second;
    s_stream.margin     = s_last_margin;
    s_stream.theta      = s_theta;
    s_stream.yaw_delta  = s_yaw_delta_deg;
    s_stream.calib_done = s_calib_done ? 1.0f : 0.0f;
    s_stream.z_idle     = s_last_z_idle;
    s_stream.in_transition = s_in_transition ? 1.0f : 0.0f;

    XM_SendUsbDataWithId(&s_stream, sizeof(s_stream), USB_STREAM_ID);
}
