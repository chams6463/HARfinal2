/**
 ******************************************************************************
 * @file    Locomotion_Mode_Classifier.c
 * @brief   7-모드 실시간 지형/보행의도 분류기
 *          STAND / WALK / TURN / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN
 *          (FSR/발접촉 미사용 — 고관절 IMU + 엔코더 + 외부 ADC EMG 기반)
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
 *   [층 2.5] IDLE(전이/재배치) 투표 거부권
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
 *       z_IDLE 이 최우수 보행모드보다 작으면 그 stride 의 지형 투표를 억제한다.
 *       판별력이 약해(주변분포 기준 d_subj 0.68~1.14) 모드 argmin 경쟁자로
 *       쓰면 정상 보행 중 오검출이 잦으므로, 반드시 거부권으로만 쓴다.
 *       효과: 기존의 수동적 "마진 미달 -> 투표 안 함"이 "전이 중이라는 증거가
 *       있음 -> 투표 안 함"이라는 능동적 판정으로 바뀐다.
 *
 *   [층 3] z-score 뱅크 (기준곡선 사용, 보행 5모드)
 *       WALK / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN 만 담당.
 *       MHE 위상 theta(0~1) 에서 r^m(theta) 대비 z 를 5모드 동시 계산하고
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
 *                       체간 전후 경사이므로 H10 pelvicAngle 로 그대로 전이된다.
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
 *   - Camargo TILT = atan2(AccX,AccZ) [deg], H10 pelvicAngle 과 offset/sign 다름
 *   - Camargo EMG = mV 스케일, 우리 외부 ADC 전압과 무관
 *   - 기준곡선의 theta 원점 = gcRight heel-strike, 우리 theta 원점 = MHE
 * 이 값을 그대로 z-score 에 넣으면 전부 쓰레기가 된다.
 *
 * 해결: BTN1 캘리브레이션(평지 보행 CALIB_STRIDE_COUNT stride)에서 우리 자신의
 * WALK 위상곡선을 만든 뒤, 기준곡선의 WALK 곡선에 맞추는 계수를 자동 추정한다.
 *   1) 위상 offset : 우리 곡선을 원형 시프트하며 r^WALK 과의 상관이 최대가
 *                    되는 시프트량을 찾는다 (MHE <-> heel-strike 원점 차이 흡수).
 *                    가장 깨끗한 THIGH_GYROMAG 로 한 번만 추정해 전 feature 공유.
 *   2) 부호 sign   : 상관계수의 부호 (좌표계 반전 흡수)
 *   3) 스케일 a    : std(r^WALK) / std(우리 WALK)  (단위 차이 흡수)
 *   4) 오프셋 b    : mean(r^WALK) - sign*a*mean(우리 WALK)
 * 이후 x_cal = sign*a*x + b 로 변환한 값을 z-score 에 넣는다.
 * => 캘리브레이션 전에는 분류하지 않는다(LED2 로 표시).
 *
 * ============================================================================
 * 4. 위상 theta 추정 — 왜 좌우 평균이 아니라 오른쪽 허벅지만 쓰는가
 * ============================================================================
 * Terrain_Intent_Classifier.c 는 thigh_avg=(left+right)/2 의 국소 최솟값을
 * MHE 로 썼다. 그러나 좌우 허벅지 각도는 서로 약 180도 위상차라 평균을 내면
 * 기본 주파수 성분이 상쇄되고 2차 고조파만 남는다 => 검출되는 주기가 stride 가
 * 아니라 step(반 stride)이 되고, 진폭도 작아 디바운스가 불안정해진다.
 * 또한 기준곡선은 Camargo 의 gcRight(오른쪽 heel-strike) 기준 1 stride 이므로,
 * 위상 기준도 오른쪽 다리 한쪽으로 맞춰야 표와 정합한다.
 * => 본 파일은 rightThighAngle 단독의 국소 최솟값을 MHE 로 사용한다.
 *
 * ============================================================================
 * 5. 위상 리샘플링 — 왜 "직전 stride 길이로 정규화"를 버렸는가
 * ============================================================================
 * 최초 버전은 theta(t) = (t - t_MHE) / stride_period_직전 으로 매 ms 마다
 * causal 하게 위상을 추정하며 그 즉시 101 bin 에 누적했다. stride 길이가
 * 안정적인 정상상태 WALK 안에서는 그럭저럭 버티지만, 두 가지 구조적 결함이
 * 있었다:
 *   1) 모드 전환 stride 에서 가장 심하게 틀어진다. 예를 들어 WALK(~1.1s)에서
 *      RAMP_UP(~1.2~1.4s)으로 들어가는 stride 는 WALK 길이로 나눠지므로,
 *      실제 theta=0.7 지점인데 추정치는 이미 0.8~0.9 로 앞서간다. 정확히
 *      "잘 분류해야 하는" 전환 stride 에서 위상축이 제일 많이 왜곡되는
 *      구조였다.
 *   2) stride 가 길어지면(추정 대비) theta 가 1.0 에 먼저 도달해 버려서,
 *      진짜 MHE 가 오기 전까지 남은 후반부 구간을 통째로 유실했다. 판별력이
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
 * 시점(=MHE, 어차피 분류가 일어나는 시점)에 사후적으로 101 bin 에 리샘플링한다.
 *   - stride 진행 중에는 위상을 전혀 추정하지 않고, RAW_SAMPLE_PERIOD_MS(10ms)
 *     간격 창평균으로 원시 버퍼(s_raw_buf)에 그냥 쌓는다(_AccumulateRawSample).
 *   - MHE 로 실측 stride_ms 를 알게 된 순간, theta_i = i/100 에 대응하는
 *     실제 시각 t_i = theta_i * stride_ms 를 원시 버퍼에서 선형보간한다
 *     (_ResamplePhaseCurve). 이는 build_h_d_reference_curves.m 이 오프라인
 *     에서 쓴 interp1(thetaImuU, ..., thetaGrid) 와 정확히 같은 연산이며,
 *     기준곡선을 만든 방식 그대로를 온라인에서 재현하는 것이다.
 *   - 레이턴시는 늘지 않는다. _ClassifyStride() 는 애초에 MHE 시점에만
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
 *        Terrains," IEEE RA-L 7(3):6439-6446. (MHE 이벤트 / majority filter)
 * @see   Camargo, J. et al. (2021) "A comprehensive, open-source dataset of
 *        lower limb biomechanics in multiple conditions," J. Biomech. 119.
 * @version 1.0
 * @date    Aug 16, 2026
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

/* --- 정지(STAND) 게이트 --- */
#define MIN_FWD_VELOCITY        0.1f    /* m/s — 이하이고 각속도도 낮으면 정지 */
#define STAND_GYRO_TH_DPS       30.0f   /* deg/s — 고관절 IMU 각속도 크기 임계 */

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
 * 대신 시간축 그대로(10ms 간격 창평균)로 원시 버퍼에 쌓아두고, MHE 로 stride 가
 * 끝나 실측 길이(stride_ms)를 아는 시점에 그 실측값으로 101 위상 bin 에
 * 리샘플링한다(build_h_d_reference_curves.m 의 interp1 과 동일 연산). 이렇게
 * 하면 stride 길이를 추정할 필요가 아예 없어져서, 모드 전환처럼 stride-to-stride
 * 길이가 크게 바뀌는 구간에서도 위상축이 왜곡되지 않는다. 분류는 원래도 MHE
 * 시점(stride 완료 후)에만 일어나므로 레이턴시는 늘지 않는다. */
#define RAW_SAMPLE_PERIOD_MS     10U     /* 원시 버퍼 창 길이 */
#define RAW_BUF_LEN              (STRIDE_MAX_MS / RAW_SAMPLE_PERIOD_MS)  /* 300 */
#define RAW_MIN_SAMPLES          10U     /* 리샘플링 최소 원시 샘플 수(방어) */

/* --- MHE(Maximum Hip Extension) 이벤트 검출 --- */
#define MHE_MIN_SWING_DEG       4.0f    /* 국소 최솟값 확정 전 최소 반등 폭 */
#define MHE_REFRACTORY_MS       400U    /* 불응기간 — stride 단위이므로 step보다 길게 */

/* --- 캘리브레이션 --- */
#define CALIB_STRIDE_COUNT      8U      /* 평지 보행 baseline 수집 stride 수 */

/* --- 분류 히스테리시스 (margin score: 0~1) ---
 * 3-class 였던 Terrain_Intent_Classifier.c 는 양방향 dual-threshold 였지만,
 * 여기는 5-way argmin 이라 "1위와 2위의 상대 마진"이 곧 확신도다.
 * 마진이 이 값에 못 미치는 stride 는 투표 자체를 하지 않아(완충지대)
 * 직전 다수결 상태가 유지된다 — 경계 지형에서의 flicker 방지. */
#define MODE_CONFIRM_TH         0.35f

/* --- 다수결 투표 ---
 * 5-way 경쟁(뱅크 5모드) 대비 window/majority 를 완화했다. 최초값(5/3, 3표
 * 필요)은 2-class(LEVEL/INCLINE) 뿐이던 Terrain_Intent_Classifier.c 에서
 * 재튜닝 없이 그대로 옮긴 값이었는데, 이 경우 확신 있는 stride 가 매번
 * 이어져도 최소 3 stride(~3~4초) 지연되고, 전환 초반 stride 가 margin 미달
 * (완충지대)이나 IDLE 거부권에 걸려 투표를 건너뛰면 5~10초까지도 늘어났다.
 * 3/2 로 낮추면 확신 있는 stride 2개 연속(~2~2.5초)이면 전환되고, 경계
 * 지형에서의 flicker 방지는 margin 임계(MODE_CONFIRM_TH)와 IDLE 거부권이
 * 1차 방어선을 맡으므로 완화의 실질 위험은 낮다. */
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
    float in_transition;/* 0/1 — 직전 stride 가 IDLE 거부권에 걸렸는가 */
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

/* --- MHE 이벤트 검출 --- */
static float    s_prev_thigh_r_deg = 0.0f;
static bool     s_thigh_was_falling = false;
static float    s_rebound_deg = 0.0f;
static uint32_t s_last_mhe_tick = 0U;
static uint32_t s_stride_period_ms = 1000U;  /* 직전 stride 실측 길이 — 표시용 theta 분모.
                                               * 분류용 위상 리샘플링에는 쓰이지 않는다
                                               * (그 stride 의 실측 길이를 그대로 쓰므로). */
static bool     s_has_first_mhe = false;

/* --- 원시 버퍼 (진행 중인 stride, 10ms 창평균, 사후 리샘플링 대기) ---
 * 위상을 추정하지 않고 시간축 그대로 쌓는다 — RAW_SAMPLE_PERIOD_MS 참고. */
static float    s_raw_buf[RC_FEATURE_COUNT][RAW_BUF_LEN];
static float    s_raw_window_sum[RC_FEATURE_COUNT];
static uint16_t s_raw_window_cnt = 0U;
static uint16_t s_raw_count = 0U;            /* 확정된 원시 버퍼 슬롯 수 */
static uint32_t s_raw_window_start_tick = 0U;

/* --- 완료된 stride 를 101 위상 bin 으로 리샘플링한 결과 ---
 * _ResamplePhaseCurve() 가 MHE 시점에 한 번에 채운다(사후 처리). 각 bin 은
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
static bool       s_in_transition = false;   /* 직전 stride 가 IDLE 거부권에 걸림 */
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

static bool  _DetectMheEvent(float thigh_r_deg, uint32_t tick);
static void  _ResetStrideAccumulator(uint32_t tick);
static void  _AccumulateRawSample(uint32_t tick);
static void  _ResamplePhaseCurve(uint32_t stride_ms);

static void  _UpdateTurnDetector(uint32_t tick);
static float _WrapDeg180(float deg);

static void  _StartCalibration(void);
static void  _AccumulateCalibStride(void);
static void  _FinishCalibration(void);
static bool  _CurveStats(const float *curve, float *out_mean, float *out_std);

static void  _ClassifyStride(void);
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

    /* --- MHE / 위상 리샘플링 초기화 --- */
    s_prev_thigh_r_deg = XM.status.h10.rightThighAngle;
    s_thigh_was_falling = false;
    s_rebound_deg = 0.0f;
    s_last_mhe_tick = tick;
    s_stride_period_ms = 1000U;
    s_has_first_mhe = false;
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

    XM_SendUsbDebugMessage("[LMC] ACTIVE — 7모드 분류 시작 (FSR 미사용)\r\n");
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

    float thigh_r  = XM.status.h10.rightThighAngle;
    float fwd_vel  = XM.status.h10.forwardVelocity;
    float gyro_mag = _HipGyroMagDps();

    /* ---------- 층 1: STAND 게이트 (기준곡선 미사용) ----------
     * 속도가 낮고 고관절 각속도도 작으면 즉시 정지로 판정하고 stride 누적을
     * 중단한다. Camargo 의 idle 라벨이 오염되어 r^STAND 를 만들 수 없으므로
     * 이 물리 게이트가 STAND 판정의 유일한 경로다. */
    if ((fwd_vel < MIN_FWD_VELOCITY) && (gyro_mag < STAND_GYRO_TH_DPS)) {
        s_current_mode = LOCO_STAND;
        s_prev_thigh_r_deg = thigh_r;      /* 재개 시 MHE 오검출 방지 */
        s_thigh_was_falling = false;
        s_rebound_deg = 0.0f;
        s_has_first_mhe = false;
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
    uint32_t elapsed = tick - s_last_mhe_tick;
    float theta = (float)elapsed / (float)s_stride_period_ms;
    if (theta < 0.0f) { theta = 0.0f; }
    if (theta > 1.0f) { theta = 1.0f; }
    s_theta = theta;

    /* ---------- 원시 버퍼 누적 (위상 추정과 무관, 시간축 그대로) ---------- */
    if (s_has_first_mhe) {
        _AccumulateRawSample(tick);
    }

    /* ---------- stride 마감 (MHE 이벤트) ---------- */
    if (_DetectMheEvent(thigh_r, tick)) {
        uint32_t stride_ms = tick - s_last_mhe_tick;
        bool valid = s_has_first_mhe
                     && (stride_ms >= STRIDE_MIN_MS)
                     && (stride_ms <= STRIDE_MAX_MS)
                     && (s_raw_count >= RAW_MIN_SAMPLES);

        /* stride 가 유효하면, 방금 실측된 stride_ms 로 원시 버퍼를 101 위상
         * bin 에 사후 리샘플링한다. 이 시점에야 진짜 길이를 알기 때문에
         * causal 추정 없이 build_h_d_reference_curves.m 과 동일한 interp1
         * 연산을 온라인에서 재현할 수 있다. */
        if (valid) {
            _ResamplePhaseCurve(stride_ms);
        }

        /* EMG 파이프라인이 준비되기 전(bias 추정 1s + RMS 윈도우 150ms)에는
         * EMG feature 가 0 이라 z 가 왜곡된다. 직전 세션의 캘리브레이션을
         * 물려받아 s_calib_done 이 이미 true 인 경우 ACTIVE 진입 직후 바로
         * 분류에 들어가버리므로, 준비 완료를 명시적으로 기다린다. */
        bool emg_ready = (s_emg_bias_ready && s_emg_buf_filled);

        if (valid && emg_ready) {
            if (s_calibrating) {
                _AccumulateCalibStride();
            } else if (s_calib_done) {
                _ClassifyStride();
            }
        }
        if (valid) {
            s_stride_period_ms = stride_ms;
        }

        s_has_first_mhe = true;
        s_last_mhe_tick = tick;
        _ResetStrideAccumulator(tick);
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
 *          대신 기준곡선과 같은 정의의 RMS(150ms boxcar)를 만든다.
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
            return XM.status.h10.pelvicAngle;

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

// ==================== MHE 위상 / stride 누적 ====================

/**
 * @brief MHE(Maximum Hip Extension) 검출 — 오른쪽 허벅지 각도의 국소 최솟값
 * @details 좌우 평균이 아니라 오른쪽 단독을 쓰는 이유는 4장 참고
 *          (좌우는 약 180도 위상차라 평균 시 기본파가 상쇄되어 step 주기가 잡힌다).
 */
static bool _DetectMheEvent(float thigh_r_deg, uint32_t tick)
{
    bool fired = false;
    float delta = thigh_r_deg - s_prev_thigh_r_deg;

    if (delta < 0.0f) {
        s_thigh_was_falling = true;
        s_rebound_deg = 0.0f;
    } else if ((delta > 0.0f) && s_thigh_was_falling) {
        s_rebound_deg += delta;

        bool refractory_ok = (tick - s_last_mhe_tick) >= MHE_REFRACTORY_MS;
        if ((s_rebound_deg >= MHE_MIN_SWING_DEG) && refractory_ok) {
            fired = true;
            s_thigh_was_falling = false;
            s_rebound_deg = 0.0f;
        }
    }

    s_prev_thigh_r_deg = thigh_r_deg;
    return fired;
}

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
    /* RAW_BUF_LEN 초과분(STRIDE_MAX_MS 를 넘는 비정상 stride)은 버린다 —
     * 어차피 _DetectMheEvent 의 STRIDE_MAX_MS 검사로 invalid 처리된다. */

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
 *          호출 시점(MHE)에는 이미 stride 가 끝나 있으므로, 여기서 계산하는
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

    /* --- 2) 위상 offset 추정 ---
     * 우리 theta 원점은 MHE, 기준곡선은 gcRight heel-strike 라 원점이 다르다.
     * 가장 깨끗한 THIGH_GYROMAG 로 원형 상관을 최대화하는 시프트를 찾아
     * 모든 feature 가 공유한다(위상 원점은 feature 마다 다르지 않으므로). */
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

        /* 부호: 위상 정렬 상태에서의 상관 부호 */
        float acc = 0.0f;
        for (int i = 0; i < RC_THETA_POINTS; i++) {
            int j = (i + s_phase_shift) % RC_THETA_POINTS;
            acc += (own[f][i] - ow_mean) * (rc[j] - rc_mean);
        }
        s_calib[f].sign = (acc >= 0.0f) ? 1.0f : -1.0f;
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
static void _ClassifyStride(void)
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

                uint8_t j = (uint8_t)((i + s_phase_shift) % RC_THETA_POINTS);
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

    /* ---------- 층 2.5: IDLE 거부권 ----------
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

    /* 히스테리시스 + IDLE 거부권:
     *   >= MODE_CONFIRM_TH 이고 전이 증거가 없을 때만 투표.
     *   전이 중이면(z_idle < z_best) 지형 투표를 억제해 직전 상태를 유지한다.
     *   IDLE 은 판별력이 약하므로(d_subj 0.68~1.14) 모드를 "대체"하지 않고
     *   투표를 "막기만" 한다 — 정상 보행 중 오검출의 영향을 최소화. */
    if ((s_last_margin >= MODE_CONFIRM_TH) && !s_in_transition) {
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
             s_in_transition ? "*" : " ",   /* * = IDLE 거부권 발동(전이 중) */
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
