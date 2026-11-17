# Locomotion_Mode_Classifier.c 정리

> 대상 파일: `D:\ifac_wearable_contest\XM_Apps\User_Algorithm\Locomotion_Mode_Classifier.c` (총 1418줄)
> 동반 파일: `ref_curve_tables.h`(자동생성 lookup table), `export_ref_curve_tables.m`(생성기, `Camargo_matlab` 폴더)
> 이 문서는 위 `.c` 파일 기준으로 작성되었으며, 줄 번호는 해당 시점의 파일을 기준으로 한다(향후 수정 시 어긋날 수 있음).

---

## 1. 전체 알고리즘 아키텍처

### 1.1 한 줄 요약

고관절 IMU + 엔코더 기반 관절각 + 외부 ADC EMG 2자리만으로, H10 외골격이 매 1ms **STAND / WALK / TURN / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN** 7가지 보행 의도를 실시간 분류한다. FSR(족압센서)은 사용하지 않는다.

### 1.2 왜 "3층 구조"인가

Camargo(2021) 공개 보행 데이터셋 7명(AB06~AB12)을 위상분해 Cohen's d로 실측 검증한 결과, 7모드를 하나의 위상곡선(reference curve) z-score 뱅크에 넣으면 **STAND와 TURN이 분류 불가능**하다는 것이 확인됐다(근거는 3장 참고). 그래서 아래처럼 판정 경로를 모드별로 분리했다.

```
                         ┌─────────────────────────────────────┐
                         │      Active_Loop() 1ms 매 틱마다      │
                         └─────────────────────────────────────┘
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
              [층 1] STAND 게이트   [층 2] TURN 검출기    [층 3] z-score 뱅크
             (기준곡선 미사용)      (기준곡선 미사용)      (기준곡선 사용, 5모드)
                    │                     │                     │
          fwd_vel<0.1 &&           |Δyaw| >= 30°           MHE stride 완료마다
          gyro<30deg/s              (800ms 창)              5모드 z 동시계산
                    │                     │                     │
                    │                     │              [층 2.5] IDLE 거부권
                    │                     │              z_idle<z_best 이면
                    │                     │              투표 억제
                    │                     │                     │
                    └─────────► 최종 우선순위 ◄──────────────────┘
                          TURN 활성 > 뱅크 결과 > (캘리브레이션前) WALK표시
                                          │
                                   s_current_mode (7-class)
```

우선순위는 `Active_Loop()` 722~731줄에 구현: **STAND 게이트가 즉시 최우선**(early return), 그 다음 **TURN 활성 여부**, 마지막으로 **z-score 뱅크 결과**.

### 1.3 7-class 출력 매핑

| `LocoMode_t` (열거값) | 의미 | 판정 경로 |
|---|---|---|
| `LOCO_STAND` (0) | 정지 | 층 1 게이트 |
| `LOCO_WALK` (1) | 평지보행 | 층 3 뱅크 (`RC_MODE_WALK`) |
| `LOCO_TURN` (2) | 회전 | 층 2 검출기 |
| `LOCO_STAIRS_UP` (3) | 계단오름 | 층 3 뱅크 (`RC_MODE_STAIRS_UP`) |
| `LOCO_STAIRS_DOWN` (4) | 계단내림 | 층 3 뱅크 (`RC_MODE_STAIRS_DOWN`) |
| `LOCO_RAMP_UP` (5) | 경사오름 | 층 3 뱅크 (`RC_MODE_RAMP_UP`) |
| `LOCO_RAMP_DOWN` (6) | 경사내림 | 층 3 뱅크 (`RC_MODE_RAMP_DOWN`) |

뱅크 내부 열거값 `RcMode_t`(WALK=0~RAMP_DOWN=4)는 `k_bank_to_loco[]`(442~448줄) 테이블로 위 7-class에 매핑된다.

### 1.4 z-score 뱅크가 쓰는 feature 4개

| feature (`RcFeature_t`) | 실제 신호 소스 | 채택 이유 |
|---|---|---|
| `RC_FEAT_TRUNK_TILT` | `XM.status.h10.pelvicAngle` | 판별력의 50%가 자세 기울기(DC)에서 옴 |
| `RC_FEAT_THIGH_GYROMAG` | `sqrt(GyrX²+GyrY²+GyrZ²)` (고관절 IMU 3축) | θ=0.10/0.60~0.70 두 피크의 깨끗한 물리 파형 |
| `RC_FEAT_EMG_SITE0_RMS` | 외부 ADC EMG site0 (비복근, gastrocnemius) | greedy 선택 1순위, 15쌍 합계 gain +5.44 |
| `RC_FEAT_EMG_SITE1_RMS` | 외부 ADC EMG site1 (광근, vastus medialis) | greedy 선택 2순위, +0.72 |

각 feature마다 101점 위상곡선(`kRcMean[mode][feat][101]`, `kRcStd[...]`)이 `ref_curve_tables.h`에 있다. 5모드 × 4feature × 101점 × (mean+std) × 4B ≈ **15.8 KB** (.rodata, flash 상수).

### 1.5 판정 관련 변수·조건 총정리

| 변수/상수 | 값 | 의미 | 사용 위치 |
|---|---|---|---|
| `MIN_FWD_VELOCITY` | 0.1 m/s | STAND 게이트 속도 임계 | 655줄 |
| `STAND_GYRO_TH_DPS` | 30 deg/s | STAND 게이트 각속도 임계 | 655줄 |
| `TURN_WINDOW_MS` | 800 ms | TURN 판정용 yaw 변화 관측창 | 232줄 |
| `TURN_ENTER_YAW_DEG` | 30° | TURN 진입 임계(창 내 변화량) | 1011줄 |
| `TURN_EXIT_YAW_DEG` | 15° | TURN 해제 임계(히스테리시스) | 1013줄 |
| `STRIDE_MIN_MS` / `STRIDE_MAX_MS` | 500 / 3000 ms | stride 유효 길이 범위 | 688~689줄 |
| `MHE_MIN_SWING_DEG` | 4° | MHE 확정 최소 반등폭(디바운스) | 867줄 |
| `MHE_REFRACTORY_MS` | 400 ms | MHE 재검출 불응기간 | 866줄 |
| `RAW_SAMPLE_PERIOD_MS` | 10 ms | 원시버퍼 창 길이 | 250줄 |
| `RAW_MIN_SAMPLES` | 10개 | 리샘플링 최소 원시표본 수(방어) | 690줄 |
| `CALIB_STRIDE_COUNT` | 8 stride | 캘리브레이션 수집량 | 259줄 |
| `MODE_CONFIRM_TH` | 0.35 | tanh 압축 마진 임계(이상이면 투표) | 1259줄 |
| `VOTE_WINDOW` / `VOTE_MAJORITY` | **3 / 2** | 다수결 투표 버퍼/과반 기준(5-way 경쟁에 맞춰 완화됨, 3.6절 참고) | 268~278줄 |
| `k_sensitivity_scale[]` | 0.7 / 1.0 / 1.4 | BTN3 민감도 3단계 | 343줄 |
| `s_in_transition` | z_idle < z_best | IDLE 거부권 발동 여부 | 1246줄 |
| `s_calib_done` | bool | 캘리브레이션 완료 여부(false면 뱅크 분류 자체를 안 함) | 709, 727줄 |

### 1.6 상태머신(TSM)

`XM_TSM`(Task State Machine) 3상태: `XM_STATE_OFF → XM_STATE_STANDBY → XM_STATE_ACTIVE`.

- **OFF**: CM 미연결. 연결되면 STANDBY로.
- **STANDBY**: `h10Mode == XM_H10_MODE_ASSIST` 되면 ACTIVE로.
- **ACTIVE**: 1ms마다 `Active_Loop()` 실행. ASSIST 해제되면 STANDBY로 복귀.

---

## 2. 줄별 코드 내용 정리

### 2.1 파일 헤더 문서 (1~188줄)

| 줄 범위 | 내용 |
|---|---|
| 1~8 | `@file`/`@brief` — 7모드 분류기, FSR 미사용 명시 |
| 9~61 | **섹션1**: 3층 구조 설계 근거 전체(STAND/TURN/IDLE거부권/뱅크) |
| 63~86 | **섹션2**: feature 41개→4개 선정 근거(채택/제외 사유) |
| 88~107 | **섹션3**: 런타임 도메인 보정(단위/좌표계 차이와 BTN1 캘리브레이션으로 sign·scale·offset·위상offset 추정) |
| 109~118 | **섹션4**: MHE에 좌우평균 대신 오른쪽 허벅지 단독을 쓰는 이유 |
| 120~161 | **섹션5**: 위상 리샘플링 재설계 근거(구버전 causal 추정의 결함, HZD phase-portrait를 채택하지 않은 이유, 사후 리샘플링 채택 이유) |
| 163~171 | **섹션6**: 버튼(BTN1~3)/LED(LED1~3) 배치 |
| 173~187 | `@note`(리네이밍 사유), `@see`(참고문헌 2편), 버전정보 |

### 2.2 include 및 매크로 정의 (190~292줄)

| 줄 범위 | 내용 |
|---|---|
| 190~195 | `xm_api.h`, `ref_curve_tables.h`, `math.h`/`stdio.h`/`string.h` include |
| 203~221 | `MUSCLE_SITE_COUNT`(EMG 근육 자리 수, 확장 가능하게 상수화) 및 파생 매크로(`EMG_CH_COUNT`, `EMG_CH_INDEX` 등) |
| 223~225 | EMG RMS 이동윈도우 150ms, bias 추정 구간 1000ms |
| 227~229 | STAND 게이트 임계값 |
| 231~236 | TURN 검출기 시간창/임계값, `TURN_YAW_BUF_LEN=80` 파생 |
| 238~240 | stride 유효 길이 범위 |
| 242~252 | **원시버퍼 상수**(사후 리샘플링용) — 왜 causal 추정을 버렸는지 재요약 주석 포함 |
| 254~256 | MHE 검출 파라미터 |
| 258~259 | 캘리브레이션 stride 수 |
| 261~266 | 분류 히스테리시스(단일 확신 임계, 5-way argmin 구조 설명) |
| 268~278 | **다수결 투표 파라미터** — `VOTE_WINDOW=3`/`VOTE_MAJORITY=2`(완화 사유 주석 포함, 3.6절) |
| 280~281 | 민감도 레벨 수 |
| 283~288 | feature 가중치(IMU=1.0, EMG=0.8) |
| 290~292 | USB 디버그 주기, 스트림 ID(0xF4) |

### 2.3 열거형/구조체 (294~332줄)

| 줄 범위 | 내용 |
|---|---|
| 300~310 | `LocoMode_t` — 최종 7-class 열거형 |
| 312~318 | `FeatureCalib_t` — feature별 sign/scale/offset/enabled 런타임 보정계수 구조체 |
| 320~332 | `LocoStreamData_t` — USB 0xF4 스트리밍 페이로드(10개 float, 40B) |

### 2.4 static 변수 (334~448줄)

| 줄 범위 | 내용 |
|---|---|
| 340 | TSM 핸들 |
| 342~344 | 민감도 프리셋 배열/인덱스 |
| 346~353 | feature 가중치 배열(인덱스는 `RcFeature_t` 순서) |
| 355~372 | EMG 파이프라인 상태(ADC 핀 배열, bias, RMS 순환버퍼) |
| 374~382 | MHE 이벤트 상태(직전 허벅지각, 하강여부, 반등폭, 마지막 MHE tick, 직전 stride 길이) |
| 384~390 | **원시버퍼**(`s_raw_buf[4][300]`, 창합/카운트/확정슬롯수/창시작tick) |
| 392~398 | 리샘플링된 stride 결과(`s_obs_sum`, `s_obs_cnt` — 사후 처리로 채워짐) |
| 400~407 | 캘리브레이션 누적기 및 결과(`s_calib[]`, `s_phase_shift`, `s_calib_done`) |
| 409~415 | TURN 검출 상태(yaw 링버퍼, 활성 플래그) |
| 417~421 | 다수결 투표 버퍼 및 현재 뱅크 모드 |
| 423~430 | 최종 판정 결과/진단값(z_best, z_second, z_idle, margin, theta) |
| 432~434 | USB 타이머/스트림 버퍼 |
| 436~448 | 디버그용 모드 이름 테이블, 뱅크→7class 매핑 테이블 |

### 2.5 함수 프로토타입 (450~487줄)

TSM 콜백 5개, 신호취득 3개, MHE/위상 3개(`_ResetStrideAccumulator`/`_AccumulateRawSample`/`_ResamplePhaseCurve`), TURN 2개, 캘리브레이션 4개, 분류 4개(`_ClassifyStride`/`_ComputeIdleZ`/`_PushVote`/`_UpdateModeFromVotes`), 버튼/LED/USB 4개.

### 2.6 `User_Setup()` / `User_Loop()` (495~552줄)

| 줄 범위 | 내용 |
|---|---|
| 497~511 | TSM 생성, OFF/STANDBY/ACTIVE 3상태 등록(ACTIVE만 entry/loop/exit 전부 등록) |
| 513~523 | USB 0xF4 스트림 메타데이터(JSON, 10개 필드명/단위) 등록 |
| 527~530 | 외부 DIO 1~4번을 ADC로 전환(EMG 4채널 활성화, FSR과 핀 공유하지만 FSR 미사용) |
| 533 | `XM_SetControlMode(XM_CTRL_MONITOR)` — 토크 미인가, 순수 모니터링 |
| 539~541 | TSM 생성 실패 시 NULL 역참조 방지 early return |
| 543~545 | CM 연결 끊기면 OFF로 강제 전환 |
| 547 | `XM_TSM_Run()` — 현재 상태의 loop 콜백 실행 |
| 551 | `XM_IO_Update()` — 버튼 디바운싱/LED 타이머 틱(호출 안 하면 BTN1~3 죽음) |

### 2.7 TSM 콜백 (562~744줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 562~569 | `Off_Loop` | CM 연결되면 STANDBY로 |
| 571~577 | `Standby_Loop` | ASSIST 모드 감지되면 ACTIVE로 |
| 579~627 | `Active_Entry` | EMG/MHE/TURN/투표 상태 전부 초기화(단, `s_calib`은 유지 — 재캘리브레이션 불필요하게) |
| 632~736 | `Active_Loop` | **메인 루프**, 아래 2.8에서 세분 |
| 738~744 | `Active_Exit` | LED 원복, 종료 로그 |

### 2.8 `Active_Loop()` 세부 (632~736줄) — 매 1ms 실행

| 줄 범위 | 내용 |
|---|---|
| 634~638 | ASSIST 해제 감지 시 STANDBY로 조기 복귀 |
| 640 | 버튼 입력 처리 |
| 642 | 현재 tick 획득 |
| 644~645 | EMG 파이프라인 갱신, TURN 검출기 갱신 |
| 647~649 | 허벅지각/전진속도/고관절각속도크기 읽기 |
| 651~666 | **층1 STAND 게이트**: 속도·각속도 둘 다 낮으면 즉시 STAND 판정, stride 상태 리셋 후 return |
| 668~677 | 표시용 theta 추정(직전 stride 길이 기준, **분류에는 미사용** — USB 로그 전용) |
| 679~682 | `s_has_first_mhe`면 매 ms `_AccumulateRawSample()` 호출(원시버퍼 누적) |
| 684~720 | **stride 마감**: `_DetectMheEvent()`가 true면 stride_ms 계산 → 유효성 검사(`valid`) → 유효하면 `_ResamplePhaseCurve()`로 사후 리샘플링 → EMG 준비 완료(`emg_ready`) 확인 → 캘리브레이션 중이면 `_AccumulateCalibStride()`, 완료됐으면 `_ClassifyStride()` → stride 상태 리셋 |
| 722~731 | **층2 TURN override**: `s_turn_active`면 TURN, 아니면 캘리브레이션 완료 여부에 따라 뱅크결과/WALK표시 |
| 733~735 | LED/USB디버그/스트림 갱신 |

### 2.9 신호 취득 함수 (748~846줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 757~763 | `_HipGyroMagDps` | 오른쪽 고관절 IMU 3축 자이로 크기 `sqrt(Gx²+Gy²+Gz²)` [deg/s] |
| 770~816 | `_UpdateEmg` | ADC 읽기(mV→V) → 진입 후 1초간 bias 추정 → bias 제거 → 제곱값 150샘플 순환버퍼 이동합 → RMS |
| 823~846 | `_SampleFeature` | feature index → 현재 측정값(원단위 그대로, 보정은 안 함) 매핑 |

### 2.10 MHE 검출 및 위상 리샘플링 (848~961줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 855~876 | `_DetectMheEvent` | 오른쪽 허벅지각 국소최솟값(하강→상승 전환) 검출, 반등폭 4°·불응기간 400ms 디바운스 |
| 878~887 | `_ResetStrideAccumulator` | `s_obs_sum/cnt`, 원시버퍼 상태 전부 0으로, 창시작 tick 갱신 |
| 896~920 | `_AccumulateRawSample` | 10ms 창평균으로 원시버퍼(`s_raw_buf`)에 샘플 push. **위상 추정 전혀 안 함** |
| 932~961 | `_ResamplePhaseCurve` | MHE 시점에 실측 `stride_ms`로 101 bin 선형보간(사후 리샘플링). `s_obs_sum/cnt` 채움 |

### 2.11 TURN 검출기 (963~1016줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 966~971 | `_WrapDeg180` | -180~+180 wrap |
| 984~1016 | `_UpdateTurnDetector` | 좌우 TransverseYaw 평균 → 800ms 전 값과의 차이(`s_yaw_delta_deg`) → 진입 30°/해제 15° 히스테리시스로 `s_turn_active` 갱신 |

### 2.12 캘리브레이션 (1018~1182줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 1020~1028 | `_StartCalibration` | BTN1 클릭 시 호출, 누적기 초기화 |
| 1031~1052 | `_AccumulateCalibStride` | 완료된 stride의 리샘플링 곡선을 캘리브레이션 누적기에 더함, 8회 채워지면 `_FinishCalibration` 호출 |
| 1055~1073 | `_CurveStats` | 위상축 방향 평균/표준편차 계산 |
| 1078~1182 | `_FinishCalibration` | ① 자신의 WALK 곡선 확정+빈bin보간 → ② THIGH_GYROMAG로 원형 상관 최대화 위상시프트 탐색 → ③ feature별 sign(상관부호)/scale(std비)/offset(평균차) 계산 → USB로 계수 로그 출력 |

### 2.13 z-score 뱅크 분류 (1184~1303줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 1192~1263 | `_ClassifyStride` | 5모드 각각에 대해 4feature 가중 z 계산 → 최소(best)/차소(second) 탐색 → **IDLE z 계산 및 거부권 판정** → tanh 마진 압축 → 확신+비전이 시에만 투표 |
| 1272~1303 | `_ComputeIdleZ` | 위상무관 평탄 템플릿(`kRcIdleMean/Std`) 대비 동일한 가중 z 계산 |

### 2.14 투표/버튼/LED/USB (1305~1417줄)

| 줄 범위 | 함수 | 내용 |
|---|---|---|
| 1305~1310 | `_PushVote` | 순환버퍼에 이번 stride 판정 push |
| 1313~1331 | `_UpdateModeFromVotes` | `VOTE_WINDOW`(3)개 중 `VOTE_MAJORITY`(2)개 이상이면 `s_bank_mode` 전환 |
| 1335~1356 | `_HandleButtonInput` | BTN1=캘리브레이션 시작, BTN2=투표/모드 리셋, BTN3=민감도 순환 |
| 1358~1378 | `_UpdateLedIndicators` | LED1 heartbeat, LED2 캘리브레이션 상태, LED3 모드 표시 |
| 1380~1395 | `_UpdateUsbDebug` | 300ms마다 텍스트 로그(모드/z값들/마진/theta/yaw/캘리브레이션여부) |
| 1397~1417 | `_UpdateStreamData` | EMG 좌우 비대칭 계산(진단용) + 10필드 구조체 채워 0xF4로 전송 |

---

## 3. 핵심 로직·이론·수식

### 3.1 MHE(Maximum Hip Extension) 이벤트 검출 — stride 경계

**출처**: Qian, Y. et al. (2022), *"Predictive Locomotion Mode Recognition and Accurate Gait Phase Estimation for Hip Exoskeleton on Various Terrains,"* IEEE RA-L 7(3):6439-6446, Sec.II-C-2.

발-접촉 센서(FSR) 없이 고관절 센서만으로 안정적인 stride 이벤트를 얻기 위해, heel-strike 대신 **고관절 최대 신전(허벅지각 국소 최솟값)** 을 stride 경계로 쓴다. 논문에서 이 방식이 무릎 위 센서만으로도 heel-strike보다 안정적으로 검출 가능한 이벤트로 제시된다.

구현(855~876줄) — 디바운스 포함 최솟값 검출:

```
delta = θ_thigh_R(t) - θ_thigh_R(t-1)

delta < 0                        → falling = true (신전 진행 중)
delta > 0 and falling            → rebound += delta
    rebound >= MHE_MIN_SWING_DEG(4°) and (t - t_last_MHE) >= REFRACTORY(400ms)
        → MHE 이벤트 확정, rebound=0, falling=false
```

**왜 좌우 평균이 아니라 오른쪽 단독인가** (4장, 109~118줄): 좌우 허벅지각은 약 180° 위상차다. 평균을 내면 기본 주파수 성분이 상쇄되어 검출 주기가 stride가 아니라 step(반 stride)이 되고 진폭도 작아져 디바운스가 불안정해진다. 또한 `ref_curve_tables.h`의 기준곡선이 Camargo의 `gcRight`(오른쪽 heel-strike) 기준 1 stride이므로, 온라인 위상 원점도 오른쪽 다리로 맞춰야 정합한다.

### 3.2 사후 위상 리샘플링 — 왜 causal 추정을 버렸는가 (5장, 120~161줄, 함수 932~961줄)

**문제였던 구버전**: `theta(t) = (t - t_MHE) / stride_period_직전`으로 매 ms마다 즉시 위상을 추정해 bin에 누적. 이건 두 가지 구조적 결함이 있었다.
1. 모드 전환 stride에서 가장 심하게 왜곡(WALK→RAMP_UP처럼 주기가 늘어나면 실제보다 위상이 앞서감).
2. stride가 예상보다 길어지면 θ가 1.0에 먼저 도달해 후반부 데이터를 통째로 유실.

**대안으로 검토했으나 채택 안 한 것**: HZD식 phase-portrait(예: `atan2(-thigh_velocity, thigh_angle-center)`) 위상변수. 이건 stride 길이를 몰라도 인과적으로 위상을 알 수 있어 이론상 매력적이지만, `ref_curve_tables.h`의 기준곡선 자체가 Camargo의 **%GC(percent gait cycle, gcRight heel-strike 시간정규화)** 로 만들어졌기 때문에 좌표계가 다르다. phase-portrait 각도를 %GC 인덱스에 그대로 대입하면 `IMU_thigh_TILT`를 폐기했던 것과 같은 종류의 오류(서로 다른 좌표계로 만든 템플릿과 신호를 억지로 맞춤)가 재발한다.

**채택한 해법 — 사후 리샘플링**:

```
[stride 진행 중]  _AccumulateRawSample(): 10ms 창평균으로 raw_buf 에 시간축 그대로 누적
                  (위상 추정 없음 — causal-estimation-free)

[MHE 발생 시점]   stride_ms 실측됨 → _ResamplePhaseCurve(stride_ms):
    for i = 0..100:
        t_target = (i/100) * stride_ms
        pos      = t_target/10 - 0.5        (raw_buf[j] 대표시각 = 창중앙 (j+0.5)*10)
        j0       = clamp(floor(pos), 0, raw_count-2)
        frac     = clamp(pos - j0, 0, 1)
        curve[i] = raw_buf[j0] + (raw_buf[j0+1] - raw_buf[j0]) * frac   ← 선형보간
```

이는 `build_h_d_reference_curves.m`(기준곡선을 만든 오프라인 스크립트)이 실제 stride 경계를 알고 나서 쓰는 `interp1(thetaImuU, signal, thetaGrid, 'linear')`와 **동일한 연산**을 온라인에서 재현한 것이다. 레이턴시는 늘지 않는다 — `_ClassifyStride()`는 원래도 MHE 시점에만 호출되는 1-stride 지연 구조이기 때문이다.

Python으로 독립 재현해 검증한 결과(전환 stride, WALK 1100ms→RAMP_UP 1350ms 가정, θ=0.65 피크 신호):

| 방식 | 추정된 피크 위치 | 오차 |
|---|---|---|
| 구버전(causal) | θ=0.790 | **0.140** |
| 신버전(사후 리샘플링) | θ=0.650 | **0.000** |

### 3.3 z-score 뱅크 — 모드 판정 핵심 수식 (1192~1263줄)

각 모드 m(WALK/STAIRS_UP/STAIRS_DOWN/RAMP_UP/RAMP_DOWN), feature f에 대해:

```
x_cal(i) = sign_f · scale_f · x(i) + offset_f              (도메인 보정, 3.4절)

d(i)     = ( x_cal(i) - kRcMean[m][f][j] ) / kRcStd[m][f][j]     j = (i + phase_shift) mod 101

z_f^m    = sqrt( mean_i[ d(i)^2 ] )                          (한 feature의 RMS z)

z^m      = sqrt( Σ_f w_f · (z_f^m)^2  /  Σ_f w_f )           (가중 결합, w_IMU=1.0, w_EMG=0.8)
```

`i`는 리샘플링된 위상 bin, `s_obs_cnt[i]==0`인 bin(리샘플링 실패)은 건너뛴다. **z^m이 가장 작은 모드가 그 stride의 후보(best)**, 그 다음으로 작은 게 second다.

### 3.4 런타임 도메인 보정 — affine 변환 자동 추정 (3장, 88~107줄, 함수 1078~1182줄)

기준곡선은 Camargo 센서 단위(gyro=rad/s, accel=g, EMG=mV)로 만들어졌고 H10(gyro=deg/s)·우리 EMG 전압과 물리적으로 다르다. 위상 원점도 다르다(기준곡선=heel-strike, 우리=MHE). 이를 BTN1 캘리브레이션(평지보행 8 stride)에서 자동 추정한다.

**① 위상 offset** — 원형 교차상관 최대화 (1109~1122줄):

```
best_shift = argmax_k  Σ_i [ own_THIGH_GYROMAG(i) - mean ] · [ ref_WALK(i+k mod 101) - mean ]
```

가장 깨끗한 파형인 THIGH_GYROMAG 하나로만 시프트를 찾고, 모든 feature가 이 값을 공유한다(위상 원점 차이는 feature에 무관하므로).

**② sign/scale/offset** — feature별 선형회귀에 준하는 계수 (1138~1157줄):

```
sign_f   = sign( Σ_i [own(i)-own_mean]·[ref(i+shift)-ref_mean] )     좌표계 반전 흡수
scale_f  = std(ref_WALK) / std(own_WALK)                             단위 스케일 흡수
offset_f = mean(ref_WALK) - sign_f · scale_f · mean(own_WALK)        DC 오프셋 흡수
```

이는 사실상 `own`을 `ref`에 맞추는 **1차원 어파인 변환**(y = a·x + b, 부호까지 포함)을 두 곡선의 평균·표준편차·상관부호만으로 닫힌 형태로 구하는 것이다.

### 3.5 IDLE 거부권 — 총분산의 법칙 (2.5절 층2.5, 1265~1303줄)

Camargo의 `idle` 라벨은 정지가 아니라 "전이/재배치" 구간이라는 사실이 실측으로 확인됐다(별도 진단: `diag_stand_curves2.m`). 이 구간의 위상축 정보량(flatness)이 1.7~7.1%(보행모드는 11~59%)로 사실상 0이므로, **총분산의 법칙**으로 위상 무관 스칼라 템플릿을 만든다:

```
Var_total = E_theta[ Var(theta) ]     +     Var_theta[ E(theta) ]
              (위상내 분산, within)          (위상간 분산 = 파형, between)

kRcIdleMean = mean_theta( r^STAND(theta) )
kRcIdleStd  = sqrt( within + between )
```

온라인에서는 이 평탄 템플릿 대비 `z_idle`을 뱅크와 동일한 수식(3.3절)으로 계산한다. 보행 중이면 `x(θ)`가 크게 출렁여 평탄 템플릿에서 멀어지므로 `z_idle`이 크고, 전이 구간이면 `x(θ)`가 평탄해 `z_idle`이 작다 — **"파형이 존재하는가" 자체가 신호**가 된다.

```
s_in_transition = (z_idle < z_best)
투표 조건       = (margin >= MODE_CONFIRM_TH) AND NOT s_in_transition
```

IDLE은 판별력이 약해(d_subj 0.68~1.14) 모드 argmin 경쟁자로 쓰면 정상 보행 중 오검출이 잦다. 그래서 모드를 "대체"하지 않고 지형 투표를 "막기만" 하는 거부권으로 한정했다.

### 3.6 tanh 마진 압축 + 다수결 히스테리시스 (1248~1262, 1305~1331줄)

**출처(패턴)**: `Terrain_Intent_Classifier.c`(구버전 3-class 지형분류기)의 fuzzy score 방식을 5-way argmin에 맞게 확장. 다수결(majority filter) 개념은 Qian et al.(2022)의 순간 오분류 스무딩 아이디어를 재구현.

```
margin_raw = (z_second - z_best) / z_best
margin     = tanh( sensitivity · margin_raw )         sensitivity ∈ {0.7, 1.0, 1.4} (BTN3)

margin >= 0.35 이고 IDLE 거부권 미발동 시:
    투표버퍼(길이 VOTE_WINDOW=3)에 best 모드 push
    3개 중 2개(VOTE_MAJORITY) 이상 동일 모드면 s_bank_mode 전환, 아니면 직전 상태 유지
```

margin이 임계 미만(완충지대)이면 아예 투표하지 않아 직전 다수결 상태가 유지된다 — 경계 지형에서의 flicker 방지.

**전환 지연에 대해**: `VOTE_WINDOW=5`/`VOTE_MAJORITY=3`(원래 값, 2-class용 `Terrain_Intent_Classifier.c`에서 재튜닝 없이 이식)에서는 확신 있는 stride가 매번 이어져도 최소 3 stride(~3~4초) 지연이 있었고, 전환 초반 stride가 margin 미달(완충지대)이나 IDLE 거부권에 걸려 투표를 건너뛰면 5~10초까지 늘어날 수 있었다. **5-way 경쟁에 맞춰 `VOTE_WINDOW=3`/`VOTE_MAJORITY=2`로 완화**했다 — 확신 있는 stride 2개 연속(~2~2.5초)이면 전환된다. 경계 지형 flicker 방지는 이 다수결 대신 margin 임계(`MODE_CONFIRM_TH`)와 IDLE 거부권이 1차 방어선을 맡으므로 완화의 실질 위험은 낮다. STAND 게이트(즉시 반응)와 TURN 검출기(800ms 창, 투표 미사용)는 이 지연 자체가 없다 — z-score 뱅크의 모드 간 전환에만 해당하는 특성이다.

### 3.7 TURN 검출기 — 시간창 heading 변화량 (963~1016줄)

기준곡선으로는 TURN을 못 가른다(TURN vs WALK, 41 feature 전부에서 Cohen's d ≤ 0.80 — 회전은 yaw 현상인데 기준곡선의 IMU feature는 sagittal tilt와 gyro *크기*뿐이라 방향 정보가 없음). 대신 H10이 제공하는(Camargo에는 없는) 좌우 `HipImuTransverseYaw`를 쓴다.

```
yaw(t)      = 0.5 · (leftHipImuTransverseYaw + rightHipImuTransverseYaw)
Δyaw(t)     = wrap180( yaw(t) - yaw(t - 800ms) )       800ms 링버퍼, 10ms 샘플

|Δyaw| >= 30°  →  TURN 진입
|Δyaw| <= 15°  →  TURN 해제                             (히스테리시스로 경계 flicker 방지)
```

절대 yaw 대신 시간창 **변화량**만 쓰는 이유는 자기/적분 드리프트 내성을 위해서다.

### 3.8 EMG RMS 파이프라인 (770~816줄)

```
v_centered(t) = v_raw(t) - bias          bias 는 ACTIVE 진입 후 1000ms 평균으로 추정
RMS(t)        = sqrt( mean_{150ms}[ v_centered(t)^2 ] )     150ms boxcar(순환버퍼 이동합)
```

150ms 윈도우는 `ref_curve_tables.h`를 만들 때 쓴 Camargo EMG envelope 윈도우(`ENV_WINDOW_S=0.15`)와 동일하게 맞춰, 온라인 RMS와 기준곡선의 정의를 일치시켰다.

### 3.9 참고문헌

- Qian, Y. et al. (2022) *"Predictive Locomotion Mode Recognition and Accurate Gait Phase Estimation for Hip Exoskeleton on Various Terrains,"* IEEE Robotics and Automation Letters, 7(3):6439-6446. — MHE 이벤트, majority filter 아이디어의 출처.
- Camargo, J. et al. (2021) *"A comprehensive, open-source dataset of lower limb biomechanics in multiple conditions,"* J. Biomech. 119. — 기준곡선(`ref_curve_tables.h`)의 원본 데이터셋(AB06~AB12, 7명).

### 3.10 "h_d" → "Rc(Reference Curve)" 명명 정정에 대해

이전 버전은 `h_d^m(θ)` 표기를 썼는데, 이는 HZD(Hybrid Zero Dynamics, Grizzle/Westervelt/Ames 계열) 이족보행 제어이론에서 virtual constraint의 "desired output"을 가리키는 특정 용어다(`y = h(x) - h_d(θ(x)) → 0`으로 zero dynamics manifold를 만드는 제어기 대상). 이 파일이 실제로 하는 일은 그런 제어 대상이 아니라 **순수 통계적 템플릿 매칭**(z-score 최근접 분류)이며, virtual constraint도 이를 추종하는 토크 제어 루프도 없다(`XM_SetControlMode(XM_CTRL_MONITOR)`). 그래서 이번 세션에서 `h_d`/`HD_`/`kHd` 계열 명칭을 전부 `r`/`RC_`/`kRc`(Reference Curve)로 정정했다. 원본 `h_d_population.mat` 파일명과 내부 필드 스키마는 그대로 두었다(이걸 만든 기존 스크립트들과의 호환 유지 목적).
