# my code

IFAC 2026 Wearable Robot Challenge 과정에서 **직접 작성한 코드만** 모은 폴더다.
작업 사본이 5개로 흩어져 있던 것을 주제별로 통합했다. 외부 제공물(XM10 SDK,
HAL/CMSIS, FreeRTOS, Camargo 데이터셋, 데이터셋 제공 데모 스크립트)은 포함하지
않는다.

- 파일 99개 / 약 28 MB
- 각 파일이 어느 작업 사본에서 왔는지는 [`MANIFEST.txt`](MANIFEST.txt) 참고

```
my code/
├── terrain_classifier/      지형 7모드 분류기 (IFAC 본 과제)
├── minecraft_emg/           EMG 동작인식 -> 마인크래프트 조작
├── squat_assist/            스쿼트 보조 토크 제어
├── host_tools/              USB-CDC 수신·로깅·시각화 도구
└── modified_sdk_examples/   배포 예제를 개조한 것
```

---

## 1. terrain_classifier — 지형 7모드 분류기

Camargo 데이터셋으로 위상 기준곡선을 만들어, H10 외골격이 매 stride
**STAND / WALK / TURN / STAIRS_UP / STAIRS_DOWN / RAMP_UP / RAMP_DOWN**
7가지를 실시간 판정한다. FSR(족압센서) 미사용.

### firmware/ — 보드에 올라가는 코드

| 파일 | 역할 |
|---|---|
| `Locomotion_Mode_Classifier_v1.0.c` | 분류기 초판. stride 경계를 허벅지각 MHE(최대 고관절 신전)로 잡고, 체간 기울기에 `pelvicAngle`을 사용. margin 임계 0.35 |
| `Locomotion_Mode_Classifier_v1.2.c` | **개선판.** stride 경계를 `isRightFootContact` rising edge로 교체(기준곡선의 theta=0과 같은 물리 사건), 체간 기울기를 좌우 `HipImuSagittalPitch` 평균으로 교체, EMG 시간축(중심창↔trailing 75ms) 보상, Body Data 미설정 시 fail-closed, margin 임계 0.30 |
| `ref_curve_tables.h` | **자동 생성 파일.** 5모드 × 4 feature × 101 위상점 × (평균+표준편차) lookup table (15.8 KB, `.rodata`). AB11 제외 21명 pooled. 직접 수정하지 말고 생성기를 다시 돌릴 것 |
| `Terrain_Intent_Classifier.c` | 전신(前身). 정지/평지/오르막 3-class 판정기. 위 분류기가 이걸 7-class로 확장한 것 |

두 버전 모두 `XM_CTRL_MONITOR` 모드로만 동작하고 실제 토크는 0 Nm다.
3칸/2표 다수결(`VOTE_WINDOW=3`, `VOTE_MAJORITY=2`)은 두 버전에서 동일하다.

### analysis/matlab/ — 기준곡선 생성 파이프라인

실행 순서대로:

| 파일 | 역할 |
|---|---|
| `extract_all_subjects.m` | `D:\gait_data`에서 `AB##` 폴더를 자동 탐색해 41개 feature를 101 위상점으로 리샘플링. 피험자별/전체 평균곡선 + 뱅크 4-feature 원시 stride 저장 |
| `build_reference_bank_clean.m` | 위와 같되 AB11을 제외한 배포용 모집단 생성 |
| `export_ref_curve_tables_clean.m` | 생성된 `.mat`을 읽어 `ref_curve_tables.h` C 헤더로 내보냄 |
| `build_h_d_reference_curves.m` | 초판 생성기(AB06~12 7명 기준) |
| `extract_all12_subjects.m` | 12명으로 확장한 중간판 |
| `export_ref_curve_tables.m` | 초판 헤더 생성기 |
| `extract_transition_features.m` | 12개 실제 전환 구간에서 IMU+EMG feature의 Cohen's d 랭킹 계산 |
| `transition_conditions_AB06.m` | AB06 실데이터로 7모드 간 전환 판정 threshold 추출 |
| `turn_detection_emg_AB06.m` | 회전(turning)을 중둔근/박근/외복사근 3채널로 검출 가능한지 검증 |
| `run_all_subjects.m` | 여러 피험자에 대해 전환 feature 추출을 일괄 실행 |
| `diag_stand_curves.m`, `diag_stand_curves2.m` | STAND(idle) 각속도가 비정상적으로 큰 원인 진단. 2번이 라벨 run 단위로 수정한 판 |
| `plot_h_d_reference_curves.m` | 생성된 기준곡선 검증용 플롯 |

### analysis/python/ — 검증 하네스

| 파일 | 역할 |
|---|---|
| `camargo_common.py` | 공용 유틸. 펌웨어의 `_CurveStats` / `_FinishCalibration` / `_ClassifyStride`를 파이썬으로 1:1 재현해 오프라인에서 같은 계산을 돌린다 |
| `validate_1_holdout_accuracy.py` | 학습에 안 쓴 피험자의 개별 stride 분류 정확도 (held-out) |
| `validate_2_template_stability.py` | 표본을 늘렸을 때 기준곡선이 얼마나 흔들리는지 |
| `validate_3_reverify_claims.py` | 설계 근거 4건(TURN 분리불가, EMG 근육 선정, shank/foot 대체손실, THIGH_TILT flatness) 재검증 |
| `validate_4_calibration_robustness.py` | 피험자별 캘리브레이션 계수(sign/scale/offset/위상)의 이상치 탐지 |
| `validate_5_loso_cv.py` | Leave-One-Subject-Out 교차검증 |
| `validate_12_threshold_sweep.py` | margin 임계값 스윕. fit/holdout을 나눠 과적합을 막는다 |
| `analyze_muscle_ranking.py` | EMG 11근육 전체 판별력 랭킹 + 1위 근육이 어느 모드쌍에서 이기는지 |
| `analyze_locomotion_bank.py` | 모드쌍별 분리도 + EMG 근육 greedy 선택 |
| `analyze_feature_separability.py` | 전환별 feature 랭킹, shank/foot 대체 손실 정량화 |
| `analyze_idle_as_class.py` | idle을 STAND가 아닌 별도 전이 클래스로 쓸 수 있는지 |
| `extend_C_anthropometric_normalization.py` | 키/체중으로 개인차를 정규화하면 정확도가 오르는지 |
| `check_c_structure.py` | 분류기 C 코드의 괄호 균형·프로토타입·TSM 콜백·미사용 static 검사 |
| `verify_resampling.py` | `_ResamplePhaseCurve()` 수식을 파이썬으로 재현해 빌드 없이 검증 |
| `count_contract.py` | 평가 trial 수·전환 수를 로더에서 실측 재계산 |

> `validate_12_threshold_sweep.py`와 `count_contract.py`는 외부에서 받은
> 연속보행 scorer(`validate_6` / `validate_9`)를 import하므로 단독 실행되지
> 않는다. 그 하네스는 외부 제공물이라 이 폴더에 포함하지 않았다.

### docs/

| 파일 | 역할 |
|---|---|
| `Locomotion_Mode_Classifier.md` | 아키텍처 설계 문서. 왜 7모드를 한 뱅크로 돌리지 않고 3층(STAND 게이트 / TURN yaw / 5모드 z-score 뱅크)으로 나눴는지 |
| `Locomotion_Mode_Classifier_Quiz_Guide.md` | 구두질문 대비 Q&A |
| `IMPROVEMENT_REPORT_v1.2.md` | v1.0 → v1.2 변경 근거와 전후 수치 |
| `AB11_AB28_outlier_analysis.md` | 이상치 피험자 2명의 원인 분석(AB11 전극 이상 / AB28 연령) |

---

## 2. minecraft_emg — EMG 동작인식 → 마인크래프트 조작

H10의 EMG + IMU로 서기/걷기/뛰기/회전을 판정해 마인크래프트 캐릭터를 조작한다.

### motion_classifier/ — 동작 분류 파이프라인 (완성)

| 파일 | 역할 |
|---|---|
| `h10_train_continuous_motion_classifier.py` | 연속 동작 분류기 학습 |
| `h10_stream_motion_model.py` | 스트리밍(실시간) 추론 모델 구현 |
| `h10_stream_motion_model.json` | **학습 완료된 모델 파라미터** (25 MB) |
| `h10_motion_segment_model.py` | 구간 단위 분류 모델 |
| `h10_live_motion_classifier.py` | 실시간 분류 실행 엔트리 |
| `h10_evaluate_mixed_motion.py` | 여러 동작이 섞인 구간에 대한 평가 |
| `mc_game_output.py` | 판정 결과를 게임 입력(키보드/마우스/RCON)으로 변환 |
| `mc_tuning.json` | 판정 임계값 등 튜닝 파라미터 |
| `audit/` | 평가 결과 JSON 3건 (locked / mixed / game output 연동) |
| `README_WINDOWS_KO.txt`, `TRAINING_README_KO.txt` | 실행·학습 절차 |
| `requirements_h10_motion_classifier.txt` | 파이썬 의존성 |

### firmware/ — 보드 쪽 설정

| 파일 | 역할 |
|---|---|
| `Final_EMG_minecraft.c` | 배포 예제 `Final_EMG.c`를 게임용 센서 매핑으로 개조. PF3=오른 종아리(MG), PF4=오른 허벅지(RF), PF5=팔(마우스 클릭 전용, 고관절 토크원으로 절대 선택 금지), PF6=예비 |
| `minecraft_motion_config.h` | 동작 판정 설정 |
| `minecraft_training_config.h` | 학습 데이터 수집 설정 |
| `minecraft_training_schema.h` | 학습 데이터 스키마 정의 |

### bridge/ — 별도 설계의 브리지 (미완성)

| 파일 | 역할 |
|---|---|
| `cdc_minecraft_bridge.py` | USB-CDC로 9채널을 받아 키보드 W / 마우스 / RCON으로 내보내는 브리지. **의존 모듈 `mc_features.py`, `mc_detectors.py`가 없어 현재 실행되지 않는다** |
| `GAME_OUTPUT_SPEC.md` | 게임 출력 모듈 규격 |
| `mc_tuning.json` | 브리지용 튜닝 값 (위 `motion_classifier/`의 것과 내용이 다름) |

---

## 3. squat_assist — 스쿼트 보조 토크 제어

| 경로 | 역할 |
|---|---|
| `firmware/Squat_EMG_Assist.c`, `Squat_EMG_Assist2.c` | EMG 기반 스쿼트 보조. 2가 후속판 |
| `firmware/Squat_FSR_Assist.c` | FSR(족압) 기반 스쿼트 보조 |
| `docs/Squat_FSR_Assist_Report.md` | FSR 방식 결과 보고 |
| `docs/Squat_EMG_Assist_functions.md` | EMG 방식 함수별 설명 |
| `docs/live_expression_sqemg.txt` | 디버거 Live Expression 관찰용 변수 목록 |
| `sim/sim_squat_v2.py`, `sim_squat_v2_5versions.py` | 토크 프로파일 시뮬레이션. 5개 버전 비교판 포함 |
| `sim/test_squat_fsm.py` | 스쿼트 상태기계 테스트 |
| `sim/squat_arch_viz.py`, `squat_arch_viz2.py` | 제어 구조 도식화 |
| `sim/squat_emg_viz.py` | EMG 신호 시각화 |
| `sim/squat_fsm_debug_viz.py` | 상태기계 전이 디버그 시각화 |

---

## 4. host_tools — USB-CDC 호스트 도구

| 경로 | 역할 |
|---|---|
| `cdc/cdc.py` | PhAI V2.2 프로토콜 실시간 수신 + 그래프 + CSV 저장 GUI |
| `cdc/cdc_4ch_live_monitor.py` (+`.md`) | 4채널 실시간 모니터 |
| `cdc/cdc_8ch_live_monitor.py` | 8채널 판 |
| `cdc/cdc_h10_live_monitor_verified.py` | H10 실기 검증을 거친 모니터 |
| `cdc/cdc_selective_logger.py` (+`.md`) | 원하는 채널만 골라 로깅 |
| `cdc/cdc_bilateral_gait_event_monitor.py` | 좌우 보행 이벤트(heel-strike 등) 실시간 검출 |
| `cdc/cdc_bilateral_gait_event_monitor_stairs.py` | 위의 계단 대응판 |
| `cdc/cdc_bilateral_gait_event_offline.py` (+`plan.md`) | 저장된 CSV로 오프라인 재검출 |
| `cdc/cdc_emg_torque_monitor.py` | EMG → 토크 변환 모니터 |
| `cdc/cdc_csv_reviewer.py` | 로깅된 CSV 검토 |
| `cdc/cdc_phai_receiver.py` | PhAI 프로토콜 수신기 |
| `msc/data_decoder_xm10.py`, `_v2.py` | MSC(대용량저장) 모드로 기록된 바이너리 로그 디코더 |
| `legacy/cdc_serial_data_logging.py` | 초기 시리얼 로깅 스크립트 |
| `viz/user_algorithm_visualizer.py` | 알고리즘 동작 시각화 |
| `viz/xm10_system_visualizer.py` | 시스템 전체 구조 시각화 |
| `viz/analyze_phase_glitch.py` | 위상 추정 글리치 분석 |
| `viz/merge_3plots.py` | 플롯 3개 합성 |
| `viz/geminiver.py` | 보조 스크립트 |

---

## 5. modified_sdk_examples — 배포 예제 개조본

배포된 예제를 베이스로 수정한 파일들이다. 원본 예제 자체는 포함하지 않았다.

| 파일 | 역할 |
|---|---|
| `Final_FSR_Fuzzy_Logic_extended.c` | FSR 퍼지로직 예제를 확장 (원본 29 KB → 46 KB) |
| `Final_EMG_H10_12ch_verified.c` | EMG 예제를 H10 12채널로 검증·확장 |
| `user_app_modified.c` | SDK 템플릿 `user_app.c`(4 KB)를 44 KB로 확장한 작업판 |
| `final_user_app.c` | 위의 정리판 |
| `user_stair_climb.c` | 계단 보행 보조 시도 |
| `TA_Body_Parameter_ID.c` | 사용자 신체 파라미터 식별 |

---

## 통합 출처

흩어져 있던 작업 사본에서 모은 결과다. 사본마다 **다른 곳엔 없는 고유 작업물**이
있었기 때문에 단순 중복 제거가 아니라 합집합을 취했다.

| 원본 위치 | 가져온 파일 수 | 주로 들어있던 것 |
|---|---|---|
| `D:\ifac_wearable_contest` | 52 | Camargo 파이프라인, 분류기 v1.0, CDC 도구 |
| `D:\Extension_Module-ghlee_03` | 20 | 마인크래프트 동작분류 완성본, 게임용 펌웨어 설정 |
| `D:\HARfinal` | 13 | 전신 분류기, 스쿼트 시뮬/시각화 |
| `D:\HARfinal2\HARfinal` | 8 | 스쿼트 보조 펌웨어 |
| `verify\...clean8` | 4 | 분류기 v1.2, 개선 보고서 |

같은 이름인데 내용이 다른 파일 19건이 있었고, 그중 의미 있게 갈린 것은 양쪽을
다 보존하거나(분류기 v1.0 / v1.2) 더 확장된 쪽을 택했다(`Final_*` 개조본).

## 포함하지 않은 것

- **XM10 SDK / HAL / CMSIS / FreeRTOS / FATFS** — 외부 제공 펌웨어 기반
- **Camargo 데이터셋** (`D:\gait_data`, 27 GB) — 외부 연구 데이터
- **데이터셋 제공 데모** `PLOT_TRIAL.m`, `STRIDES.m`, `RUN_OSIM.m`, `PLOTS.m`,
  `BATCHRUN.m`, `init.m` — Jonathan Camargo 작성
- **외부에서 받은 연속보행 scorer** (`validate_6` / `validate_9` 등)
- **생성 산출물** — `h_d_population*.mat`, `stride_bank*.mat` (수 MB~44 MB,
  `analysis/matlab`의 스크립트로 재생성 가능)
- **로그 데이터** — 측정 CSV, 플롯 PNG, 빌드 출력(`Debug/`), `__pycache__`

## 데이터 출처

`terrain_classifier`의 기준곡선과 모든 검증 수치는 아래 공개 데이터셋에서 나왔다.

> Jonathan Camargo, Aditya Ramanathan, Will Flanagan, Aaron Young (2021),
> *A comprehensive, open-source dataset of lower limb biomechanics in multiple
> conditions of stairs, ramps, and level-ground ambulation and transitions*,
> Journal of Biomechanics 119:110320.
> https://doi.org/10.1016/j.jbiomech.2021.110320
