# Squat_EMG_Assist.c — 실습 기반 수정 이력

> 작성 기준: 2026-06-10  
> 대상 파일: `XM_Apps/User_Algorithm/Squat_EMG_Assist.c`  
> 시간은 KST 기준

---

## 1. 실습 전 준비 — FSR 제거 결정 (전 세션, ~2026-06-10 02:00 KST)

실습 당일 이전에, 실제 하드웨어에 FSR 센서를 연결하지 않은 상태에서 실습하기로 결정.

| # | 문제 상황 | 해결 방안 |
|---|---|---|
| 1 | FSR 센서 미연결인데 코드에 FSR 캘 상태머신·변수·CDC 채널이 남아 있어 동작 안 됨 | FSR 관련 코드 전체 제거 (상수·enum·static 변수·public 변수·함수·호출부·CDC 채널 모두 삭제). CDC 채널 14개 → 8개로 축소 |
| 2 | `_ApplyTorque` 안전 게이트에 `fsr_cal_ready == 1` 조건이 포함돼 있어 FSR 캘 없으면 토크 출력 불가 | 안전 게이트 조건에서 `fsr_cal_ready` 제거 (`squat_control_ON == 1 && H10 ASSIST 모드` 만 유지) |
| 3 | FSR 제거 후 `User_Setup`에서 DIO_3~6 ADC 전환 코드 및 `_ResetFsrCal()` 잔류 | `User_Setup`에서 FSR 관련 핀 설정 및 초기화 호출 모두 제거 |

---

## 2. 실습 중 첫 번째 문제 묶음 (2026-06-10 약 16:40 KST)

실제 하드웨어 착용 후 실습 도중 발생한 3가지 문제.

| # | 문제 상황 | 원인 | 해결 방안 |
|---|---|---|---|
| 1 | Live Expressions에 `fsr_lh_load` 등 FSR 변수가 아직 남아 있고 `fsr_cal_zero_done` 등이 표시됨 | 이전 세션에서 코드는 제거했으나 Live Expressions 목록을 수동으로 업데이트하지 않음 | Live Expressions에서 `fsr_*` 변수 전부 수동 삭제. 유효 변수 목록 재정리 |
| 2 | 루프 돌아가는 주기가 너무 길게 느껴짐 | FSR 캘 상태머신이 완료 전까지 루프 내 조건 분기가 EMG 처리를 막고 있었던 것 | FSR 제거로 해당 분기 자체 소멸. 1ms 루프 정상화 |
| 3 | 엔코더에 잡히는 각도값이 터무니없이 큼 (수백~수천 도) | `leftHipMotorAngle`은 전원 ON 이후 **누적 각도** → offset 미설정 시 부팅 시점의 누적값이 그대로 노출됨. 기존 코드에 gear ratio 나눗셈이 있었으나 이것이 원인이 아님 | gear ratio 관련 변수·연산 완전 제거. `control_angle = sign × (raw − offset)` 구조로 단순화. 기립 자세에서 `encoder_offset_lh/rh_deg`에 raw 값 입력하여 0° 기준점 설정하도록 안내 |

---

## 3. 빌드 에러 — tasks.c 파일 손상 (2026-06-10 약 17:06 KST)

```
multiple definition of `squat_control_ON';
Middlewares/.../tasks.c:274: first defined here
```

| 문제 상황 | 원인 | 해결 방안 |
|---|---|---|
| 빌드 시 `squat_control_ON`, `compensation_ON` 등 알고리즘 변수가 `tasks.c`와 중복 정의 오류. 동시에 `xTaskGetSchedulerState` 등 FreeRTOS 함수 undefined reference 발생 | 실수로 `Middlewares/Third_Party/FreeRTOS/Source/tasks.c` (워크스페이스 경로)를 `Squat_EMG_Assist.c` 내용으로 덮어씀. 라인 번호까지 일치하는 것으로 확인 | `D:\HARfinal2\HARfinal\Middlewares\Third_Party\FreeRTOS\Source\tasks.c` (레포 원본)를 워크스페이스 경로로 복사하여 복구. 해시값으로 일치 확인 후 빌드 재실행 |

---

## 4. CSV 실측 데이터 기반 EMG/엔코더/토크 3가지 수정 (2026-06-10 약 17:43 KST)

실측 데이터 파일 `xm10_4ch_20260609_210543.csv` 분석 결과를 반영.

| # | 문제 상황 | 원인 분석 | 해결 방안 | 변경 내용 |
|---|---|---|---|---|
| 1 | `emg_rh_envelope_v`, `emg_lh_envelope_v` 값이 `0.0n` 수준으로 매우 작게 나옴. EMG 비례 보조 토크도 거의 0 | 코드 기본 bias `EMG_DEFAULT_BIAS_V = 1.65V`인데 실측 baseline은 **1.604V**. 46mV 오차가 rectification 후 상시 오프셋으로 고착. `EMG_DEFAULT_FULL_SCALE_V = 1.0V`로 나누면 norm ≈ 0.02~0.05 | bias 기본값을 실측에 맞게 낮추고, full_scale도 캘 없이 norm이 유효한 범위에 들어오도록 하향 | `EMG_DEFAULT_BIAS_V`: 1.65 → **1.60** V  `EMG_DEFAULT_FULL_SCALE_V`: 1.000 → **0.200** V |
| 2 | 최대 토크 2.5Nm로는 ASCENDING 구간 보조가 부족함. 일어나는 시간이 짧아 EMG 비례 방식으로는 충분한 토크를 낼 수 없음 | ASCENDING 구간의 동작 시간이 짧아 EMG norm이 올라오기 전에 구간이 끝남 | ASCENDING 구간은 `squat_max_torque_nm` 고정 적용으로 변경. 허용 최대치도 상향 | `squat_max_torque_nm`: 0.3 → **0.5** Nm (초기값)  `assist_torque_limit_nm`: 0.5 → **2.5** Nm  `HARD_MAX_ASSIST_TORQUE_NM`: 2.5 → **2.5** Nm (이 시점 유지) |
| 3 | 기립 자세에서 엔코더 `control_angle`이 0°가 아니라 ~30° 값을 보임. 내려가는 방향이 `+` | 기립 시 raw 누적 각도가 30° 수준. offset 기본값 0.0f로 보정 안 됨 | encoder_offset 기본값을 30°로 설정. FSM velocity 부호 확인 — 하강=+ 이므로 상승 시 음수 → `BOTTOM→ASCENDING` 조건 `avg_velocity < -0.05f` 그대로 유지 | `encoder_offset_lh_deg`: 0.0 → **30.0** f  `encoder_offset_rh_deg`: 0.0 → **30.0** f |

---

## 5. 토크 출력 로직 재수정 (2026-06-10 약 17:57 KST)

이전 수정(4번)에서 ASCENDING만 최대 토크로 바꿨는데, DESCENDING 구간도 포함해야 한다는 요청.

| 문제 상황 | 해결 방안 | 최종 토크 로직 |
|---|---|---|
| 기존에는 ASCENDING에만 토크를 적용했으나, DESCENDING 구간(내려가면서 버티는 구간)에도 EMG 비례 보조가 필요 | DESCENDING: `emg_norm × squat_max_torque_nm` (EMG 비례). ASCENDING: `squat_max_torque_nm` 고정 적용. BOTTOM/STAND/RETURN: base = 0 | **DESCENDING** → EMG 비례  **ASCENDING** → 고정 최대 토크  **그 외** → 0 (보상 토크만) |

---

## 6. 4가지 추가 수정 (2026-06-10 약 18:32 KST)

다음 실습을 위한 사전 수정 묶음.

| # | 문제 상황 | 해결 방안 | 변경 내용 |
|---|---|---|---|
| 1 | 최대 토크를 10Nm까지 사용 가능하다고 확인됨. 실습 중 단계적으로 올리며 실험 예정 | `HARD_MAX_ASSIST_TORQUE_NM` 상향. 기본값 6Nm로 설정 | `HARD_MAX_ASSIST_TORQUE_NM`: 2.5 → **10.0** Nm  `squat_max_torque_nm`: 0.5 → **6.0** Nm  `assist_torque_limit_nm`: 2.5 → **10.0** Nm |
| 2 | 보상 토크(compensation)가 너무 작아 착용자가 거의 느끼지 못하는 수준 (45°에서 약 0.14 Nm) | `gravity_mgl_nm`·`compensation_scale` 기본값 상향. 45°에서 **2.83 Nm** 수준으로 증가 | `gravity_mgl_nm`: 1.0 → **8.0** Nm  `compensation_scale`: 0.20 → **0.50** |
| 3 | Phase 경계에서 토크가 급격히 변화하여 착용자가 충격감을 느낌. EMG 기반 부드러운 전환 요청 | ① 토크 출력에 5Hz LPF ramp 적용 → 200ms에 걸쳐 부드럽게 전환  ② BOTTOM→ASCENDING 전환에 EMG 확정 조건 추가: avg EMG norm ≥ `emg_ascend_threshold`(기본 0.5)이면 velocity 조건 없이 전환 가능 | 신규 변수: `emg_ascend_threshold = 0.5f`  신규 상수: `TORQUE_RAMP_HZ = 5.0f`  신규 static: `s_torque_lh_ramp`, `s_torque_rh_ramp` |
| 4 | 수동 캘리브레이션(BTN1/BTN2)이 하드웨어 문제로 잘 되지 않아, 캘 없이도 EMG 비례 토크가 실용적인 값으로 나와야 함 | 0.3Hz 극저역 LPF로 DC baseline을 자동 추적 (`s_emg_auto_bias_v`). `EMG_CAL_IDLE` 상태에서 수동 bias 대신 자동 추적값을 사용. 1~2초 후 bias 수렴 → norm이 유효 범위로 진입 | 신규 상수: `EMG_AUTO_BIAS_LPF_HZ = 0.3f`  신규 static: `s_emg_auto_bias_v[2]`  `_ProcessEmgSignals` 내 bias 분기: CAL_DONE → 수동 bias / IDLE → 자동 bias |

---

## 변수 대조표 — 주요 수정 전후 비교

| 변수 / 상수 | 최초 값 | 현재 값 | 비고 |
|---|---|---|---|
| `HARD_MAX_ASSIST_TORQUE_NM` | 2.5 Nm | **10.0 Nm** | |
| `EMG_DEFAULT_BIAS_V` | 1.65 V | **1.60 V** | 실측 baseline 기반 |
| `EMG_DEFAULT_FULL_SCALE_V` | 1.000 V | **0.200 V** | 캘 없이 norm 유효 |
| `squat_max_torque_nm` | 0.3 Nm | **6.0 Nm** | Live Expr. 조정 가능 |
| `assist_torque_limit_nm` | 0.5 Nm | **10.0 Nm** | |
| `gravity_mgl_nm` | 1.0 Nm | **8.0 Nm** | 성인 70kg 기준 추정 |
| `compensation_scale` | 0.20 | **0.50** | |
| `encoder_offset_lh_deg` | 0.0° | **30.0°** | 기립 실측값 기준 |
| `encoder_offset_rh_deg` | 0.0° | **30.0°** | 기립 실측값 기준 |
| `emg_ascend_threshold` | (없음) | **0.5** | 신규. 0.0이면 비활성 |
| `TORQUE_RAMP_HZ` | (없음) | **5.0 Hz** | 신규. 토크 스무딩 |

---

## 토크 출력 로직 최종 구조

```
DESCENDING  →  base = emg_norm × squat_max_torque_nm  (EMG 비례)
ASCENDING   →  base = squat_max_torque_nm              (고정 최대)
그 외       →  base = 0                                (보상 토크만)

최종 토크 = clamp(base + compensation_torque, ±limit)
         → 5Hz LPF ramp (s_torque_lh/rh_ramp)
         → × torque_sign
         → XM_SetAssistTorqueLH/RH()
```

---

## Live Expressions 실습 시 조정 변수

| 변수명 | 현재 기본값 | 용도 |
|---|---|---|
| `squat_max_torque_nm` | 6.0 Nm | 실습 시 1.0 → 2.0 → ... → 10.0 단계적 증가 |
| `torque_sign` | 1.0 | 방향 반대이면 -1.0 으로 변경 |
| `encoder_offset_lh_deg` | 30.0° | 기립 자세에서 `left_encoder_angle_deg` 읽어 입력 |
| `encoder_offset_rh_deg` | 30.0° | 기립 자세에서 `right_encoder_angle_deg` 읽어 입력 |
| `emg_ascend_threshold` | 0.5 | BOTTOM→ASCENDING EMG 확정 임계. 0.0이면 비활성 |
| `gravity_mgl_nm` | 8.0 | 착용자 체중·링크 길이에 맞게 조정 |
| `compensation_scale` | 0.50 | 보상 토크 비율. 과하면 낮출 것 |
| `compensation_ON` | 1 | 0이면 보상 토크 끔 (안전 테스트 초기 권장) |
