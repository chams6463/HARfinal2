# Live Expression 변수 가이드

> 기준 코드: `XM_Apps/User_Algorithm/Squat_EMG_Assist.c` (현재 최신 버전)  
> 작성일: 2026-06-10

---

## 읽기 전에 — 구조적 이해

Live Expressions에 보이는 변수들은 모두 **public 변수**로, 루프 맨 끝 `_UpdatePublicSignals()`에서 내부 static 변수(`s_*`)로부터 복사되어 갱신된다.  
즉 실제 계산은 `s_*` 변수 기반으로 이루어지고, 여기서 보이는 값은 **1ms마다 찍힌 스냅샷**이다.  
→ Live Expressions의 값이 이상하면 내부 `s_*` 변수를 Expressions 창에 직접 입력해서 추가 확인 가능하다.

---

## 그룹 1 — 보상 토크 튜닝 파라미터

### `gravity_mgl_nm`
- **의미**: 중력 보상 모델에서 사용하는 `M × g × L` 값. 질량(kg) × 중력가속도(9.81 m/s²) × 링크 팔 길이(m)의 추정값.
- **역할**: 스쿼트 중 고관절 각도 θ에 비례한 중력 보상 토크 계산에 사용.
  ```
  left_gravity_torque_nm = gravity_mgl_nm × sin(left_control_angle_deg)
  ```
- **현재 기본값**: 8.0 Nm (성인 70kg 기준 추정)
- **다른 변수와의 관계**: `compensation_scale`과 곱해져서 `left_compensation_torque_nm`이 결정됨.
  - 기립(θ ≈ 0°) 시 → sin(0) = 0 → 보상 토크 0
  - 45° 스쿼트 시 → 8.0 × sin(45°) × 0.5 = **2.83 Nm**
  - 90° 스쿼트 시 → 8.0 × sin(90°) × 0.5 = **4.0 Nm**
- **관찰 시 유의점**:
  - 기립 자세에서 `left_compensation_torque_nm`이 0이 아니면 → `encoder_offset` 미설정이 원인
  - 값을 너무 키우면 보상 토크만으로도 착용자가 뒤로 밀릴 수 있음 → 처음엔 4.0 이하에서 시작 권장

---

### `compensation_scale`
- **의미**: 중력 보상 토크를 실제 몇 % 출력할지 결정하는 비율 (0.0 ~ 1.0).
- **역할**: `gravity_mgl_nm × sin(θ)`로 계산된 이론 보상 토크에 이 값을 곱해 최종 출력.
  ```
  left_compensation_torque_nm = compensation_scale × (gravity_torque + friction_torque)
  ```
- **현재 기본값**: 0.50 (50%)
- **다른 변수와의 관계**: `gravity_mgl_nm`과 함께 보상 토크 크기를 이중으로 조절하는 구조. 둘 중 어느 쪽을 건드려도 결과는 같지만, `gravity_mgl_nm`은 물리 모델 파라미터이고 `compensation_scale`은 "얼마나 믿고 쓸지"의 조절 손잡이.
- **관찰 시 유의점**:
  - 0.0으로 설정하면 중력/마찰 보상 토크가 완전히 꺼짐 (base 토크만 나옴). 안전 테스트 초기에 권장
  - 1.0은 모델이 정확하다고 가정할 때의 전출력. 모델 오차가 크면 토크가 노이즈처럼 진동할 수 있음

---

## 그룹 2 — CDC 스트림 제어

### `cdc_stream_enable`
- **의미**: USB CDC를 통해 Python Decoder로 실시간 신호 스트리밍을 켜고 끄는 스위치.
- **역할**: 1이면 `cdc_stream_period_ms` 주기로 8채널 float 데이터를 USB로 전송. 0이면 전송 안 함.
- **현재 기본값**: 1 (켜짐)
- **관찰 시 유의점**:
  - Python Decoder에 데이터가 안 들어오면 이 값이 1인지 먼저 확인
  - 디버거로 실시간 관찰 중이면 0으로 꺼도 된다 (USB 부하 줄임)

---

### `cdc_stream_period_ms`
- **의미**: CDC 스트림 전송 주기 (밀리초). 기본 10ms = 100 Hz 전송.
- **역할**: 루프는 1ms마다 돌지만 CDC는 이 값만큼의 간격으로만 전송. 너무 짧으면 USB 버퍼 오버플로우 발생.
- **현재 기본값**: 10 ms (100 Hz)
- **다른 변수와의 관계**: `cdc_stream_enable = 1` 일 때만 유효.
- **관찰 시 유의점**:
  - Python에서 데이터가 주기적으로 끊기면(dropout) → 20~50으로 늘릴 것
  - 0으로 설정하면 전송이 완전히 멈춤 (0 나누기 방지 코드로 처리됨)

---

## 그룹 3 — EMG 캘리브레이션 상태 플래그

> 이 3개는 **쓰기가 아니라 읽기 전용**. 캘 진행 상태를 확인하는 용도.

### `emg_cal_rest_done`
- **의미**: BTN1 클릭으로 시작한 "휴식 상태 바이어스 캘리브레이션"이 완료됐는지 여부.
- **역할**: 1이 되면 `s_emg_bias_v[0/1]`에 3초 평균 바이어스 전압이 저장됨. 이후 EMG 신호 centering이 더 정확해짐.
- **현재 코드에서 의미**:
  - 0: 자동 bias 추적(`s_emg_auto_bias_v`) 사용 중
  - 1: 수동 캘 바이어스 사용 중
- **관찰 시 유의점**:
  - **최신 코드에서는 0이어도 auto-bias로 동작하므로 0이라고 토크가 안 나오는 건 아님**
  - 수동 캘이 잘 됐는지 확인하는 지표로 사용

---

### `emg_cal_effort_done`
- **의미**: BTN2 클릭으로 시작한 "최대 수축 full-scale 캘리브레이션"이 완료됐는지 여부.
- **역할**: 1이 되면 `s_emg_full_scale_v[0/1]`에 3초 동안의 최대 envelope 값이 저장됨. 이후 norm이 실제 근력 범위에 맞게 0~1로 정규화됨.
- **현재 코드에서 의미**:
  - 0: `EMG_DEFAULT_FULL_SCALE_V = 0.200V` 기본값 사용
  - 1: 실측 최대 envelope 기반 개인화 full-scale 사용
- **관찰 시 유의점**:
  - 0일 때 `emg_rh_norm`이 이상하게 높거나(1.0 고정) 낮으면 → full-scale 기본값(0.2V)과 실제 신호 크기가 많이 다른 것
  - DESCENDING 구간에서 토크가 부족하면 이 값이 0인지 확인

---

### `emg_cal_done`
- **의미**: `emg_cal_rest_done == 1` AND `emg_cal_effort_done == 1` 일 때 1. 완전한 개인화 캘 완료 여부.
- **역할**: 두 단계 캘 모두 완료 시 1이 됨. 현재 코드에서 토크 출력 여부를 직접 제어하지는 않음.
- **관찰 시 유의점**:
  - **이 값이 0이어도 최신 코드에서는 auto-bias + 기본 full-scale로 토크 출력 가능**
  - 1이면 가장 정확한 개인화 상태. 0이면 기본값 모드로 동작 중임을 인지하고 사용

---

## 그룹 4 — EMG 신호 체인

> `pf3_volt → emg_rh_envelope_v → emg_rh_norm` 순으로 처리되는 파이프라인.

### `pf3_volt` / `pf4_volt`
- **의미**: ADC에서 직접 읽은 EMG 센서 원시 전압.
  - `pf3_volt` = PF3 핀 = `XM_EXT_ADC_5` = 오른쪽 고관절(RH) EMG
  - `pf4_volt` = PF4 핀 = `XM_EXT_ADC_6` = 왼쪽 고관절(LH) EMG
- **코드에서**: `emg_rh_raw_v`와 동일한 값 (`s_emg_raw_v[0/1]`에서 복사).
- **정상 범위**: 근육 이완 시 약 **1.5~1.8V** (센서 바이어스 전압). 근육 수축 시 ±0.1~0.5V 변동.
- **관찰 시 유의점**:
  - **0.0V 고정**: `XM_SwitchDioToAdc()` 설정 안 됨. 또는 센서 핀 연결 오류
  - **3.3V 고정**: ADC 레퍼런스 오류 또는 센서 전원 문제
  - **1.604V 근처에서 거의 안 변함**: 센서는 연결됐지만 근육 수축 신호가 없거나 매우 약함
  - 이완 시 `pf3_volt ≈ pf4_volt`이면 두 채널이 같은 핀을 읽고 있는 것 → `s_emg_pins` 배열 확인

---

### `emg_rh_envelope_v` / `emg_lh_envelope_v`
- **의미**: EMG 신호 처리 파이프라인을 거친 **근육 활성도 포락선(envelope) 전압**. 단위: V.
- **계산 과정**:
  ```
  raw_v
  → bias 제거 (centering)
  → 80Hz LPF (고주파 노이즈 제거)
  → 전파 정류 (절댓값)
  → 5Hz LPF (포락선 평활화)
  = envelope_v
  ```
- **정상 범위**: 근육 이완 시 ~0.001~0.010V, 강한 수축 시 0.05~0.5V (센서·배치에 따라 다름)
- **다른 변수와의 관계**: 이 값에서 데드밴드(0.020V)를 빼고 full-scale로 나누면 `emg_rh_norm`이 됨.
- **관찰 시 유의점**:
  - **이완 시 0이 아닌 일정 값(예: 0.04V)이 계속 나옴**: bias 오차. 수동 캘 또는 auto-bias 수렴 기다림
  - **수축해도 0.020V를 못 넘음**: 데드밴드 이하 → `norm`이 항상 0. 전극 위치 확인 또는 `EMG_DEADBAND_V` 낮춤
  - **LH(pf4) envelope가 RH의 1/10 수준**: LH 센서 연결 상태 또는 전극 위치 확인 (어제 실측에서 PF4 신호가 매우 작았음)

---

### `emg_rh_norm` / `emg_lh_norm`
- **의미**: EMG 활성도를 0~1로 정규화한 값. **토크 계산에 직접 사용되는 핵심 변수**.
- **계산 공식**:
  ```
  norm = clamp( (envelope_v - 0.020) / (full_scale_v - 0.020), 0, 1 )
  ```
  - `full_scale_v`: 캘 전 기본 0.200V, 캘 후 실측 최대 envelope
- **다른 변수와의 관계**:
  - DESCENDING 구간 토크 = `emg_norm × squat_max_torque_nm`
  - ASCENDING 구간은 norm을 사용하지 않음 (고정 최대 토크)
- **관찰 시 유의점**:
  - **항상 0**: `envelope_v`가 데드밴드(0.020V) 이하. 신호 파이프라인 문제
  - **항상 1.0 고정**: `full_scale_v`가 너무 작은 것. Effort 캘을 더 강하게 수축해서 재수행
  - **이완 시 0이 아닌 0.05~0.1**: 정상 동작이지만 약한 오프셋 존재. 허용 범위이면 `EMG_DEADBAND_V`를 0.030~0.050으로 높임
  - DESCENDING 중 이 값이 크면 클수록 토크가 강해짐 → 토크가 약하면 이 값부터 확인

---

## 그룹 5 — 엔코더 / 각도

### `left_control_angle_deg` / `right_control_angle_deg`
- **의미**: 기립 자세를 0°로 기준 잡은 **고관절 제어 각도**. 내려가는 방향이 양수(+).
- **계산 공식**:
  ```
  left_control_angle_deg = encoder_sign_lh × (left_encoder_angle_deg − encoder_offset_lh_deg)
  ```
  - `left_encoder_angle_deg`: 전원 ON 이후 누적 원시 각도 (수백~수천도 가능)
  - `encoder_offset_lh_deg`: 기립 시 raw 값 (현재 기본 30.0°)
  - `encoder_sign_lh`: 방향 부호 (기본 1.0f)
- **정상 기대값**:
  - 기립: ~0°
  - 하프 스쿼트: ~20~40°
  - 딥 스쿼트: ~60~80°
- **FSM과의 관계**: 좌우 평균 `(left + right) / 2`가 phase 전환 임계값(`squat_enter_threshold_deg = 15°` 등)과 비교됨
- **관찰 시 유의점**:
  - **기립인데 값이 30° 이상**: `encoder_offset_lh_deg`를 현재 `left_encoder_angle_deg` 값으로 설정
  - **스쿼트해도 값이 변하지 않음**: 엔코더 CAN 통신 문제. `left_encoder_angle_deg` 자체를 확인
  - **left/right 값이 서로 반대 부호로 움직임**: `encoder_sign_lh` 또는 `encoder_sign_rh` 중 하나가 반대
  - **FSM이 STAND에서 전환이 안 됨**: 이 값이 `squat_enter_threshold_deg(15°)`를 못 넘는 것. offset 재설정 또는 임계값 낮춤

---

## 그룹 6 — 최종 보조 토크

### `left_assist_torque_nm` / `right_assist_torque_nm`
- **의미**: 모터에 실제로 명령되는 **최종 보조 토크 값**. 단위: Nm.
- **계산 과정**:
  ```
  DESCENDING: base = emg_norm × squat_max_torque_nm
  ASCENDING:  base = squat_max_torque_nm (고정)
  그 외:      base = 0

  assist_torque = clamp(base + compensation_torque, ±assist_torque_limit_nm)

  모터 출력 = torque_sign × assist_torque
            → 5Hz LPF ramp (s_torque_lh_ramp)
            → XM_SetAssistTorqueLH()
  ```
- **중요**: Live Expressions에 보이는 이 값은 **ramp 적용 전** 목표값. 실제로 모터에 가는 값은 `s_torque_lh_ramp`로, 200ms에 걸쳐 이 값으로 수렴한다.
- **정상 기대값**:
  - STAND/BOTTOM/RETURN: 0 (또는 보상 토크만)
  - DESCENDING (emg_norm=0.5): 0.5 × 6.0 = **3.0 Nm**
  - ASCENDING: **6.0 Nm** (고정)
- **관찰 시 유의점**:
  - **ASCENDING인데 0**: `squat_phase`가 ASCENDING(3)이 아닌 것 → `squat_phase` 먼저 확인
  - **limit(10 Nm)에 고정**: `squat_max_torque_nm + gravity 보상`이 limit 초과. 둘 다 낮출 것
  - **DESCENDING인데 0**: `emg_rh_norm = emg_lh_norm = 0` → EMG 파이프라인 확인
  - **음수**: `torque_sign = -1.0`이거나 `compensation_torque`가 base보다 큰 음수인 것

---

## 한눈에 보는 신호 흐름도

```
pf3_volt (raw ADC)
    │
    ▼ bias 제거 → 80Hz LPF → 정류 → 5Hz LPF
    │
emg_rh_envelope_v
    │
    ▼ (envelope - deadband) / full_scale
    │
emg_rh_norm (0~1)
    │
    ├── DESCENDING 구간 → × squat_max_torque_nm → base torque
    └── ASCENDING 구간 → squat_max_torque_nm 직접


left_encoder_angle_deg (raw, 누적)
    │
    ▼ sign × (raw - offset)
    │
left_control_angle_deg (0° = 기립)
    │
    ├── FSM phase 판정 (squat_enter/bottom/stand_threshold 비교)
    └── compensation: gravity_mgl_nm × sin(θ) × compensation_scale
                            │
                    left_compensation_torque_nm


base + compensation
    │
    ▼ clamp(±assist_torque_limit_nm)
    │
left_assist_torque_nm (목표값, Live Expr. 표시)
    │
    ▼ × torque_sign → 5Hz LPF ramp
    │
XM_SetAssistTorqueLH() ← 실제 모터 출력
```

---

## 빠른 진단 체크리스트

| 증상 | 먼저 확인할 변수 |
|---|---|
| 토크가 전혀 없음 | `assist_mode_active` → `squat_control_ON` → `squat_phase` |
| DESCENDING 토크 약함 | `emg_rh_norm`, `emg_lh_norm` → `emg_rh_envelope_v` → `pf3_volt` |
| ASCENDING 토크 없음 | `squat_phase` 값이 3인지 확인 |
| 보상 토크 없거나 이상함 | `left_control_angle_deg`가 기립 시 0인지 → `compensation_ON` 값 |
| FSM이 STAND에서 안 넘어감 | `left_control_angle_deg`, `right_control_angle_deg` 값 변화 확인 |
| Python Decoder에 데이터 없음 | `cdc_stream_enable = 1`, `cdc_stream_period_ms` 값 확인 |
