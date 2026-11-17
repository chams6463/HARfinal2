# Squat_EMG_Assist.c — 함수 기능 정리

`Squat_EMG_Assist.c`의 모든 static 함수 프로토타입과 역할을 정리한 문서.

---

## 1. 입력 (Input)

### `_ReadButtons()`
- **반환**: `BtnEvents_t` (버튼 이벤트 구조체)
- **역할**: 외골격의 물리 버튼 상태를 읽어 edge-detected 이벤트로 반환.
- **사용처**: EMG 캘리브레이션 시작 트리거(Rest/Effort), 어시스트 모드 전환 등.

---

## 2. EMG 처리

### `_SampleEmg()`
- **역할**: 1kHz ISR마다 EMG 채널 2개(RH, LH)의 ADC를 읽어 raw 전압으로 변환.
- **부수 효과**: Rest 캘 진행 중이면 `s_emg_cal_sum[]`에 누적.

### `_ResetEmgFilters()`
- **역할**: EMG 신호 처리 파이프라인의 모든 필터 상태(centered, pre_lpf, rect, envelope)를 0으로 초기화.
- **호출 시점**: Rest 캘 진행 중(필터 왜곡 방지), ASSIST 모드 진입 시.

### `_ProcessEmgSignals()`
- **역할**: raw → centered(bias 제거) → 80Hz LPF → 정류 → 5Hz envelope LPF → 정규화(0~1).
- **출력**: `s_emg_norm_priv[]` (정규화 결과).
- **주의**: Rest 캘 중에는 호출 안 됨.

### `_UpdateEmgCal(const BtnEvents_t *ev)`
- **역할**: 캘리브레이션 FSM 상태 갱신. 버튼 이벤트와 시간 경과를 보고 Rest/Effort 캘의 진행/완료를 처리.
- **완료 시**: `s_emg_bias_v[]`, `s_emg_full_scale_v[]`를 평균/최대치로 갱신.

### `_StartEmgRestCal()`
- **역할**: Rest 캘 시작. `s_emg_cal_sum[]=0`, count 리셋, 상태 → `EMG_CAL_REST_RUNNING`.
- **동작**: 3초간 raw 전압 누적 후 평균 → bias.

### `_StartEmgEffortCal()`
- **역할**: Effort 캘(MVC) 시작. `s_emg_cal_max_env[]=0`, 상태 → `EMG_CAL_EFFORT_RUNNING`.
- **동작**: 사용자 최대 수축 시 envelope 피크 추적 → full_scale.

### `_ResetEmgCal()`
- **역할**: 캘 상태/누적치 모두 초기 상태로 되돌림 (재캘용).

### `_GetEmgNorm(int ch)`
- **반환**: 채널 `ch`의 정규화된 EMG 값 (0~1).
- **역할**: 외부 모듈에서 EMG 값을 읽을 때 사용하는 접근자.

---

## 3. 엔코더 + 보상

### `_SampleEncoder()`
- **역할**: 좌/우 무릎(또는 고관절) 엔코더 각도를 읽어 raw → offset/부호 보정 → control_angle로 변환.
- **출력**: `left_encoder_angle_deg`, `left_control_angle_deg` 등.

### `_UpdateCompensation()`
- **역할**: 중력 보상 + (선택)마찰 보상 토크 계산.
- **계산**:
  - `gravity_torque = gravity_mgl_nm × sin(θ)`
  - `compensation_torque = scale × (gravity + friction)` (compensation_ON 게이팅)
- **부수 효과**: 수치 미분으로 각속도 계산, `s_comp_prev_*` 갱신.

---

## 4. 스쿼트 FSM + 출력

### `_UpdateSquatFsm()`
- **역할**: 5단계 스쿼트 FSM 상태 전이 (STAND → DESCENDING → BOTTOM → ASCENDING → RETURN → STAND).
- **판정 기준**:
  - 대부분 전이: 평균 각도 임계치 (15°/45°/5°)
  - BOTTOM → ASCENDING: 각속도(`ω < -0.05`) OR EMG(`avg ≥ emg_ascend_threshold`)
- **디바운스**: STAND ↔ DESCENDING, RETURN → STAND 전이에 150ms 디바운스.

### `_ApplyTorque()`
- **역할**: 최종 어시스트 토크 계산 후 모터에 명령.
- **흐름**:
  1. Phase별 base 토크: DESCENDING=EMG비례, ASCENDING=고정, BOTTOM/STAND=0
  2. `assist = clamp(base + compensation, ±limit)`
  3. ramp 평활화 (`s_torque_lh_ramp` LPF)
  4. `XM_SetTorque()` 호출
- **안전**: 3중 클램프 (limit / HARD_MAX / ramp).

---

## 5. Housekeeping

### `_UpdatePublicSignals()`
- **역할**: 내부 static 배열의 값을 디버그용 전역 스칼라(`emg_rh_raw_v`, `left_gravity_torque_nm` 등)로 복사.
- **목적**: STM32CubeIDE Live Expressions에서 관찰 가능하게 함.
- **호출 시점**: 매 ISR 끝.

### `_SendStream()`
- **역할**: USB CDC 스트림 패킷(`s_stream`) 채워서 호스트로 전송.
- **주기**: `cdc_stream_period_ms` (기본 10ms = 100Hz). `s_stream_tick`으로 분주.

---

## 6. 유틸리티 (인라인 후보)

### `_LowPassUpdate(prev, input, cutoff_hz)`
- **반환**: 1차 IIR LPF 출력값.
- **공식**: `y[n] = y[n-1] + α(x[n] - y[n-1])`, α = f(cutoff_hz, dt).
- **사용처**: EMG envelope, 각속도 평활화, 토크 ramp.

### `_ClampFloat(value, lo, hi)`
- **반환**: `value`를 `[lo, hi]` 범위로 제한.
- **사용처**: 토크 안전 클램프, 사용자 설정값 검증.

### `_AbsFloat(x)`
- **반환**: `|x|`.
- **사용처**: 마찰 데드존 비교 (`|ω| > deadzone`).

### `_SignFloat(x)`
- **반환**: x > 0이면 +1, x < 0이면 -1, x = 0이면 0.
- **사용처**: Coulomb 마찰 `sign(ω)`.

---

## 호출 순서 (1kHz ISR)

```
ISR 진입
  ├─ _ReadButtons()              ← 입력 이벤트
  ├─ _SampleEmg()                ← EMG raw + 캘 누적
  ├─ _UpdateEmgCal(&ev)          ← 캘 FSM
  ├─ _ProcessEmgSignals()        ← 필터링 + 정규화 (캘 중엔 스킵)
  ├─ _SampleEncoder()            ← 각도 읽기
  ├─ _UpdateCompensation()       ← 각속도 + 보상 토크
  ├─ _UpdateSquatFsm()           ← 스쿼트 phase 갱신
  ├─ _ApplyTorque()              ← base + comp → ramp → 모터
  ├─ _UpdatePublicSignals()      ← 디버그 미러링
  └─ _SendStream()               ← (조건부) USB 전송
ISR 종료
```

---

## 그룹별 역할 요약

| 그룹 | 역할 | 함수 수 |
|------|------|---------|
| 입력 | 버튼 이벤트 | 1 |
| EMG | 샘플링 + 필터 + 캘 + 접근자 | 8 |
| 엔코더/보상 | 각도 읽기 + 중력/마찰 보상 | 2 |
| FSM/출력 | phase 판정 + 토크 명령 | 2 |
| Housekeeping | 디버그 미러 + USB 스트림 | 2 |
| 유틸리티 | LPF/clamp/abs/sign | 4 |
| **총** | | **19** |
