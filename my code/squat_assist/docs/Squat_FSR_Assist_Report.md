# Squat_FSR_Assist.c 구현 레포트

## 1. 개요

`Squat_FSR_Assist.c`는 `Squat_EMG_Assist.c`의 **Phase 판정 로직만** 교체한 파일이다.  
원본 파일은 수정하지 않았으며, 기존 FSR 캘리브레이션·EMG 처리·보상 토크·CDC 스트림 파이프라인은 그대로 유지했다.

| 항목 | `Squat_EMG_Assist.c` | `Squat_FSR_Assist.c` |
|---|---|---|
| USB Module ID | 0xF0 | **0xF1** (충돌 방지) |
| Phase 판정 기준 | Encoder 각도 단독 | **Encoder + FSR Heel-Toe + EMG 3중 퓨전** |
| CDC 채널 수 | 14 | **16** (heel_ratio, avg_emg 추가) |
| 신규 함수 | — | `_ComputeHeelToeRatio()` |
| 신규 공개 변수 | — | `fsr_heel_ratio`, `fsr_total_load`, `avg_emg_norm`, 퓨전 튜닝 파라미터 8개 |

---

## 2. 설계 배경

### 2.1 Encoder 단독 판정의 한계

기존 코드는 `avg_angle`(좌우 고관절 평균 각도) 임계값만으로 FSM 전환을 결정했다.  
이는 **기하학적 자세** 정보만 반영하며 아래 상황에서 오판이 발생할 수 있다.

- 앞으로 허리를 숙이는 Good-Morning 동작 → 각도 증가 → 오인식
- 기립 직후 흔들림으로 인한 각도 노이즈 → 잦은 STAND↔DESCENDING 진동
- 스쿼트 최저점에서 속도 반전이 노이즈에 가려 늦게 감지됨

### 2.2 추가 신호의 의의

| 신호 | 스쿼트 하강 | 최저점 | 상승 복귀 | 기립 |
|---|---|---|---|---|
| Encoder `avg_angle` | ↑ 증가 | 최대 | ↓ 감소 | ~0° |
| FSR `heel_ratio` | **↑ 증가** (체중이 뒤로) | 높음 | **↓ 감소** (앞꿈치 push-off) | 중간(~0.5) |
| EMG `avg_emg` | 증가 (eccentric) | 높음 | **높음** (concentric) | 낮음 |

세 신호를 퓨전하면 동작 의도를 교차 검증할 수 있어 오인식이 줄어든다.

---

## 3. 구현 내용

### 3.1 신규 함수: `_ComputeHeelToeRatio()`

```
heel_load  = fsr_load[LH] + fsr_load[RH]   (뒤꿈치 2채널 합산)
toe_load   = fsr_load[LT] + fsr_load[RT]   (앞꿈치 2채널 합산)
total_load = heel_load + toe_load

if total_load > fsr_min_total_load:
    heel_ratio = heel_load / total_load      ← 0(all-toe) ~ 1(all-heel)
else:
    heel_ratio = 0.5  (하중 불충분: neutral 고정)
```

`fsr_min_total_load`(기본 0.05): 발이 들렸거나 FSR 캘이 안 된 상태에서 비율이 튀는 것을 방지.

---

### 3.2 신규 신호: EMG 평균 (`s_avg_emg`)

`_ProcessEmgSignals()` 끝에서 계산한다.

```c
s_avg_emg = (s_emg_norm_priv[EMG_LH] + s_emg_norm_priv[EMG_RH]) * 0.5f;
```

EMG 캘이 미완료이면 `s_emg_norm_priv[]`가 극히 작으므로 퓨전 기여도가 자동으로 낮아진다 (안전 특성).

---

### 3.3 퓨전 점수 계산식

STAND → DESCENDING 전환과 RETURN → STAND 전환에 적용한다.

```
score = phase_w_enc × enc_cond
      + phase_w_fsr × fsr_cond
      + phase_w_emg × emg_cond

전환 확정: score ≥ phase_fusion_thresh  (+ 150 ms debounce)
```

기본 가중치: `w_enc=0.50 / w_fsr=0.30 / w_emg=0.20 / thresh=0.50`

| 만족 조건 | score | 전환 여부 |
|---|---|---|
| encoder 만 | 0.50 | 통과 (단독도 허용) |
| encoder + FSR | 0.80 | 통과 |
| encoder + EMG | 0.70 | 통과 |
| encoder + FSR + EMG | 1.00 | 통과 |
| FSR + EMG (encoder 미만) | 0.50 | 통과 (강한 퓨전 확신) |
| FSR 만 | 0.30 | **차단** |
| EMG 만 | 0.20 | **차단** |

> **더 엄격한 퓨전을 원하면** `phase_fusion_thresh = 0.60`으로 설정.  
> encoder 단독(0.50)으로는 전환되지 않고, 최소 1개 이상의 보조 신호가 필요해진다.

---

### 3.4 FSM 전환 로직 전체

```
STAND ──────────────────────────────────────────────────────────
  조건: enc(각도≥enter) + fsr(heel≥heel_thresh) + emg(≥emg_thresh)
  퓨전 score ≥ fusion_thresh + 150 ms debounce
  → DESCENDING

DESCENDING ──────────────────────────────────────────────────────
  → BOTTOM  : avg_angle ≥ bottom_threshold (encoder 단독)
  → RETURN  : avg_angle < enter_threshold  (중도 복귀, 즉시)

BOTTOM ──────────────────────────────────────────────────────────
  조건 (OR):
    (A) avg_velocity < -0.05 rad/s        ← 속도 반전
    (B) fsr_heel_ratio ≤ toe_thresh  AND  avg_emg ≥ emg_thresh
                                          ← toe push-off + 근활성
  → ASCENDING

ASCENDING (EMG 비례 보조 토크 출력 구간) ───────────────────────
  → RETURN  : avg_angle < enter_threshold (즉시)

RETURN ──────────────────────────────────────────────────────────
  → STAND   : enc(각도≤stand) + fsr(neutral zone) + emg(이완)
              퓨전 score ≥ fusion_thresh + 150 ms debounce
  → ASCENDING: avg_angle ≥ enter_threshold (재하강, 즉시)
```

---

## 4. 새로 추가된 Live Expressions 변수

### 관측 전용 (Observe Only)

| 변수 | 설명 | 정상 범위 |
|---|---|---|
| `fsr_heel_ratio` | 실시간 heel/(heel+toe) | 하강 0.60↑, 기립 0.48~0.52 |
| `fsr_total_load` | 4채널 FSR 합산 하중 | 체중 전하중 시 ~4.0 |
| `avg_emg_norm` | 좌우 EMG norm 평균 | 이완 0~0.05, 수축 0.10↑ |

### 튜닝 파라미터 (Write from Debugger)

| 변수 | 기본값 | 설명 |
|---|---|---|
| `fsr_heel_thresh` | 0.58 | 하강 판정 heel 임계값 |
| `fsr_toe_thresh` | 0.42 | 상승 판정 toe 임계값 |
| `fsr_min_total_load` | 0.05 | FSR 유효 최소 하중 |
| `emg_phase_thresh` | 0.10 | EMG 활성 판정 임계값 |
| `phase_w_enc` | 0.50 | encoder 퓨전 가중치 |
| `phase_w_fsr` | 0.30 | FSR 퓨전 가중치 |
| `phase_w_emg` | 0.20 | EMG 퓨전 가중치 |
| `phase_fusion_thresh` | 0.50 | 전환 확정 점수 기준 |

---

## 5. CDC 스트림 채널 구성 (Module ID 0xF1, 16채널, 100 Hz)

| # | 이름 | 단위 | 설명 |
|---|---|---|---|
| 1 | EMG RH Env | V | 우측 EMG 포락선 |
| 2 | EMG LH Env | V | 좌측 EMG 포락선 |
| 3 | EMG RH Nrm | — | 우측 EMG 정규화 |
| 4 | EMG LH Nrm | — | 좌측 EMG 정규화 |
| 5 | FSR LT | — | 좌측 앞꿈치 하중 |
| 6 | FSR LH | — | 좌측 뒤꿈치 하중 |
| 7 | FSR RT | — | 우측 앞꿈치 하중 |
| 8 | FSR RH | — | 우측 뒤꿈치 하중 |
| 9 | Squat Ph | id | Phase 번호 (0~4) |
| 10 | L Torque | Nm | 좌측 보조 토크 |
| 11 | R Torque | Nm | 우측 보조 토크 |
| **12** | **Avg EMG** | **—** | **EMG 평균 norm (신규)** |
| 13 | FSR Cal | bool | FSR 캘 완료 |
| 14 | Control | bool | squat_control_ON |
| **15** | **Heel Ratio** | **—** | **heel/(heel+toe) 비율 (신규)** |
| 16 | Assist | bool | assist_enable |

---

## 6. 캘리브레이션 절차

원본(`Squat_EMG_Assist.c`)과 동일하다.

1. H10를 ASSIST 모드로 전환
2. **BTN1 PRESSED** → FSR zero 캘 (무부하 1초)
3. **BTN2 PRESSED** → FSR full-load 캘 (체중 전체 1초)
4. **BTN1 CLICK** → EMG rest 캘 (이완 3초)
5. **BTN2 CLICK** → EMG effort 캘 (최대 수축 3초)
6. `fsr_cal_ready == 1` + `emg_cal_done == 1` 확인
7. `squat_control_ON = 1` 설정

---

## 7. 문제 해결 가이드

### phase 판정이 원본보다 너무 까다로울 때
```
phase_fusion_thresh = 0.40   # 낮출수록 쉽게 전환
```

### FSR 신호를 퓨전에서 완전히 제외하고 싶을 때
```
phase_w_fsr = 0   # FSR 기여 0
```

### EMG 캘 전에도 동작시키고 싶을 때
```
phase_w_emg = 0   # EMG 기여 0 → encoder + FSR 2중 퓨전
```

### BOTTOM → ASCENDING 전환이 속도 반전 없이 일어날 때
- 원인: FSR toe 우세 + EMG 활성 경로 (B)가 먼저 트리거
- 해결: `fsr_toe_thresh`를 0.35로 낮추거나 `emg_phase_thresh`를 0.20으로 올림

### 기립 자세인데 STAND로 안 돌아올 때
- 원인: EMG가 여전히 `emg_phase_thresh` 이상
- 해결: `phase_w_emg = 0`으로 임시 비활성화 후 EMG 잔류 원인 조사

---

## 8. 변경하지 않은 부분

아래 로직은 `Squat_EMG_Assist.c`와 완전히 동일하다.

- FSR ADC 샘플링 / LPF / 캘리브레이션 상태머신
- EMG 신호 처리 파이프라인 (bias 제거 → LPF → 정류 → 포락선 → 정규화)
- EMG 캘리브레이션 상태머신
- Encoder 샘플링 / 속도 계산
- 중력 · 마찰 보상 토크
- 보조 토크 출력 (`_ApplyTorque`) — ASCENDING phase에서 EMG 비례 출력
- 안전 게이트 (squat_control_ON + fsr_cal_ready + H10 ASSIST)
- USB CDC 스트림 전송 구조
