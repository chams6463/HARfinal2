# 민서 지형분류기 v1.2 선택 개선 보고서

> 작성: 2026-08-25  
> 목표: 전환 시점을 런타임에 주지 않는 연속보행에서 판정 정확도를 높이되,
> wrong-mode를 늘리는 수정은 채택하지 않는다.  
> 보존: `VOTE_WINDOW=3`, `VOTE_MAJORITY=2`, `_PushVote()`,
> `_UpdateModeFromVotes()`는 v1.1과 동일하다.

## 1. 최종 결론

필요성이 실제 데이터로 확인된 두 수정만 최종 C에 넣었다.

1. 기준곡선과 다른 MHE stride 경계를 H10 우측 footContact rising edge로 교체했다.
2. 전후 평면 feature에 잘못 연결된 `pelvicAngle`을 양측
   `HipImuSagittalPitch` 평균으로 교체했다.

혼합 전환 점수는 정답 전환을 늘렸지만 wrong-mode도 늘어 최종 코드에서 제외했다.
IDLE/margin과 causal EMG 수정은 v1.1 상태를 유지한다.

대표 성능은 기준곡선 학습에서도 제외한 EMG 이상치 AB11을 평가에서도 제외한
clean 8명으로 산정했다. 전환 시점을 런타임에 주지 않은 연속보행과 20회 무작위
캘리브레이션에서 heel-strike 경계 상한 proxy는 다음과 같이 개선됐다.

- 안정 stride 후보 분류 정확도: **61.83% → 90.19%**
- 구간 안 최종 정답 전환 정확도: **5.14% → 14.39%**
- wrong-mode episode: **18.04% → 12.67%**
- 안정 이동 시간 정확도: **4.43% → 16.17%**

AB11을 포함한 9명 결과는 대표 정확도가 아니라 이상치 포함 스트레스 테스트로
§5.2에 별도 표시했다.

> [!warning]
> 개선은 분명하지만 14.39% 최종 전환 성공은 여전히 자동 토크 선택기로 부족하다.
> 또한 after 경계는 데이터셋의 gcRight heel-strike를 사용한 상한 proxy다. 실제
> H10 footContact 지연·누락·chatter를 검증한 결과가 아니다.

## 2. 왜 MHE가 가장 큰 병목이었는가

### 변경 전

```text
Camargo 기준곡선 theta=0: right heel-strike
H10 런타임 theta=0:       right-thigh MHE
평지 8 stride에서 찾은 원형 shift 하나로 둘을 정렬
```

MHE와 heel-strike는 서로 다른 사건이다. 두 사건의 시간차가 모든 지형에서
일정해야 하나의 shift로 정렬할 수 있지만, 실제 보폭과 지형이 바뀌면 그 차이도
변한다. 따라서 평지 캘리브레이션은 계단·경사로 위상 오차를 제거하지 못했다.

### 변경 후

```c
bool heel_strike_r = s_body_data_configured
                     && s_contact_r_armed
                     && contact_r
                     && !s_prev_contact_r;
```

기준곡선과 런타임 모두 오른쪽 heel-strike를 theta 원점으로 사용한다.

- contact가 false를 거친 뒤의 rising edge만 허용한다.
- 500 ms보다 짧은 rising edge는 chatter로 무시한다.
- 3,000 ms보다 긴 간격은 분류하지 않고 경계만 재동기화한다.
- 첫 edge는 시작점이며 두 번째 유효 edge부터 완성 stride를 분류한다.
- 미래 표본과 정답 전환 라벨은 사용하지 않는다.

## 3. H10 Body Data 처리

footContact와 forwardVelocity는 H10 내부 보행분석 출력이라 사용자 신체정보가
필요하다. 실제 값을 알지 못한 상태에서 공식 예제의 표준체형을 복사하면 잘못된
접촉 경계를 정상 입력처럼 사용할 위험이 있다.

따라서 상단 `LMC_BODY_*` 여덟 값의 기본값은 0이다. 하나라도 0이면
`_SetupBodyData()`는 `XM_SendUserBodyData()`를 호출하지 않고 분류 경계를
비활성화한다. 사용 전에 체중, 신장, 좌우 대퇴·하퇴·발목 높이 실측값을 입력해야 한다.

## 4. 왜 pelvicAngle이 잘못됐는가

### 변경 전

```c
case RC_FEAT_TRUNK_TILT:
    return XM.status.h10.pelvicAngle;
```

Camargo `TRUNK_TILT`는 `atan2(trunk_Accel_X, trunk_Accel_Z)`로 만든 전후
시상면 기울기다. H10 API의 `pelvicAngle`은 골반 좌우 tilt다. `sign/scale/offset`
캘리브레이션으로 서로 다른 해부학적 평면을 변환할 수는 없다.

### 변경 후

```c
return 0.5f * (XM.status.h10.leftHipImuSagittalPitch
               + XM.status.h10.rightHipImuSagittalPitch);
```

양측 평균은 H10에서 사용할 수 있는 시상면 proxy다. 단, Camargo trunk IMU와
H10 proximal IMU의 위치가 같다는 뜻은 아니므로 실제 H10 로그 검증은 남는다.

## 5. 20회 연속보행 전후 결과

평가 계약은 `validate_11_h10_selected.py`에 고정했다.

- 대표 cohort: AB11을 제외한 AB06~AB14의 clean 8명
- 스트레스 cohort: AB11을 포함한 9명(전체 839개 연속 trial, 3,356전환)
- 전환 라벨은 scorer만 사용
- trial 내부 전환 시 런타임 상태를 리셋하지 않음
- 별도 평지 8 stride 캘리브레이션
- 20회 무작위 캘리브레이션 반복
- IDLE/margin, EMG 전처리, 3/2 다수결 동일
- after의 gcRight heel-strike는 H10 footContact 상한 proxy

### 5.1 대표 정확도: AB11 제외 clean 8명 pooled

| 지표 | MHE 경계 | heel-strike 경계 | 변화 |
|---|---:|---:|---:|
| 안정 stride 후보 argmin | 61.83% | 90.19% | +28.36%p |
| 실제 투표 stride | 24.46% | 52.50% | +28.04%p |
| 2 stride 이내 전환 | 0.86% | 3.55% | +2.69%p |
| 구간 안 최종 정답 전환 | 5.14% | 14.39% | +9.25%p |
| 안정 이동 시간 정확도 | 4.43% | 16.17% | +11.74%p |
| 전환 직전 목표 조기출력 | 39.45% | 33.09% | -6.36%p |
| wrong-mode episode | 18.04% | 12.67% | **-5.38%p 개선** |
| 중앙 전환 지연 | 3.152 s | 2.815 s | -0.337 s |

여기서 **90.19%는 완성된 안정 stride마다 margin/IDLE 차단 전에 가장 가까운
후보를 고르는 분류 정확도**다. 전체 연속보행에서 차단, 3/2 투표, 전환 지연까지
포함한 더 엄격한 end-to-end 지표는 **구간 안 최종 정답 전환 14.39%**다.

### 5.2 스트레스 테스트: AB11 포함 전체 9명 pooled

| 지표 | MHE 경계 | heel-strike 경계 | 변화 |
|---|---:|---:|---:|
| 안정 stride 후보 argmin | 57.93% | 86.10% | +28.17%p |
| 실제 투표 stride | 23.43% | 48.09% | +24.66%p |
| 2 stride 이내 전환 | 0.81% | 3.33% | +2.53%p |
| 구간 안 최종 정답 전환 | 4.78% | 13.11% | +8.32%p |
| 안정 이동 시간 정확도 | 4.28% | 14.57% | +10.29%p |
| 전환 직전 목표 조기출력 | 40.06% | 34.49% | -5.57%p |
| wrong-mode episode | 17.38% | 11.85% | **-5.52%p 개선** |
| 중앙 전환 지연 | 3.149 s | 2.815 s | -0.334 s |

## 6. 검토했지만 넣지 않은 수정

### 6.1 현재→목표 혼합 전환 점수

완성 stride의 앞부분을 현재 모드, 뒷부분을 목표 모드 기준곡선과 비교하고
분할점을 30/40/50/60/70%에서 탐색했다. 런타임에 전환 라벨은 주지 않았다.

3회 절제평가에서 보수적인 `gain>=0.10`, `separation>=0.10` 후보는:

- 최종 정답 전환: 13.17% → 17.01%
- 2 stride 이내 전환: 3.08% → 3.93%
- wrong-mode: 11.96% → 12.93%

정답 전환은 늘었지만 wrong-mode도 약 0.97%p 증가해 안전 게이트를 통과하지
못했다. 더 공격적인 설정은 wrong-mode가 20% 이상으로 증가했다. 구현과 결과는
`validate_10_selected_fixes.py`와 `minseo_selected_fixes_3.json`에 보존했다.

### 6.2 다수결 변경

사용자 요구에 따라 실험하지 않았다. 최종 C의 3/2 함수는 v1.1과 동일하다.

## 7. 검증 완료와 남은 한계

완료:

- Python 검증 스크립트 문법 검사
- Camargo 연속보행 20회 평가
- C 괄호·프로토타입·TSM 콜백·미사용 static 구조 검사
- Rev1.1 SDK 전체 헤더를 사용한 host C syntax 검사(`-Werror`)
- 3/2 상수와 함수 동일성 확인

미완료:

- 실제 ARM Release 링크·플래시
- 실제 사용자의 Body Data 입력
- H10 footContact 지연·누락·chatter 측정
- 양측 SagittalPitch와 Camargo trunk tilt의 실기 대응 확인
- 실제 H10 12방향 전환 정확도
- 토크 효과와 안전성 검증

> [!danger]
> 이 버전은 `XM_CTRL_MONITOR`, 토크 0의 Shadow 후보로만 사용한다. 실제 H10
> wrong-mode와 안정보행 오작동 게이트를 통과하기 전에는 토크 프로파일 선택에
> 연결하지 않는다.

## 8. 재현 명령

```bash
cd Camargo_matlab
python validate_10_selected_fixes.py --calib-reps 3
python validate_11_h10_selected.py --calib-reps 20
python check_c_structure.py
```
