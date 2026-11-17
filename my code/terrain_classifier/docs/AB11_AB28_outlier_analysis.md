# AB11 / AB28 오분류 원인 분석

검증[1](held-out 정확도) 및 검증[5](LOSO 교차검증)에서 이상치로 나타난
AB11(LOSO 61.9%), AB28(held-out 78.2%) 두 피험자를 feature별 거리 분석으로
직접 파고든 결과.

## AB11 — RAMP_DOWN이 STAIRS_DOWN으로 새는 이유

7명 뱅크로 AB11 개별 혼동행렬을 다시 뽑아보면 LOSO와 거의 동일하게
**RAMP_DOWN 정확도 29.4%**(143개 중 96개가 STAIRS_DOWN으로 오분류)로 나온다.
4개 feature 각각이 "진짜 RAMP_DOWN 템플릿"과 "오분류된 STAIRS_DOWN 템플릿"
중 어느 쪽에 더 가까운지 뜯어보면:

| feature | d(RAMP_DOWN) | d(STAIRS_DOWN) | 판정 |
|---|---|---|---|
| TRUNK_TILT | 0.284 | 0.630 | 정상 (RAMP쪽이 더 가까움) |
| THIGH_GYROMAG | 0.343 | 0.778 | 정상 |
| **EMG_SITE0(gastrocmed)** | **1.975** | **1.144** | **STAIRS쪽에 더 가까움** |
| EMG_SITE1(vastusmedialis) | 0.436 | 0.681 | 정상 |

핵심: 나머지 3개 feature는 다 정상적으로 RAMP_DOWN을 가리키는데,
**EMG_SITE0(gastrocmed) 하나가 두 모드 어느 쪽과도 안 맞을 정도로 절대거리
자체가 비정상적으로 큽니다**(1.1~2.0 — 다른 feature는 전부 1 미만). 게다가
검증[4]에서 이미 봤듯 AB11의 EMG 캘리브레이션 scale이 2.09x/2.34x로
원본 7명 중에서도 높은 축(전극 신호가 유난히 작아서 크게 뻥튀기해야 했다는
뜻)입니다. **전극 접촉 불량/피부저항 문제로 이 세션의 gastrocmed EMG 파형
자체가 왜곡됐을 가능성**이 가장 유력한 가설이다.

## AB28 — RAMP_UP이 STAIRS_UP으로 새는 이유

AB28은 (WALK/STAIRS_UP/STAIRS_DOWN 전부 100%인데) **RAMP_UP만 45.3%**로
처진다(159개 중 77개가 STAIRS_UP으로). 같은 분석:

| feature | d(RAMP_UP) | d(STAIRS_UP) | 판정 |
|---|---|---|---|
| TRUNK_TILT | 0.247 | 0.466 | 정상 |
| **THIGH_GYROMAG** | **0.759** | **0.530** | **STAIRS쪽에 더 가까움** |
| EMG_SITE0(gastrocmed) | 0.235 | 0.518 | 정상 |
| **EMG_SITE1(vastusmedialis)** | **0.421** | **0.377** | **STAIRS쪽에 근소하게 더 가까움(가중치 0.8)** |

AB11과 달리 **캘리브레이션 계수는 전부 정상 범위**(검증[4]에서 이상치로 안
잡힘) — 즉 이건 전극/센서 문제가 아니라 **진짜 파형 모양(고관절 자이로
궤적)이 다른 문제**로 보인다. 눈에 띄는 점: AB28은 12명 중 **유일하게
33세**이고(나머지는 전부 19~24세), 여성이다. 뱅크를 만든 원본 7명은 전부
20대 초반 — **연령대 차이로 인한 경사로 보행 패턴 차이**(계단처럼 더
조심스럽게/짧은 보폭으로 오르는 전략)일 가능성이 있어 보인다.

## 직접 파일에서 확인해볼 것 — 추천 경로

같은 사람의 ramp 구간과 stair 구간을 같은 근육/센서로 나란히 비교하는 게
가장 직접적이다. MATLAB에서 raw signal 위치를 찾아뒀다.

### AB11 — gastrocmed EMG를 rampdescent vs stairdescent에서 비교

```
D:\gait_data\AB11\10_28_2018\ramp\emg\ramp_1_l_01_01.mat   (data.gastrocmed, t=24.69~27.52s)
D:\gait_data\AB11\10_28_2018\stair\emg\stair_1_l_01_01.mat (data.gastrocmed, t=25.18~26.73s)
```

`data.Header`로 시간을 자르고 `data.gastrocmed`(그리고 참고로
`data.vastusmedialis`)를 두 구간 겹쳐 그려본다. gastrocmed가 유독 **진폭이
작거나 flat/노이즈에 묻혀** 있으면 전극 문제 가설이 맞는 것이고, 오히려
파형 자체는 멀쩡한데 모양이 계단과 비슷하면 다른 얘기가 된다.

### AB28 — 고관절(thigh) 자이로를 rampascent vs stairascent에서 비교

```
D:\gait_data\AB28\03_02_2019\ramp\imu\Ramp_1_L_01_01.mat   (data.thigh_Gyro_X/Y/Z, t=12.90~16.68s)
D:\gait_data\AB28\03_02_2019\stair\imu\Stair_1_L_01_01.mat (data.thigh_Gyro_X/Y/Z, t=10.08~11.30s)
```

`sqrt(Gx²+Gy²+Gz²)`로 크기를 만들어서 두 구간을 겹쳐본다. 계단은 stride가
짧아서(1.2초) 자이로 피크가 좁고 뾰족하고, 램프는 보통 더 완만하다. AB28의
ramp 파형이 **자기 자신의 stair 파형과 더 닮았는지**(뱅크의 ramp 평균과는
덜 닮았는지)가 핵심 확인 포인트다.

### 공통 팁

- `gcRight\*.mat`(같은 폴더에 있음)을 같이 로드하면 `data.HeelStrike`
  (0~100%GC)로 여러 stride를 위상 정렬해서 겹쳐 그릴 수 있다 — 한 stride만
  보면 우연일 수 있으니 3~5개 stride를 겹쳐서 일관된 패턴인지 확인하는 걸
  추천한다.
- AB11/AB28은 `conditions\*.mat`의 `labels`가 MATLAB **table** 객체라서
  `scipy.io.loadmat`(Python)으로는 못 읽는다 — 확인은 MATLAB에서 해야 한다
  (`S = load(...); S.labels`).
- osimxml 폴더의 SubjectInfo(AB28: 33세/F/1.69m/62kg, AB11: 21세/M/1.75m/77kg)도
  같이 띄워두고 보면 맥락 잡기 좋다.
