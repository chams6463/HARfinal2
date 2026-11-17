H10 Minecraft 연속 판정 v9 (Windows 전달본)
============================================

이 폴더 하나를 팀원 Windows 노트북으로 복사하면 됩니다.
펌웨어 데이터 계약은 13개 float이고, 대응하는 Final_EMG.c도 포함했습니다.

[핵심 판정/키 계약]
- H10 1 kHz 센서 50행마다 result JSON 1개 출력 (50 ms)
- state=walk: W 유지, walk가 아니면 W 즉시 해제
- jump_started=true: Space 1회 입력
- crouch_active=true: Shift 유지
- crouch_active=false: state/confidence와 무관하게 Shift 즉시 해제
- 완전 웅크림: 양쪽 고관절이 학습 최대 굴곡의 95% 이상이고 정지 plateau인
  상태가 2 tick(100 ms) 연속일 때만 crouch_active=true
- turn_rate_dps: 부호와 크기로 좌우 시선 회전
- arm_emg: PF5 입력으로 기존 좌/우클릭 처리

[중요]
- 웅크리기를 쓸 것이므로 --disable-crouch 옵션을 붙이지 마세요.
- Minecraft 창을 클릭해 포커스를 준 뒤 실행하세요.
- COM6은 예시입니다. 장치 관리자에서 실제 XM10 USB CDC 포트로 바꾸세요.
- Assist mode는 필요하지 않습니다. 이 Python 코드는 토크 명령을 보내지 않습니다.

1. 최초 1회 설치
----------------
이 폴더에서 PowerShell 또는 cmd를 열고 실행합니다.

  py -m pip install -r requirements_h10_motion_classifier.txt

같은 Python 환경에 이미 설치했다면 다시 실행할 필요가 없습니다.

2. STM32/XM10
-------------
- STM32CubeIDE 프로젝트의 User_Algorithm에 이 폴더의 Final_EMG.c를 적용합니다.
- Build 후 XM10에 Flash/Run 합니다.
- XM10은 켜진 상태여야 하며, USB CDC가 Windows에서 COM 포트로 보여야 합니다.
- Python만 실행한다고 새 C 코드가 STM32에 적용되지는 않습니다.

3. Minecraft/RCON
-----------------
server.properties에서 RCON을 켜고 서버를 재시작합니다.

  enable-rcon=true
  rcon.port=25575
  rcon.password=emgdemo

4. 실시간 실행
--------------

  py h10_live_motion_classifier.py --port COM6 --minecraft-output --player minseogong3 --rcon-password emgdemo --input-rate-hz 1000

센서와 확률까지 함께 확인하려면 다음 옵션을 추가합니다.

  --with-probabilities --with-sensors

Minecraft 입력 없이 판정 JSON만 확인하려면 다음처럼 실행합니다.

  py h10_live_motion_classifier.py --port COM6 --input-rate-hz 1000 --with-probabilities

[정상 출력 예]
  {"state":"walk","jump_started":false,"crouch_active":false,...}

[v9 오프라인 검증]
- 잠금 시험 stand/walk/jump macro-F1: 99.72%
- 잠금 시험 점프 사건: 21/21, 오검출 1, F1 97.67%
- 혼합 시험 state macro-F1: 98.33%
- 혼합 시험 walk: 296/296 tick 정답
- 혼합 시험 jump 상태: 175/175 tick 정답
- 혼합 시험 점프 사건: 12/12, 오검출 0, F1 100%
- 이륙 힘 피크 대비 점프 신호 중앙값: 76 ms 빠름
- Python 판정 계산: 중앙값 2.41 ms, p95 3.29 ms/tick (50 ms 주기)

주의: 위 수치는 같은 착용자/같은 수집 세션의 시간 홀드아웃 및 원본 뒤쪽
블록을 이어 붙인 혼합 프록시입니다. 실제 Windows CDC 연결, 새 착용일의 자유
전환, Minecraft 화면 동작까지 검증한 수치는 아닙니다.
