H10 연속 판정 v9 학습/검증 재현
================================

목표
----
- 센서 1 kHz 입력을 인과적으로 처리해 정확히 50 ms마다 결과 1개를 냅니다.
- 지속 상태(stand/walk/jump)와 순간 사건(jump_started)을 분리합니다.
- crouch_active는 양쪽 고관절이 학습된 최대 굴곡 plateau에 도달했을 때만 true입니다.

학습 코드
---------
  tools/h10_train_continuous_motion_classifier.py

혼합 연속 시험 코드
-------------------
  tools/h10_evaluate_mixed_motion.py

원본 11개 CSV가 C:\Users\<name>\Downloads에 있다면 저장소 루트에서 실행합니다.

  py tools\h10_train_continuous_motion_classifier.py ^
    --source-dir "%USERPROFILE%\Downloads" ^
    --output-dir outputs\h10_continuous_motion_classifier_v9

평가용 모델로 혼합 시험을 실행합니다. 전체 재학습 모델이 아니라 반드시
evaluation_model.json을 사용해야 시험용 뒤쪽 데이터가 학습에 들어가지 않습니다.

  py tools\h10_evaluate_mixed_motion.py ^
    --source-dir "%USERPROFILE%\Downloads" ^
    --model outputs\h10_continuous_motion_classifier_v9\evaluation_model.json ^
    --output-dir outputs\h10_mixed_motion_test_v9

분할 계약
---------
- 각 원본 파일 앞 60%: 학습
- 60~78%: 모델 및 점프 문턱 선택
- 마지막 22%: 잠금 시험
- 분할 경계 양쪽 400 ms: 제외
- 50 ms 창을 무작위로 섞지 않음

점프 계약
---------
- 1 kHz acc_norm을 21 ms median 처리
- acc_norm < 1.5 m/s^2 저중력 비행점을 물리 사건으로 검출
- 비행 전 600 ms에서 이륙 힘 피크를 찾음
- 20/40/80/160/320 ms 인과 특징 사용
- 양쪽 고관절이 같은 방향으로 움직이는 jump와 반대로 움직이는 walk/run을 구분
- state 모델과 jump_started 사건 모델을 분리해, state가 jump로 유지되어도 다음
  점프의 rising edge에서 Space를 다시 한 번 보낼 수 있음

완전 웅크림/Shift 계약
----------------------
- 학습된 최대 굴곡은 stand 중립각 대비 sit_stand 양쪽 고관절 q95로 정의
- 양쪽이 그 span의 95% 이상이고 |속도| <= 30 deg/s인 판정이 2회 연속일 때만
  crouch_active=true (50 ms 주기이므로 100 ms 확인)
- 위 조건이 하나라도 깨지는 바로 다음 50 ms 결과에서 crouch_active=false
- 게임 출력은 crouch_active=true일 때만 Shift를 누르고, false면 무조건 Shift를 뗌
- 0.92/0.95/0.97 비교에서 0.95는 최대 굴곡 의미를 강화하면서 시험 tail의
  2/2 plateau 검출을 유지했고, 비웅크림 10개 파일 8.23분에서 오검출 0

v9 오프라인 잠금 시험
---------------------
- stand/walk/jump accuracy: 99.78%
- stand/walk/jump macro-F1: 99.72%
- 점프 사건: 21/21 검출, 오검출 1회, F1 97.67%
- 이륙 힘 피크 대비 점프 신호 중앙값: 69 ms 빠름

v9 혼합 50 ms 시험
-------------------
- runtime reset: 0회
- state macro-F1: 98.33%
- walk: 296/296 tick 정답
- jump 상태: 175/175 tick 정답
- 점프 사건: 12/12 검출, 오검출 0회, F1 100%
- 이륙 힘 피크 대비 점프 신호 중앙값: 76 ms 빠름
- 완전 웅크림: 두 plateau 모두 검출, 다른 동작 블록에서 Shift 오검출 0
- 완전 웅크림의 50 ms 경계 일치: precision/recall 94.29% (각 2 tick 경계 차이)

산출물
------
- evaluation_model.json: 앞 60% 학습만 사용한 잠금 시험/혼합 검증용
- h10_stream_motion_model.json: 잠금 시험 뒤 전체 원본으로 재학습한 배포용
- evaluation.json: 잠금 시험 지표
- test_50ms_predictions.csv: 마지막 22%의 50 ms 판정
- jump_event_manifest.csv: 물리 점프 사건 정답
- provenance.json: 원본 해시와 학습 계약

한계
----
이 수치는 같은 착용자·같은 수집 세션의 시간 홀드아웃과 원본 뒤쪽 블록을 이어
붙인 혼합 프록시입니다. 새 착용일의 자유 동작, Windows USB CDC, 실제 Minecraft
화면에서의 성공률을 대신하지 않습니다.
