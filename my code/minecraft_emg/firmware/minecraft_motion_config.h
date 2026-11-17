#ifndef MINECRAFT_MOTION_CONFIG_H
#define MINECRAFT_MOTION_CONFIG_H

/*
 * minecraft_motion_config.h
 *
 * 실제 H10 로그가 없는 상태에서 정한 [설계] 시작값이다. 착용 실측에서 축·부호와
 * 분포를 확인하기 전에는 이 값으로 동작 성능을 주장하거나 토크에 연결하지 않는다.
 */

/* ---- 착용자 Body Data ---------------------------------------------------
 * 0인 항목이 하나라도 있으면 XM_SendUserBodyData()를 호출하지 않고 판정을 닫는다.
 * 임의의 예제 체형을 실제 착용자 값처럼 보내지 않기 위한 fail-closed 기본값이다.
 */
#ifndef MINECRAFT_BODY_WEIGHT_G
#define MINECRAFT_BODY_WEIGHT_G             75000U
#endif
#ifndef MINECRAFT_BODY_HEIGHT_MM
#define MINECRAFT_BODY_HEIGHT_MM            1820U
#endif
#ifndef MINECRAFT_BODY_RIGHT_THIGH_MM
#define MINECRAFT_BODY_RIGHT_THIGH_MM       360U
#endif
#ifndef MINECRAFT_BODY_LEFT_THIGH_MM
#define MINECRAFT_BODY_LEFT_THIGH_MM        360U
#endif
#ifndef MINECRAFT_BODY_RIGHT_SHANK_MM
#define MINECRAFT_BODY_RIGHT_SHANK_MM       330U
#endif
#ifndef MINECRAFT_BODY_LEFT_SHANK_MM
#define MINECRAFT_BODY_LEFT_SHANK_MM        330U
#endif
#ifndef MINECRAFT_BODY_RIGHT_ANKLE_MM
#define MINECRAFT_BODY_RIGHT_ANKLE_MM       110U
#endif
#ifndef MINECRAFT_BODY_LEFT_ANKLE_MM
#define MINECRAFT_BODY_LEFT_ANKLE_MM        110U
#endif

/* ---- 센서 축 부호 -------------------------------------------------------
 * 실제 H10에서 굴곡·시계방향 부호를 확인한 뒤 +1.0f 또는 -1.0f로 고정한다.
 */
#define MINECRAFT_HIP_FLEXION_SIGN          (+1.0f)
#define MINECRAFT_KNEE_FLEXION_SIGN         (+1.0f)
#define MINECRAFT_YAW_CLOCKWISE_SIGN        (+1.0f)

/* ---- 초기 중립 자세 보정 ------------------------------------------------ */
#define MINECRAFT_CALIBRATION_MS            1500U
#define MINECRAFT_CALIBRATION_MIN_SAMPLES   100U
#define MINECRAFT_CALIBRATION_MAX_GYRO_DPS  8.0f
#define MINECRAFT_SAMPLE_STALE_MS           100U

/* ---- 제자리걸음·제자리뛰기 -------------------------------------------- */
#define MINECRAFT_STEP_INTERVAL_MIN_MS      180U
#define MINECRAFT_STEP_INTERVAL_MAX_MS      1300U
#define MINECRAFT_WALK_CADENCE_MIN_SPM      55.0f
#define MINECRAFT_RUN_CADENCE_MIN_SPM       150.0f
#define MINECRAFT_RUN_CONFIDENCE_FULL_SPM   270.0f
#define MINECRAFT_HOP_CADENCE_MIN_SPM       75.0f
#define MINECRAFT_MOTION_ENTER_DWELL_MS     120U
#define MINECRAFT_MOTION_STOP_TIMEOUT_MS    850U
#define MINECRAFT_FLIGHT_DWELL_MS           25U
#define MINECRAFT_JUMP_OUTPUT_HOLD_MS       120U
#define MINECRAFT_RUN_EVIDENCE_HOLD_MS      450U
#define MINECRAFT_JUMP_ACCEL_DELTA_MPS2     2.5f

/* ---- 회전 --------------------------------------------------------------- */
#define MINECRAFT_TURN_FILTER_TAU_MS        80.0f
#define MINECRAFT_TURN_ENTER_DPS            18.0f
#define MINECRAFT_TURN_EXIT_DPS             8.0f
#define MINECRAFT_TURN_FULL_SCALE_DPS       100.0f

/* ---- 실제 앉기·웅크리기 ------------------------------------------------ */
#define MINECRAFT_CROUCH_HIP_ENTER_DEG      20.0f
#define MINECRAFT_CROUCH_KNEE_ENTER_DEG     25.0f
#define MINECRAFT_CROUCH_HIP_EXIT_DEG       12.0f
#define MINECRAFT_CROUCH_KNEE_EXIT_DEG      15.0f
#define MINECRAFT_CROUCH_ENTER_DWELL_MS     250U
#define MINECRAFT_CROUCH_EXIT_DWELL_MS      180U

/* ---- USB 게임 명령 스트림 ---------------------------------------------- */
#define MINECRAFT_USB_MODULE_ID             0xF4U
#define MINECRAFT_USB_PERIOD_MS             20U

#endif /* MINECRAFT_MOTION_CONFIG_H */
