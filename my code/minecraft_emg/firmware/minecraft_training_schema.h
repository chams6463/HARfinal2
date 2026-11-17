#ifndef MINECRAFT_TRAINING_SCHEMA_H
#define MINECRAFT_TRAINING_SCHEMA_H

#include <stdint.h>

#define MINECRAFT_TRAINING_SCHEMA_VERSION 1U

typedef enum {
    MINECRAFT_LABEL_STILL = 0,
    MINECRAFT_LABEL_WALK_LOW,
    MINECRAFT_LABEL_WALK_MID,
    MINECRAFT_LABEL_WALK_HIGH,
    MINECRAFT_LABEL_JUMP_LOW,
    MINECRAFT_LABEL_JUMP_MID,
    MINECRAFT_LABEL_JUMP_HIGH,
    MINECRAFT_LABEL_CROUCH,
    MINECRAFT_LABEL_COUNT
} MinecraftTrainingLabel_t;

/*
 * XM10 USB CDC에서 PC로 보내는 한 표본. packed 구조와 PC decoder를 함께 바꾼다.
 * leftHipTorque/rightHipTorque는 이름과 달리 SDK에서 모터 전류 [A]다.
 */
typedef struct __attribute__((packed)) {
    uint32_t sample_ms;
    uint32_t session_start_ms;
    uint32_t sample_index;
    uint32_t h10_assist_loop_count;
    uint32_t h10_post_processing_count;

    uint16_t trial_index;
    uint16_t schema_version;

    uint8_t participant_id;
    uint8_t label_id;
    uint8_t recording;
    uint8_t h10_connected;
    uint8_t h10_mode;
    uint8_t h10_assist_level;
    uint8_t h10_fsm_state;
    uint8_t neutral_position_set;
    uint8_t pvector_r_done;
    uint8_t pvector_l_done;
    uint8_t contact_l;
    uint8_t contact_r;

    float pelvic_angle_deg;
    float hip_angle_l_deg;
    float hip_angle_r_deg;
    float thigh_angle_l_deg;
    float thigh_angle_r_deg;
    float knee_angle_l_deg;
    float knee_angle_r_deg;
    float forward_velocity_mps;

    float hip_motor_current_l_a;
    float hip_motor_current_r_a;
    float hip_motor_angle_l_deg;
    float hip_motor_angle_r_deg;

    float imu_roll_l_deg;
    float imu_roll_r_deg;
    float imu_pitch_l_deg;
    float imu_pitch_r_deg;
    float imu_yaw_l_deg;
    float imu_yaw_r_deg;

    float imu_acc_x_l_mps2;
    float imu_acc_y_l_mps2;
    float imu_acc_z_l_mps2;
    float imu_acc_x_r_mps2;
    float imu_acc_y_r_mps2;
    float imu_acc_z_r_mps2;

    float imu_gyr_x_l_dps;
    float imu_gyr_y_l_dps;
    float imu_gyr_z_l_dps;
    float imu_gyr_x_r_dps;
    float imu_gyr_y_r_dps;
    float imu_gyr_z_r_dps;
} MinecraftTrainingRecord_t;

_Static_assert(sizeof(MinecraftTrainingRecord_t) == 156U,
               "MinecraftTrainingRecord_t wire size must remain 156 bytes");

#endif /* MINECRAFT_TRAINING_SCHEMA_H */
