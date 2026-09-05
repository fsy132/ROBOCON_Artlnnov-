#pragma once

#include <stdint.h>
#include "dvc_motor.h"

typedef enum
{
    ACTION_STATUS_IDLE = 0,
    ACTION_STATUS_RUNNING = 1,
    ACTION_STATUS_DONE = 2,
    ACTION_STATUS_FAIL = 3,
} ActionStatus_t;

typedef enum
{
    ACTION_ID_NONE = 0,
    ACTION_ID_CLIMB_UP_LOW = 1,
    ACTION_ID_CLIMB_UP_HIGH = 2,
    ACTION_ID_CLIMB_DOWN_LOW = 3,
    ACTION_ID_CLIMB_DOWN_HIGH = 4,
    ACTION_ID_BLOCK_LOW_TO_HIGH_PICK = 5,
    ACTION_ID_BLOCK_HIGH_TO_LOW_PICK = 6,
    ACTION_ID_BLOCK_SUCK = 7,
    ACTION_ID_BLOCK_STORE = 8,
    ACTION_ID_BLOCK_PLACE = 9,
    ACTION_ID_BLOCK_TAKE_OUT_STORAGE = 10,
    ACTION_ID_CLIMB_UP_LOW_FINISH = 11,
    ACTION_ID_CLIMB_UP_LOW_GRAB = 13,
    ACTION_ID_CLIMB_UP_HIGH_GRAB = 14,
    ACTION_ID_GRIPPER_PICK = 15,
    ACTION_ID_GRIPPER_RELEASE = 16,
    ACTION_ID_CONFRONT_BLOCK_PLACE = 17,
    ACTION_ID_MAX = ACTION_ID_CONFRONT_BLOCK_PLACE,
} ActionId_t;

typedef enum
{
    ACTION_STEP_IDLE = 0,
    ACTION_STEP_CHASSIS_1,
    ACTION_STEP_LIFT_1,
    ACTION_STEP_SIDE,
    ACTION_STEP_LIFT_2,
    ACTION_STEP_CHASSIS_2,
    ACTION_STEP_LIFT_3,
    ACTION_STEP_BLOCK_LIFT_1,
    ACTION_STEP_BLOCK_LIFT_2,
    ACTION_STEP_BLOCK_GPIO_1,
    ACTION_STEP_BLOCK_GPIO_2,
    ACTION_STEP_BLOCK_WAIT_1,
    ACTION_STEP_BLOCK_MOTOR_1,
    ACTION_STEP_BLOCK_MOTOR_2,
    ACTION_STEP_DONE,
    ACTION_STEP_FAIL,
} ActionStep_t;

extern Class_Motor_C620 motor1;
extern Class_Motor_C620 motor2;
extern Class_Motor_C620 motor3;
extern Class_Motor_C620 motor4;
extern Class_Motor_C620 motor5;
extern Class_Motor_C620 motor6;
extern Class_Motor_C620 motor7;
extern Class_Motor_C620 motor8;
extern Class_Motor_C610 motor9;
extern Class_Motor_C610 motor10;
extern Class_Motor_C610 motor11;
extern Class_Motor_C620 motor12;
extern Class_Motor_C620 motor13;
extern Class_Motor_C620 motor14;
extern Class_Motor_C610 motor15;

extern char uart6_fb_buf[96];
extern uint32_t uart6_fb_last_tick;

extern float chassis_cmd_vx_mm_s;
extern float chassis_cmd_vy_mm_s;
extern float chassis_cmd_w_mdeg_s;
extern uint32_t chassis_cmd_last_tick;
extern float vx_f;
extern float vy_f;
extern float w_f;

extern uint8_t lift1_cmd;
extern uint8_t lift2_cmd;
extern uint8_t lift3_cmd;
extern uint8_t lift1_zero_inited;
extern uint8_t lift2_zero_inited;
extern uint8_t lift12_home_active;
#if 0
extern uint8_t lift12_home_request;
extern uint8_t lift12_home_done;
extern uint8_t lift12_home_fail;
#endif

extern float can2_2006_target_angle_12;

extern uint8_t side_wheel_enable;
extern float side_wheel_target_rpm;

extern uint8_t active_mtf01_index;
extern uint8_t side_distance_enable;
extern float side_distance_target_mm;
extern float side_distance_now_mm;
extern uint32_t side_distance_last_tick;
extern uint32_t mtf01_last_update_tick;
extern uint32_t mtf01_last_update_ms;

extern uint8_t hwt101_rx_data;
extern float hwt101_gyro_z_dps;
extern float hwt101_yaw_deg;
extern float hwt101_yaw_zero_deg;
extern uint8_t hwt101_angle_valid;
extern uint32_t hwt101_last_update_tick;
extern uint8_t chassis_yaw_hold_enable;
extern float chassis_yaw_hold_target_deg;
extern uint8_t chassis_gyro_turn_enable;
extern float chassis_gyro_turn_target_deg;

extern uint8_t action_status;
extern uint8_t action_id;
extern uint8_t action_step;
extern uint32_t action_step_start_tick;
extern uint32_t action_fb_last_tick;

void chassis_drive_pid_use_climb(uint8_t enable);
void chassis_drive_pid_restore_requested(void);

extern uint8_t chassis_distance_enable;
extern uint8_t chassis_distance_mtf_index;
extern float chassis_distance_target_mm;
extern float chassis_distance_now_mm;
extern uint32_t chassis_distance_last_tick;
extern uint32_t chassis_distance_mtf_last_update_tick;
extern uint32_t chassis_distance_mtf_last_update_ms;
extern uint8_t chassis_distance_failed;
extern uint8_t side_distance_failed;
