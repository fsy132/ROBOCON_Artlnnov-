/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_bsp.h"
#include "drv_math.h"
#include "dvc_serialplot.h"
#include "dvc_motor.h"
#include <stdio.h>
#include <string.h>

#include <stdlib.h>
#include <math.h>

#include "mtf01.h"
#include "app_config.h"
#include "app_shared.h"
#include "hwt101.h"
#include "distance_control.h"
#include "action_group.h"
#include "protocol.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Project configuration lives in app_config.h.
#ifndef CHASSIS_FORCE_LOCK_IGNORE_MS
#define CHASSIS_FORCE_LOCK_IGNORE_MS 300U
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ==================== UART6 上位机指令接收 ====================
// Host command UART is now bound to USART3. Old uart6_* variable names are kept to reduce churn.
uint8_t uart6_rx_data = 0;
char uart6_rx_line[80];
volatile uint8_t uart6_rx_index = 0;
volatile uint8_t uart6_rx_complete = 0;
uint32_t uart6_rx_last_tick = 0;
volatile uint32_t protocol_heartbeat_ack_count = 0;
volatile uint32_t protocol_handshake_hash = 0;
volatile uint8_t protocol_handshake_tx_pending = 0;
volatile uint8_t protocol_heartbeat_tx_pending = 0;
volatile uint8_t protocol_action_cmd_pending = 0;
volatile uint8_t protocol_action_cmd_id = 0;

// ==================== UART6 / UART2 状态反馈 ====================
uint32_t uart6_fb_last_tick = 0;
uint8_t uart3_motor_debug_enable = 0;
#if UART2_FORWARD_UART3_RX || UART2_HOST_ASCII_DEBUG
uint8_t uart2_forward_buf[UART2_FORWARD_BUF_SIZE];
volatile uint16_t uart2_forward_head = 0;
volatile uint16_t uart2_forward_tail = 0;
#endif

// ==================== 电机对象 ====================
Class_Motor_C620 motor1;
Class_Motor_C620 motor2;
Class_Motor_C620 motor3;
Class_Motor_C620 motor4;

Class_Motor_C620 motor5;
Class_Motor_C620 motor6;
Class_Motor_C620 motor7;
Class_Motor_C620 motor8;

Class_Motor_C610 motor9;
Class_Motor_C610 motor10;
Class_Motor_C610 motor11;
#if 0

Class_Motor_C610 motor11; // CAN1 0x207，新增 2006，速度控制
Class_Motor_C620 motor12; // CAN2 0x205，3508，角度控制

Class_Motor_C620 motor13; // CAN2 0x206，升降台3 第一个 3508
Class_Motor_C620 motor14; // CAN2 0x207，升降台3 第二个 3508

// ==================== UART6 指令目标量 ====================
#endif
Class_Motor_C620 motor12;
Class_Motor_C620 motor13;
Class_Motor_C620 motor14;
Class_Motor_C610 motor15;

float chassis_cmd_vx_mm_s = 0.0f;
float chassis_cmd_vy_mm_s = 0.0f;
float chassis_cmd_w_mdeg_s = 0.0f;
uint32_t chassis_cmd_last_tick = 0;
volatile uint8_t chassis_force_lock_until_motion = 0;
uint32_t chassis_force_lock_tick = 0;

uint8_t lift1_cmd = 0;
uint8_t lift2_cmd = 0;

// ==================== 底盘目标轮角 ====================
float wheel_target_angle_1 = 0.0f;
float wheel_target_angle_2 = 0.0f;
float wheel_target_angle_3 = 0.0f;
float wheel_target_angle_4 = 0.0f;
uint8_t wheel_angle_target_inited = 0;

// ==================== 底盘速度平滑滤波变量 ====================
float vx_f = 0.0f;
float vy_f = 0.0f;
float w_f  = 0.0f;

// ==================== 底盘实际速度反馈 ====================
float chassis_now_vx = 0.0f;
float chassis_now_vy = 0.0f;
float chassis_now_wz = 0.0f;
float chassis_yaw = 0.0f;
float chassis_odom_x = 0.0f;
float chassis_odom_y = 0.0f;
uint32_t chassis_fb_last_update_tick = 0;

static float chassis_abs_f(float value)
{
    return value >= 0.0f ? value : -value;
}

static float chassis_vel_fb_deadband(float value)
{
    return (chassis_abs_f(value) < CHASSIS_VEL_FB_DEADBAND_M_S) ? 0.0f : value;
}

static uint8_t chassis_cmd_nonzero(float vx_mm_s, float vy_mm_s, float w_mdeg_s)
{
    return (chassis_abs_f(vx_mm_s) > 1.0f) ||
           (chassis_abs_f(vy_mm_s) > 1.0f) ||
           (chassis_abs_f(w_mdeg_s) > 10.0f);
}

static uint8_t chassis_drive_pid_climb_mode = 0xFF;
static uint8_t chassis_confront_climb_mode = 0;

void chassis_drive_pid_use_climb(uint8_t enable)
{
    float angle_kp = 0.0f;
    float omega_kp = 0.0f;
    float omega_ki = 0.0f;
    float omega_i_max = 0.0f;
    float omega_out_max = 0.0f;

    enable = enable ? 1U : 0U;
    if (chassis_drive_pid_climb_mode == enable)
    {
        return;
    }

    if (enable)
    {
        angle_kp = CHASSIS_PID_CLIMB_ANGLE_KP;
        omega_kp = CHASSIS_PID_CLIMB_OMEGA_KP;
        omega_ki = CHASSIS_PID_CLIMB_OMEGA_KI;
        omega_i_max = CHASSIS_PID_CLIMB_OMEGA_I_MAX;
        omega_out_max = CHASSIS_PID_CLIMB_OMEGA_OUT_MAX;
    }
    else
    {
        angle_kp = CHASSIS_PID_FLAT_ANGLE_KP;
        omega_kp = CHASSIS_PID_FLAT_OMEGA_KP;
        omega_ki = CHASSIS_PID_FLAT_OMEGA_KI;
        omega_i_max = CHASSIS_PID_FLAT_OMEGA_I_MAX;
        omega_out_max = CHASSIS_PID_FLAT_OMEGA_OUT_MAX;
    }

    motor1.PID_Angle.Init(angle_kp, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, 0.001f, 0.0f);
    motor2.PID_Angle.Init(angle_kp, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, 0.001f, 0.0f);
    motor3.PID_Angle.Init(angle_kp, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, 0.001f, 0.0f);
    motor4.PID_Angle.Init(angle_kp, 0.0f, 0.0f, 0.0f, 0.0f, MAX_WHEEL_SPEED, 0.001f, 0.0f);

    motor1.PID_Omega.Init(omega_kp, omega_ki, 0.0f, 0.0f, omega_i_max, omega_out_max, 0.001f, 0.0f);
    motor2.PID_Omega.Init(omega_kp, omega_ki, 0.0f, 0.0f, omega_i_max, omega_out_max, 0.001f, 0.0f);
    motor3.PID_Omega.Init(omega_kp, omega_ki, 0.0f, 0.0f, omega_i_max, omega_out_max, 0.001f, 0.0f);
    motor4.PID_Omega.Init(omega_kp, omega_ki, 0.0f, 0.0f, omega_i_max, omega_out_max, 0.001f, 0.0f);

    chassis_drive_pid_climb_mode = enable;
}

void chassis_drive_pid_restore_requested(void)
{
    chassis_drive_pid_use_climb(chassis_confront_climb_mode);
}

static uint8_t action_id_locks_chassis(uint8_t id)
{
    return ((id >= ACTION_ID_BLOCK_LOW_TO_HIGH_PICK) &&
            (id <= ACTION_ID_BLOCK_TAKE_OUT_STORAGE)) ||
           id == ACTION_ID_CONFRONT_BLOCK_PLACE;
}

#define HOST_ACTION_ID_PICK_WEAPON_TIP       1U
#define HOST_ACTION_ID_RELEASE_WEAPON_TIP    2U
#define HOST_ACTION_ID_CHASSIS_LOCK          6U

static uint8_t host_action_id_to_local(uint8_t host_id)
{
    switch (host_id)
    {
        case HOST_ACTION_ID_PICK_WEAPON_TIP: // Upper computer: pick weapon tip.
            return ACTION_ID_GRIPPER_PICK;
        case HOST_ACTION_ID_RELEASE_WEAPON_TIP: // Upper computer: release weapon tip.
            return ACTION_ID_GRIPPER_RELEASE;

        default:
            return host_id;
    }
}

static uint8_t chassis_force_lock_ignore_active(void)
{
    return chassis_force_lock_until_motion &&
           ((uint32_t)(HAL_GetTick() - chassis_force_lock_tick) < CHASSIS_FORCE_LOCK_IGNORE_MS);
}

static void chassis_cmd_zero_now(void)
{
    chassis_cmd_vx_mm_s = 0.0f;
    chassis_cmd_vy_mm_s = 0.0f;
    chassis_cmd_w_mdeg_s = 0.0f;
    chassis_cmd_last_tick = HAL_GetTick();

    vx_f = 0.0f;
    vy_f = 0.0f;
    w_f = 0.0f;

    wheel_target_angle_1 = motor1.Get_Now_Angle();
    wheel_target_angle_2 = motor2.Get_Now_Angle();
    wheel_target_angle_3 = motor3.Get_Now_Angle();
    wheel_target_angle_4 = motor4.Get_Now_Angle();
    wheel_angle_target_inited = 1;

    motor1.Set_Target_Angle(wheel_target_angle_1);
    motor2.Set_Target_Angle(wheel_target_angle_2);
    motor3.Set_Target_Angle(wheel_target_angle_3);
    motor4.Set_Target_Angle(wheel_target_angle_4);
}

static float chassis_ramp_f(float current, float target, float accel_step, float decel_step)
{
    float delta = target - current;
    uint8_t decel =
        ((current > 0.0f && target < 0.0f) || (current < 0.0f && target > 0.0f) ||
         (chassis_abs_f(target) < chassis_abs_f(current)));
    float step = decel ? decel_step : accel_step;

    if (step <= 0.0f) return target;
    if (delta > step) return current + step;
    if (delta < -step) return current - step;
    return target;
}

// ==================== 升降台零点 ====================
float lift1_zero_angle_5 = 0.0f;
float lift1_zero_angle_6 = 0.0f;
uint8_t lift1_zero_inited = 0;
uint8_t lift12_home_active = 1;
#if 0
uint8_t lift12_home_request = 0;
uint8_t lift12_home_done = 0;
uint8_t lift12_home_fail = 0;
#endif

float lift2_zero_angle_7 = 0.0f;
float lift2_zero_angle_8 = 0.0f;
uint8_t lift2_zero_inited = 0;

// ==================== 2006 角度控制零点和相对目标 ====================
#if 0
float can1_2006_zero_angle_11 = 0.0f;
float can1_2006_target_omega_11 = 0.0f;
uint8_t can1_2006_zero_inited = 0;

#endif
float can2_2006_zero_angle_12 = 0.0f;
float can2_2006_target_angle_12 = 0.0f;
uint8_t can2_2006_zero_inited = 0;

uint8_t side_wheel_enable = 0;
float side_wheel_target_rpm = 0.0f;
uint32_t side_wheel_cmd_last_tick = 0;
uint32_t side_wheel15_feedback_last_tick = 0U;
// ==================== 升降台3零点 ====================
uint8_t lift3_cmd = 0;
float lift3_zero_angle_13 = 0.0f;
float lift3_zero_angle_14 = 0.0f;
uint8_t lift3_zero_inited = 0;

// ==================== MTF-01P 光流模块接收 ====================
uint8_t mtf01_rx_data_1 = 0;   // 1号光流，车中间，USART6
uint8_t mtf01_rx_data_2 = 0;   // 2号光流，车头，UART8

// MTF01 #1 (middle module) is bound to USART6; MTF01 #2 (front module) is bound to UART8.
uint8_t active_mtf01_index = MTF01_MIDDLE_INDEX;

// ==================== MTF-01P 位移闭环状态 ====================
uint8_t side_distance_enable = 0;
float side_distance_target_mm = 0.0f;
float side_distance_now_mm = 0.0f;
uint32_t side_distance_last_tick = 0;
uint32_t mtf01_last_update_tick = 0;
uint32_t mtf01_last_update_ms = 0;

float chassis_flow_lateral_mm = 0.0f;
float chassis_flow_correct_mm_s = 0.0f;
float chassis_flow_yaw_correct_wz = 0.0f;
float chassis_flow_lateral_vel_f = 0.0f;
uint32_t chassis_flow_mtf_update_tick = 0;
uint32_t chassis_flow_mtf_time_ms = 0;

// ==================== 陀螺仪变量 ====================
uint8_t hwt101_rx_data = 0;

float hwt101_gyro_z_dps = 0.0f;
float hwt101_yaw_deg = 0.0f;
float hwt101_yaw_zero_deg = 0.0f;
uint8_t hwt101_angle_valid = 0;
uint32_t hwt101_last_update_tick = 0;

uint8_t chassis_yaw_hold_enable = 0;
float chassis_yaw_hold_target_deg = 0.0f;
float chassis_yaw_hold_correct_wz = 0.0f;

uint8_t chassis_gyro_turn_enable = 0;
float chassis_gyro_turn_target_deg = 0.0f;

uint8_t sbus_rx_data = 0;
uint8_t sbus_rx_buf[25];
uint8_t sbus_rx_index = 0;
uint16_t sbus_ch[8];

// ==================== 状态机 ====================
// ==================== Action group state ====================
uint8_t action_status = ACTION_STATUS_IDLE;
uint8_t action_id = ACTION_ID_NONE;
uint8_t action_step = ACTION_STEP_IDLE;
uint32_t action_step_start_tick = 0;
uint32_t action_fb_last_tick = 0;

uint8_t chassis_distance_enable = 0;
uint8_t chassis_distance_mtf_index = MTF01_MIDDLE_INDEX;
float chassis_distance_target_mm = 0.0f;
float chassis_distance_now_mm = 0.0f;
uint32_t chassis_distance_last_tick = 0;
uint32_t chassis_distance_mtf_last_update_tick = 0;
uint32_t chassis_distance_mtf_last_update_ms = 0;

// ==================== 底盘距离闭环失败标志 ====================
uint8_t chassis_distance_failed = 0;
uint8_t side_distance_failed = 0;

static void chassis_force_lock_now(void)
{
    chassis_force_lock_until_motion = 1;
    chassis_force_lock_tick = HAL_GetTick();
    chassis_gyro_turn_enable = 0;
    chassis_yaw_hold_enable = 0;
    chassis_distance_enable = 0;
    chassis_distance_now_mm = 0.0f;
    chassis_distance_target_mm = 0.0f;
    side_distance_enable = 0;
    side_distance_now_mm = 0.0f;
    side_distance_target_mm = 0.0f;
    chassis_cmd_zero_now();
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
// ==================== 通信回调 ====================
void CAN1_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer);
void CAN2_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer);
void uart6_cmd_parse(char *line);
static void bus_servo_uart4_tx_task(void);
static void bus_servo_uart4_tx_done_isr(void);
static void bus_servo_move_time_write(uint8_t servo_id, uint16_t position, uint16_t time_ms);
static void uart3_tx_task(void);
static void uart3_tx_done_isr(void);
static void gripper_pick_action_start(void);
static void gripper_release_action_start(void);
static void gripper_pick_action_task(void);
static void gripper_release_action_task(void);
static void gripper_servo_boot_cancel(void);
static void gripper_servo_boot_task(void);
static void flip_servo_boot_cancel(void);
static void flip_servo_boot_task(void);
static uint8_t flip_servo_position_from_cmd(int32_t cmd, uint16_t *position);
static void gripper_servo_set(uint8_t grip);
static void flip_servo_set(uint16_t position);

// ==================== 底盘控制 ====================
void omni_move(float vx, float vy, float w);
void chassis_control_task(void);
void chassis_feedback_update(void);
void uart6_feedback_task(void);
void uart3_motor_debug_task(void);
void uart2_forward_task(void);
void sbus_chassis_task(void);

// ==================== 升降台控制 ====================
void lift1_control_task(void);
void lift2_control_task(void);
#if 0
void lift12_home_task(void);
#endif
void lift3_control_task(void);

// ==================== 2006 控制 ====================
void side_wheel_control_task(void);
void can2_2006_angle_task(void);
#if UART2_DEBUG_MTF || UART3_DEBUG_MTF
void uart2_mtf_feedback_task(void);
#endif

// ==================== 反馈输出 ====================

// ==================== 备用遥控器接口，可选保留 ====================
void move_front(float speed);
void move_back(float speed);
void move_left(float speed);
void move_right(float speed);
void turn_left(float speed);
void turn_right(float speed);
void stop(float speed);

// ==================== 初始化和周期任务 ====================
void uart_init_task(void);
void motor_pid_init(void);
void motor_can_init(void);
void motor_pid_task(void);

// ==================== MTF 距离闭环任务 ====================

// ==================== 陀螺仪 ====================

// ==================== 状态机 ====================


#if 0
static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "pa",
    "ia",
    "da",
    "po",
    "io",
    "do",
    "torque",
    "fx",
};
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define UART3_TX_QUEUE_NEXT(index) ((uint8_t)(((index) + 1U) % UART3_TX_QUEUE_SIZE))

static uint8_t uart3_tx_queue[UART3_TX_QUEUE_SIZE][UART3_TX_FRAME_MAX_LEN];
static uint16_t uart3_tx_len[UART3_TX_QUEUE_SIZE];
static volatile uint8_t uart3_tx_head = 0;
static volatile uint8_t uart3_tx_tail = 0;
static volatile uint8_t uart3_tx_busy = 0;

static void uart3_tx_try_start(void)
{
    if (uart3_tx_busy || uart3_tx_tail == uart3_tx_head)
    {
        return;
    }

    uart3_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart3,
                             uart3_tx_queue[uart3_tx_tail],
                             uart3_tx_len[uart3_tx_tail]) != HAL_OK)
    {
        uart3_tx_busy = 0;
    }
}

static uint8_t uart3_tx_enqueue(const uint8_t *data, uint16_t len)
{
    uint8_t next_head = UART3_TX_QUEUE_NEXT(uart3_tx_head);

    if (data == 0 || len == 0 || len > UART3_TX_FRAME_MAX_LEN)
    {
        return 0;
    }

    if (next_head == uart3_tx_tail)
    {
        return 0;
    }

    memcpy(uart3_tx_queue[uart3_tx_head], data, len);
    uart3_tx_len[uart3_tx_head] = len;
    uart3_tx_head = next_head;
    uart3_tx_try_start();
    return 1;
}

static void uart3_tx_done_isr(void)
{
    if (uart3_tx_tail != uart3_tx_head)
    {
        uart3_tx_tail = UART3_TX_QUEUE_NEXT(uart3_tx_tail);
    }

    uart3_tx_busy = 0;
    uart3_tx_try_start();
}

static void uart3_tx_task(void)
{
    if (uart3_tx_busy && huart3.gState == HAL_UART_STATE_READY)
    {
        uart3_tx_done_isr();
        return;
    }

    uart3_tx_try_start();
}

extern "C" void serial_write(const uint8_t* data, uint16_t len)
{
    (void)uart3_tx_enqueue(data, len);
}

#if UART2_FORWARD_UART3_RX || UART2_HOST_ASCII_DEBUG
static void uart2_forward_push(uint8_t data)
{
    uint16_t next = (uint16_t)((uart2_forward_head + 1U) % UART2_FORWARD_BUF_SIZE);

    if (next == uart2_forward_tail)
    {
        return;
    }

    uart2_forward_buf[uart2_forward_head] = data;
    uart2_forward_head = next;
}

static void uart2_forward_write(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uart2_forward_push(data[i]);
    }
}

void uart2_forward_task(void)
{
    uint8_t chunk[16];
    uint8_t len = 0;

    while ((uart2_forward_tail != uart2_forward_head) && (len < sizeof(chunk)))
    {
        chunk[len++] = uart2_forward_buf[uart2_forward_tail];
        uart2_forward_tail = (uint16_t)((uart2_forward_tail + 1U) % UART2_FORWARD_BUF_SIZE);
    }

    if (len > 0U)
    {
        HAL_UART_Transmit(&huart2, chunk, len, 2);
    }
}
#else
void uart2_forward_task(void)
{
}
#endif

#if UART2_HOST_ASCII_DEBUG
static void uart2_ascii_append_i32(char *buf, uint16_t *pos, int32_t value)
{
    char tmp[12];
    uint8_t len = 0;
    uint32_t v;

    if (value < 0)
    {
        buf[(*pos)++] = '-';
        v = (uint32_t)(-value);
    }
    else
    {
        v = (uint32_t)value;
    }

    do
    {
        tmp[len++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v > 0U);

    while (len > 0U)
    {
        buf[(*pos)++] = tmp[--len];
    }
}

static void uart2_ascii_append_text(char *buf, uint16_t *pos, const char *text)
{
    while (*text)
    {
        buf[(*pos)++] = *text++;
    }
}

static void uart2_ascii_send_cmdvel(int32_t forward_mm_s, int32_t left_mm_s, int32_t wz_mdeg_s)
{
    char msg[64];
    uint16_t pos = 0;

    uart2_ascii_append_text(msg, &pos, "$RXCHS,");
    uart2_ascii_append_i32(msg, &pos, forward_mm_s);
    msg[pos++] = ',';
    uart2_ascii_append_i32(msg, &pos, left_mm_s);
    msg[pos++] = ',';
    uart2_ascii_append_i32(msg, &pos, wz_mdeg_s);
    msg[pos++] = '\r';
    msg[pos++] = '\n';

    uart2_forward_write((uint8_t *)msg, pos);
}

static void uart2_ascii_send_u8(const char *tag, uint8_t value)
{
    char msg[32];
    uint16_t pos = 0;

    uart2_ascii_append_text(msg, &pos, tag);
    uart2_ascii_append_i32(msg, &pos, value);
    msg[pos++] = '\r';
    msg[pos++] = '\n';

    uart2_forward_write((uint8_t *)msg, pos);
}

static void uart2_ascii_send_u32(const char *tag, uint32_t value)
{
    char msg[48];
    uint16_t pos = 0;

    uart2_ascii_append_text(msg, &pos, tag);
    uart2_ascii_append_i32(msg, &pos, (int32_t)value);
    msg[pos++] = '\r';
    msg[pos++] = '\n';

    uart2_forward_write((uint8_t *)msg, pos);
}
#endif

extern "C" void on_receive_Heartbeat(const Packet_Heartbeat* pkt)
{
    protocol_heartbeat_ack_count = pkt->count;
    protocol_heartbeat_tx_pending = 1;
#if UART2_HOST_ASCII_DEBUG
    uart2_ascii_send_u32("$RXHB,", pkt->count);
#endif
}

extern "C" void on_receive_Handshake(const Packet_Handshake* pkt)
{
    protocol_handshake_hash = PROTOCOL_HASH;
    if (protocol_handshake_tx_pending < 3)
    {
        protocol_handshake_tx_pending++;
    }
#if UART2_HOST_ASCII_DEBUG
    uart2_ascii_send_u32("$RXHS,", pkt->protocol_hash);
#endif
}

extern "C" void on_receive_CmdVel(const Packet_CmdVel* pkt)
{
    float vx_mm_s = pkt->vy * 1000.0f;
    float vy_mm_s = pkt->vx * 1000.0f;
    float w_mdeg_s = pkt->rotation * 1000.0f;

#if UART2_HOST_ASCII_DEBUG
    uart2_ascii_send_cmdvel((int32_t)(pkt->vx * 1000.0f),
                            (int32_t)(pkt->vy * 1000.0f),
                            (int32_t)(pkt->rotation * 1000.0f));
#endif

    if (chassis_force_lock_ignore_active())
    {
        chassis_cmd_zero_now();
        return;
    }

    if (chassis_force_lock_until_motion)
    {
        if (!chassis_cmd_nonzero(vx_mm_s, vy_mm_s, w_mdeg_s))
        {
            chassis_cmd_zero_now();
            return;
        }
        chassis_force_lock_until_motion = 0;
    }

    chassis_cmd_vx_mm_s = vx_mm_s;
    chassis_cmd_vy_mm_s = vy_mm_s;
    chassis_cmd_w_mdeg_s = w_mdeg_s;
    chassis_cmd_last_tick = HAL_GetTick();
}

extern "C" void on_receive_ActionGroupCmd(const Packet_ActionGroupCmd* pkt)
{
    if (pkt->action_id == HOST_ACTION_ID_CHASSIS_LOCK)
    {
#if UART2_HOST_ASCII_DEBUG
        uart2_ascii_send_u8("$RXLOCK,", pkt->action_id);
#endif
        if (chassis_force_lock_until_motion)
        {
            chassis_cmd_zero_now();
        }
        else
        {
            chassis_force_lock_now();
        }
        protocol_action_cmd_id = ACTION_ID_NONE;
        protocol_action_cmd_pending = 0;
        return;
    }

    uint8_t local_id = host_action_id_to_local(pkt->action_id);

    if (local_id <= ACTION_ID_MAX)
    {
#if UART2_HOST_ASCII_DEBUG
        uart2_ascii_send_u8("$RXACT,", local_id);
#endif

        if (local_id == ACTION_ID_GRIPPER_PICK)
        {
            gripper_pick_action_start();
            return;
        }
        if (local_id == ACTION_ID_GRIPPER_RELEASE)
        {
            gripper_release_action_start();
            return;
        }

        if (action_id_locks_chassis(local_id))
        {
            chassis_force_lock_now();
        }

        protocol_action_cmd_id = local_id;
        protocol_action_cmd_pending = 1;
    }
}

extern "C" void on_receive_ConfrontClimbCmd(const Packet_ConfrontClimbCmd* pkt)
{
    chassis_confront_climb_mode = pkt->mode ? 1U : 0U;

    if (action_status != ACTION_STATUS_RUNNING)
    {
        chassis_drive_pid_restore_requested();
    }
}

extern "C" void on_receive_WeaponFlipCmd(const Packet_WeaponFlipCmd* pkt)
{
    uint16_t position = 0;

    if (!flip_servo_position_from_cmd((int32_t)pkt->flip, &position))
    {
        return;
    }

    flip_servo_set(position);
}

static inline void sbus_decode_byte(uint8_t data)
{
#if RC_SBUS_ENABLE
    if (sbus_rx_index == 0 && data != 0x0F)
    {
        return;
    }

    sbus_rx_buf[sbus_rx_index++] = data;

    if (sbus_rx_index < 25)
    {
        return;
    }

    sbus_rx_index = 0;

    sbus_ch[0] = ((sbus_rx_buf[1] | sbus_rx_buf[2] << 8) & 0x07FF);
    sbus_ch[1] = ((sbus_rx_buf[2] >> 3 | sbus_rx_buf[3] << 5) & 0x07FF);
    sbus_ch[3] = ((sbus_rx_buf[5] >> 1 | sbus_rx_buf[6] << 7) & 0x07FF);
    sbus_ch[4] = ((sbus_rx_buf[6] >> 4 | sbus_rx_buf[7] << 4) & 0x07FF);
    sbus_ch[5] = ((sbus_rx_buf[7] >> 7 | sbus_rx_buf[8] << 1 | sbus_rx_buf[9] << 9) & 0x07FF);
#else
    (void)data;
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        sbus_decode_byte(sbus_rx_data);
        HAL_UART_Receive_IT(&huart1, &sbus_rx_data, 1);
    }
    else if (huart->Instance == USART3)
    {
        static uint8_t uart6_binary_state = 0;
        static uint8_t uart6_binary_len = 0;
        static uint8_t uart6_binary_count = 0;

        uart6_rx_last_tick = HAL_GetTick();

#if UART2_FORWARD_UART3_RX
        uart2_forward_push(uart6_rx_data);
#endif

        if (uart6_binary_state || uart6_rx_data == FRAME_HEADER1)
        {
            protocol_fsm_feed(uart6_rx_data);

            switch (uart6_binary_state)
            {
                case 0:
                    uart6_binary_state = (uart6_rx_data == FRAME_HEADER1) ? 1 : 0;
                    break;
                case 1:
                    uart6_binary_state = (uart6_rx_data == FRAME_HEADER2) ? 2 : 0;
                    break;
                case 2:
                    uart6_binary_state = 3;
                    break;
                case 3:
                    uart6_binary_len = uart6_rx_data;
                    uart6_binary_count = 0;
                    uart6_binary_state = (uart6_binary_len == 0) ? 5 : 4;
                    break;
                case 4:
                    uart6_binary_count++;
                    if (uart6_binary_count >= uart6_binary_len)
                    {
                        uart6_binary_state = 5;
                    }
                    break;
                default:
                    uart6_binary_state = 0;
                    break;
            }

            HAL_UART_Receive_IT(&huart3, &uart6_rx_data, 1);
            return;
        }

        if (!uart6_rx_complete)
        {
            if (uart6_rx_data == '\n' || uart6_rx_data == '\r')
            {
                if (uart6_rx_index > 0)
                {
                    uart6_rx_line[uart6_rx_index] = '\0';
                    uart6_rx_complete = 1;
                    uart6_rx_index = 0;
                }
            }
            else
            {
                if (uart6_rx_index < sizeof(uart6_rx_line) - 1)
                {
                    uart6_rx_line[uart6_rx_index++] = (char)uart6_rx_data;
                }
                else
                {
                    uart6_rx_index = 0;
                }
            }
        }

        HAL_UART_Receive_IT(&huart3, &uart6_rx_data, 1);
    }
    else if (huart->Instance == USART6)
    {
        mtf01_decode(0, mtf01_rx_data_1);
        HAL_UART_Receive_IT(&huart6, &mtf01_rx_data_1, 1);
    }
    else if (huart->Instance == UART7)
    {
        hwt101_decode(hwt101_rx_data);
#if UART2_FORWARD_HWT101_RAW
        HAL_UART_Transmit(&huart2, &hwt101_rx_data, 1, 1);
#endif
        HAL_UART_Receive_IT(&huart7, &hwt101_rx_data, 1);
    }
    else if (huart->Instance == UART8)
    {
        mtf01_decode(1, mtf01_rx_data_2);
        HAL_UART_Receive_IT(&huart8, &mtf01_rx_data_2, 1);
    }
}
/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 * 
 * * @return void
 * @note 处理不同ID的电机CAN数据。
 */
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uart3_tx_done_isr();
    }
    else if (huart->Instance == UART4)
    {
        bus_servo_uart4_tx_done_isr();
    }
}

void CAN1_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
       case (0x201):
        {
            motor1.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x202):
        {
            motor2.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x203):
        {
            motor3.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x204):
        {
            motor4.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x205):
        {
        motor9.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case (0x206):
        {
        motor10.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case (0x207):
        {
        motor11.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case (0x208):
        {
        motor12.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}


void CAN2_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        case 0x201:
        {
            motor5.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case 0x202:
        {
            motor6.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case 0x203:
        {
            motor7.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case 0x204:
        {
            motor8.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case 0x205:
        {
        motor15.CAN_RxCpltCallback(Rx_Buffer->Data);
        side_wheel15_feedback_last_tick = HAL_GetTick();
        }
        break;
        case 0x206:
        {
        motor13.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

        case 0x207:
        {
        motor14.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}

// ==================== UART6 上位机指令解析 ====================

#define BUS_SERVO_TX_FRAME_LEN 10U

static uint8_t bus_servo_tx_queue[BUS_SERVO_TX_QUEUE_SIZE][BUS_SERVO_TX_FRAME_LEN];
static volatile uint8_t bus_servo_tx_head = 0;
static volatile uint8_t bus_servo_tx_tail = 0;
static volatile uint8_t bus_servo_tx_busy = 0;

static uint8_t bus_servo_tx_next(uint8_t index)
{
    return (uint8_t)((index + 1U) % BUS_SERVO_TX_QUEUE_SIZE);
}

static void bus_servo_uart4_try_start(void)
{
    if (bus_servo_tx_busy || bus_servo_tx_tail == bus_servo_tx_head)
    {
        return;
    }

    bus_servo_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart4,
                             bus_servo_tx_queue[bus_servo_tx_tail],
                             BUS_SERVO_TX_FRAME_LEN) != HAL_OK)
    {
        bus_servo_tx_busy = 0;
    }
}

static uint8_t bus_servo_uart4_enqueue(const uint8_t *frame)
{
    uint8_t next_head = bus_servo_tx_next(bus_servo_tx_head);

    if (next_head == bus_servo_tx_tail)
    {
        return 0;
    }

    memcpy(bus_servo_tx_queue[bus_servo_tx_head], frame, BUS_SERVO_TX_FRAME_LEN);
    bus_servo_tx_head = next_head;
    bus_servo_uart4_try_start();
    return 1;
}

static void bus_servo_uart4_tx_done_isr(void)
{
    if (bus_servo_tx_tail != bus_servo_tx_head)
    {
        bus_servo_tx_tail = bus_servo_tx_next(bus_servo_tx_tail);
    }

    bus_servo_tx_busy = 0;
    bus_servo_uart4_try_start();
}

static void bus_servo_uart4_tx_task(void)
{
    if (bus_servo_tx_busy && huart4.gState == HAL_UART_STATE_READY)
    {
        bus_servo_uart4_tx_done_isr();
        return;
    }

    bus_servo_uart4_try_start();
}

static void bus_servo_move_time_write(uint8_t servo_id, uint16_t position, uint16_t time_ms)
{
    uint8_t tx[BUS_SERVO_TX_FRAME_LEN];
    uint32_t checksum = 0;

    if (position > BUS_SERVO_POS_MAX)
    {
        position = BUS_SERVO_POS_MAX;
    }

    if (time_ms == 0)
    {
        time_ms = BUS_SERVO_TIME_DEFAULT_MS;
    }
    else if (time_ms > BUS_SERVO_TIME_MAX_MS)
    {
        time_ms = BUS_SERVO_TIME_MAX_MS;
    }

    tx[0] = 0x55;
    tx[1] = 0x55;
    tx[2] = servo_id;
    tx[3] = BUS_SERVO_MOVE_TIME_DATA_LEN;
    tx[4] = BUS_SERVO_MOVE_TIME_WRITE;
    tx[5] = (uint8_t)(position & 0xFF);
    tx[6] = (uint8_t)(position >> 8);
    tx[7] = (uint8_t)(time_ms & 0xFF);
    tx[8] = (uint8_t)(time_ms >> 8);

    for (uint8_t i = 2; i <= 8; i++)
    {
        checksum += tx[i];
    }
    tx[9] = (uint8_t)(~checksum);

    bus_servo_uart4_enqueue(tx);
}

static uint8_t gripper_pick_flip_sent = 0;
static uint8_t gripper_release_flip_sent = 0;

static void gripper_pick_action_start(void)
{
    chassis_drive_pid_use_climb(0);
    gripper_servo_boot_cancel();
    flip_servo_boot_cancel();
    chassis_force_lock_now();
    bus_servo_move_time_write(GRIPPER_SERVO_ID, GRIPPER_SERVO_GRIP_POS, GRIPPER_SERVO_GRIP_TIME_MS);
    gripper_pick_flip_sent = 0;

    action_id = ACTION_ID_GRIPPER_PICK;
    action_step = ACTION_STEP_DONE;
    action_status = ACTION_STATUS_RUNNING;
    action_step_start_tick = HAL_GetTick();
    action_fb_last_tick = 0;
}

static void gripper_release_action_start(void)
{
    chassis_drive_pid_use_climb(0);
    gripper_servo_boot_cancel();
    flip_servo_boot_cancel();
    chassis_force_lock_now();
    bus_servo_move_time_write(GRIPPER_SERVO_ID, GRIPPER_SERVO_RELEASE_POS, GRIPPER_SERVO_RELEASE_TIME_MS);
    gripper_release_flip_sent = 0;

    action_id = ACTION_ID_GRIPPER_RELEASE;
    action_step = ACTION_STEP_DONE;
    action_status = ACTION_STATUS_RUNNING;
    action_step_start_tick = HAL_GetTick();
    action_fb_last_tick = 0;
}

static uint32_t gripper_pick_action_wait_ms(void)
{
    uint32_t wait_ms = GRIPPER_SERVO_GRIP_TIME_MS;
    uint32_t flip_done_ms = GRIPPER_PICK_FLIP_DELAY_MS + FLIP_SERVO_TIME_MS;

    if (flip_done_ms > wait_ms)
    {
        wait_ms = flip_done_ms;
    }

    return wait_ms;
}

static uint32_t gripper_servo_boot_next_tick = 0;
static uint8_t gripper_servo_boot_retry_left = 0;

static void gripper_servo_boot_cancel(void)
{
    gripper_servo_boot_retry_left = 0;
}

static void gripper_servo_boot_init(void)
{
#if GRIPPER_SERVO_BOOT_RELEASE_ENABLE
    gripper_servo_boot_next_tick = HAL_GetTick() + GRIPPER_SERVO_BOOT_DELAY_MS;
#if GRIPPER_SERVO_BOOT_REPEAT_FOREVER
    gripper_servo_boot_retry_left = 1;
#else
    gripper_servo_boot_retry_left = GRIPPER_SERVO_BOOT_RETRY_COUNT;
#endif
#endif
}

static void gripper_servo_boot_task(void)
{
#if GRIPPER_SERVO_BOOT_RELEASE_ENABLE
    uint32_t now = HAL_GetTick();

    if (gripper_servo_boot_retry_left == 0)
    {
        return;
    }

    if ((int32_t)(now - gripper_servo_boot_next_tick) < 0)
    {
        return;
    }

    bus_servo_move_time_write(GRIPPER_SERVO_ID, GRIPPER_SERVO_RELEASE_POS, GRIPPER_SERVO_RELEASE_TIME_MS);
#if !GRIPPER_SERVO_BOOT_REPEAT_FOREVER
    gripper_servo_boot_retry_left--;
#endif
    gripper_servo_boot_next_tick = now + GRIPPER_SERVO_BOOT_RETRY_PERIOD_MS;
#endif
}

static void gripper_pick_action_task(void)
{
    uint32_t elapsed = 0;

    if (action_id != ACTION_ID_GRIPPER_PICK || action_status != ACTION_STATUS_RUNNING)
    {
        return;
    }

    elapsed = HAL_GetTick() - action_step_start_tick;

    if (!gripper_pick_flip_sent && elapsed >= GRIPPER_PICK_FLIP_DELAY_MS)
    {
        bus_servo_move_time_write(FLIP_SERVO_ID, FLIP_SERVO_UP_POS, FLIP_SERVO_TIME_MS);
        gripper_pick_flip_sent = 1;
    }

    if (elapsed >= gripper_pick_action_wait_ms())
    {
        action_status = ACTION_STATUS_DONE;
        action_fb_last_tick = 0;
        chassis_drive_pid_restore_requested();
    }
}

static void gripper_release_action_task(void)
{
    uint32_t elapsed = 0;

    if (action_id != ACTION_ID_GRIPPER_RELEASE || action_status != ACTION_STATUS_RUNNING)
    {
        return;
    }

    elapsed = HAL_GetTick() - action_step_start_tick;

    if (!gripper_release_flip_sent && elapsed >= GRIPPER_SERVO_RELEASE_TIME_MS)
    {
        bus_servo_move_time_write(FLIP_SERVO_ID, FLIP_SERVO_DOWN_POS, FLIP_SERVO_TIME_MS);
        gripper_release_flip_sent = 1;
    }

    if (elapsed >= (uint32_t)GRIPPER_SERVO_RELEASE_TIME_MS + FLIP_SERVO_TIME_MS)
    {
        action_status = ACTION_STATUS_DONE;
        action_fb_last_tick = 0;
        chassis_drive_pid_restore_requested();
    }
}

static void gripper_servo_set(uint8_t grip)
{
    uint16_t position = grip ? GRIPPER_SERVO_GRIP_POS : GRIPPER_SERVO_RELEASE_POS;
    uint16_t time_ms = grip ? GRIPPER_SERVO_GRIP_TIME_MS : GRIPPER_SERVO_RELEASE_TIME_MS;

    gripper_servo_boot_cancel();
    chassis_force_lock_now();
    bus_servo_move_time_write(GRIPPER_SERVO_ID, position, time_ms);
}

static uint8_t flip_servo_position_from_cmd(int32_t cmd, uint16_t *position)
{
    switch (cmd)
    {
        case 0:
            *position = FLIP_SERVO_DOWN_POS;
            return 1;

        case 1:
            *position = FLIP_SERVO_FLAT_POS;
            return 1;

        case 2:
            *position = FLIP_SERVO_UP_POS;
            return 1;

        default:
            return 0;
    }
}

static void flip_servo_set(uint16_t position)
{
    flip_servo_boot_cancel();
    chassis_force_lock_now();
    bus_servo_move_time_write(FLIP_SERVO_ID, position, FLIP_SERVO_TIME_MS);
}

static uint32_t flip_servo_boot_next_tick = 0;
static uint8_t flip_servo_boot_retry_left = 0;

static void flip_servo_boot_cancel(void)
{
    flip_servo_boot_retry_left = 0;
}

static void flip_servo_boot_init(void)
{
#if FLIP_SERVO_BOOT_DOWN_ENABLE
    flip_servo_boot_next_tick = HAL_GetTick() + FLIP_SERVO_BOOT_DELAY_MS;
#if FLIP_SERVO_BOOT_REPEAT_FOREVER
    flip_servo_boot_retry_left = 1;
#else
    flip_servo_boot_retry_left = FLIP_SERVO_BOOT_RETRY_COUNT;
#endif
#endif
}

static void flip_servo_boot_task(void)
{
#if FLIP_SERVO_BOOT_DOWN_ENABLE
    uint32_t now = HAL_GetTick();

    if (flip_servo_boot_retry_left == 0)
    {
        return;
    }

    if ((int32_t)(now - flip_servo_boot_next_tick) < 0)
    {
        return;
    }

    bus_servo_move_time_write(FLIP_SERVO_ID, FLIP_SERVO_DOWN_POS, FLIP_SERVO_TIME_MS);
#if !FLIP_SERVO_BOOT_REPEAT_FOREVER
    flip_servo_boot_retry_left--;
#endif
    flip_servo_boot_next_tick = now + FLIP_SERVO_BOOT_RETRY_PERIOD_MS;
#endif
}

void uart6_cmd_parse(char *line)
{
    char *p = line;
    char *end = 0;

    if (strcmp(p, "$GRIP,1") == 0)
    {
        gripper_servo_set(1);
        return;
    }

    if (strcmp(p, "$GRIP,0") == 0)
    {
        gripper_servo_set(0);
        return;
    }

    if (strncmp(p, "$FLIP,", 6) == 0)
    {
        int32_t cmd = strtol(p + 6, &end, 10);
        uint16_t position = 0;
        if (end == p + 6) return;
        if (!flip_servo_position_from_cmd(cmd, &position)) return;

        flip_servo_set(position);
        return;
    }

    if (strncmp(p, "$CHS,", 5) == 0)
    {
        int32_t forward = 0, left = 0, wz = 0;
        p += 5;

        forward = strtol(p, &end, 10);
        if (end == p || *end != ',') return;

        p = end + 1;
        left = strtol(p, &end, 10);
        if (end == p || *end != ',') return;

        p = end + 1;
        wz = strtol(p, &end, 10);
        if (end == p) return;

        if (chassis_force_lock_ignore_active())
        {
            chassis_cmd_zero_now();
            return;
        }

        if (chassis_force_lock_until_motion)
        {
            if (!chassis_cmd_nonzero((float)left, (float)forward, (float)wz))
            {
                chassis_cmd_zero_now();
                return;
            }
            chassis_force_lock_until_motion = 0;
        }

        chassis_cmd_vx_mm_s = (float)left;
        chassis_cmd_vy_mm_s = (float)forward;
        chassis_cmd_w_mdeg_s = (float)wz;

        chassis_cmd_last_tick = HAL_GetTick();
        return;
    }

    if (strncmp(p, "$TURN,", 6) == 0)
    {
        int32_t angle_deg = strtol(p + 6, &end, 10);
        if (end == p + 6) return;

        if (!hwt101_is_valid()) return;

        chassis_gyro_turn_target_deg = hwt101_get_yaw_deg() + (float)angle_deg;
        while (chassis_gyro_turn_target_deg > 180.0f) chassis_gyro_turn_target_deg -= 360.0f;
        while (chassis_gyro_turn_target_deg < -180.0f) chassis_gyro_turn_target_deg += 360.0f;
        chassis_gyro_turn_enable = 1;
        chassis_yaw_hold_enable = 0;
        return;
    }

    if (p[0] == '$' && p[1] == 'L' &&
        ((p[2] == '1' && p[3] == ',') ||
         (p[2] == '2' && p[3] == ',') ||
         (p[2] == '1' && p[3] == '2' && p[4] == ',')))
{
    uint8_t both = (p[3] == '2');
    char *arg = both ? (p + 5) : (p + 4);
    int32_t cmd = strtol(arg, &end, 10);
    if (end == arg) return;
    if (cmd < 0 || cmd > 3) return;

    if (p[2] == '1') lift1_cmd = (uint8_t)cmd;
    if (both || p[2] == '2') lift2_cmd = (uint8_t)cmd;
    return;
}

if (strncmp(p, "$ZERO,", 6) == 0)
{
    int32_t id = strtol(p + 6, &end, 10);
    if (end == p + 6) return;

    if (id == 1 || id == 12)
    {
        lift1_zero_angle_5 = motor5.Get_Now_Angle();
        lift1_zero_angle_6 = motor6.Get_Now_Angle();
        motor5.Set_Target_Angle(lift1_zero_angle_5);
        motor6.Set_Target_Angle(lift1_zero_angle_6);
        lift1_zero_inited = 1;
        lift1_cmd = 0;
    }

    if (id == 2 || id == 12)
    {
        lift2_zero_angle_7 = motor7.Get_Now_Angle();
        lift2_zero_angle_8 = motor8.Get_Now_Angle();
        motor7.Set_Target_Angle(lift2_zero_angle_7);
        motor8.Set_Target_Angle(lift2_zero_angle_8);
        lift2_zero_inited = 1;
        lift2_cmd = 0;
    }

    return;
}

   if (strncmp(p, "$SW,", 4) == 0)
{
    int32_t enable = 0;
    int32_t rpm = 0;

    p += 4;
    enable = strtol(p, &end, 10);
    if (end == p || *end != ',') return;

    p = end + 1;
    rpm = strtol(p, &end, 10);
    if (end == p) return;

    side_wheel_enable = (enable != 0) ? 1 : 0;
    side_wheel_target_rpm = (float)rpm;
    side_wheel_cmd_last_tick = HAL_GetTick();
    return;
}

    if (strcmp(p, "$STOP") == 0)
    {
        

        chassis_cmd_vx_mm_s = 0.0f;
        chassis_cmd_vy_mm_s = 0.0f;
        chassis_cmd_w_mdeg_s = 0.0f;

        vx_f = 0.0f;
        vy_f = 0.0f;
        w_f = 0.0f;

    lift1_cmd = 0;
    lift2_cmd = 0;
    lift3_cmd = 0;

    side_wheel_enable = 0;
    side_wheel_target_rpm = 0.0f;
    side_distance_enable = 0;
    side_distance_target_mm = 0.0f;
    side_distance_now_mm = 0.0f;

    can2_2006_target_angle_12 = 0.0f;

    motor9.Set_Target_Omega(0.0f);
    motor10.Set_Target_Omega(0.0f);
    motor11.Set_Target_Omega(0.0f);
    motor15.Set_Target_Omega(0.0f);

    action_group_stop();

        return;
    }
if (strncmp(p, "$A12,", 5) == 0)
{
    int32_t angle_mdeg = 0;

    p += 5;
    angle_mdeg = strtol(p, &end, 10);
    if (end == p) return;

    // 单位：0.001度 -> rad；目标是相对上电零点角度
    can2_2006_target_angle_12 = (float)angle_mdeg * MDEG_TO_RAD;
    return;
}

if (strncmp(p, "$L3,", 4) == 0)
{
    int32_t cmd = strtol(p + 4, &end, 10);
    if (end == p + 4) return;
    if (cmd < 0 || cmd > LIFT3_CMD_60CM) return;

    lift3_cmd = (uint8_t)cmd;
    return;
}

if (p[0] == '$' && p[1] == 'P' && p[2] == 'I' && p[3] >= '5' && p[3] <= '7' && p[4] == ',')
{
    int32_t level = strtol(p + 5, &end, 10);
    if (end == p + 5) return;

    uint32_t pin = GPIO_PIN_5 << (p[3] - '5');
    GPIOI->BSRR = level ? pin : (pin << 16U);
    return;
}
if (strncmp(p, "$SWD,", 5) == 0)
{
    int32_t distance_mm = 0;

    p += 5;
    distance_mm = strtol(p, &end, 10);
    if (end == p) return;

   side_distance_start(MTF01_MIDDLE_INDEX, (float)distance_mm);
    return;
}
if (strncmp(p, "$ACT,", 5) == 0)
{
    int32_t id = strtol(p + 5, &end, 10);
    if (end == p + 5) return;
    if (id < 0 || id > ACTION_ID_MAX) return;

    if (id == ACTION_ID_GRIPPER_PICK)
    {
        gripper_pick_action_start();
        return;
    }
    if (id == ACTION_ID_GRIPPER_RELEASE)
    {
        gripper_release_action_start();
        return;
    }

    if (id == 0)
    {
        action_group_stop();
    }
    else
    {
        action_group_start((uint8_t)id);
    }
    return;
}
}

void omni_move(float vx, float vy, float w)
{
    const float left_right_accel_step = CHASSIS_LEFT_RIGHT_ACCEL_MM_S2 * 0.000001f;
    const float left_right_decel_step = CHASSIS_LEFT_RIGHT_DECEL_MM_S2 * 0.000001f;
    const float front_back_accel_step = CHASSIS_FRONT_BACK_ACCEL_MM_S2 * 0.000001f;
    const float front_back_decel_step = CHASSIS_FRONT_BACK_DECEL_MM_S2 * 0.000001f;
    const float angular_accel_step = CHASSIS_ANGULAR_ACCEL_MDEG_S2 * MDEG_TO_RAD * 0.001f;
    const float angular_decel_step = CHASSIS_ANGULAR_DECEL_MDEG_S2 * MDEG_TO_RAD * 0.001f;

    vx_f = chassis_ramp_f(vx_f, vx, left_right_accel_step, left_right_decel_step);
    vy_f = chassis_ramp_f(vy_f, vy, front_back_accel_step, front_back_decel_step);
    w_f  = chassis_ramp_f(w_f,  w,  angular_accel_step, angular_decel_step);

    // 4 个 45 度安装全向轮/麦轮式解算
    // vx: +左移, -右移
    // vy: +前进, -后退
    // w : +逆时针, -顺时针
    float wheel_lf = vy_f - vx_f - CHASSIS_K * w_f;
    float wheel_lb = vy_f + vx_f - CHASSIS_K * w_f;
    float wheel_rb = vy_f - vx_f + CHASSIS_K * w_f;
    float wheel_rf = vy_f + vx_f + CHASSIS_K * w_f;

    float omega_1 = wheel_lf / WHEEL_RADIUS;
    float omega_2 = wheel_lb / WHEEL_RADIUS;
    float omega_3 = wheel_rb / WHEEL_RADIUS;
    float omega_4 = wheel_rf / WHEEL_RADIUS;

    float max_abs = 0.0f;
    float abs_1 = omega_1 > 0.0f ? omega_1 : -omega_1;
    float abs_2 = omega_2 > 0.0f ? omega_2 : -omega_2;
    float abs_3 = omega_3 > 0.0f ? omega_3 : -omega_3;
    float abs_4 = omega_4 > 0.0f ? omega_4 : -omega_4;

    if (abs_1 > max_abs) max_abs = abs_1;
    if (abs_2 > max_abs) max_abs = abs_2;
    if (abs_3 > max_abs) max_abs = abs_3;
    if (abs_4 > max_abs) max_abs = abs_4;

    if (max_abs > MAX_WHEEL_SPEED)
    {
        float scale = MAX_WHEEL_SPEED / max_abs;
        omega_1 *= scale;
        omega_2 *= scale;
        omega_3 *= scale;
        omega_4 *= scale;
    }

        // 第一次进入内外环控制时，让目标角度等于当前实际角度
    // 防止刚上电时目标角度从 0 开始，导致电机突然猛转
    if (!wheel_angle_target_inited)
    {
        wheel_target_angle_1 = motor1.Get_Now_Angle();
        wheel_target_angle_2 = motor2.Get_Now_Angle();
        wheel_target_angle_3 = motor3.Get_Now_Angle();
        wheel_target_angle_4 = motor4.Get_Now_Angle();

        wheel_angle_target_inited = 1;
    }

    // 控制周期是 1ms，所以 dt = 0.001s
    // 角度 = 角速度 * 时间
    // 这里把目标轮速积分成目标轮角度，交给角度外环控制
    wheel_target_angle_1 += MOTOR1_DIR * omega_1 * 0.001f;
    wheel_target_angle_2 += MOTOR2_DIR * omega_2 * 0.001f;
    wheel_target_angle_3 += MOTOR3_DIR * omega_3 * 0.001f;
    wheel_target_angle_4 += MOTOR4_DIR * omega_4 * 0.001f;

    motor1.Set_Target_Angle(wheel_target_angle_1);
    motor2.Set_Target_Angle(wheel_target_angle_2);
    motor3.Set_Target_Angle(wheel_target_angle_3);
    motor4.Set_Target_Angle(wheel_target_angle_4);
}

static float chassis_flow_lateral_hold_mm_s(void)
{
#if CHASSIS_FLOW_HOLD_ENABLE
    volatile MTF01_Data_t *mtf = &mtf01_data[CHASSIS_FLOW_HOLD_MTF_INDEX];

    if (((chassis_cmd_vy_mm_s < CHASSIS_FLOW_HOLD_MIN_FORWARD_MM_S) &&
         (chassis_cmd_vy_mm_s > -CHASSIS_FLOW_HOLD_MIN_FORWARD_MM_S)) ||
        !mtf->distance_valid ||
        !mtf->flow_valid)
    {
        chassis_flow_lateral_mm = 0.0f;
        chassis_flow_correct_mm_s = 0.0f;
        chassis_flow_yaw_correct_wz = 0.0f;
        chassis_flow_lateral_vel_f = 0.0f;
        chassis_flow_mtf_update_tick = 0;
        chassis_flow_mtf_time_ms = 0;
        return 0.0f;
    }

    if (mtf->update_tick == chassis_flow_mtf_update_tick)
    {
        return chassis_flow_correct_mm_s;
    }

    chassis_flow_mtf_update_tick = mtf->update_tick;
    uint32_t flow_time_ms = mtf->time_ms;
    float flow_dt_s = 0.01f;

    if (chassis_flow_mtf_time_ms != 0U)
    {
        uint32_t dt_ms = flow_time_ms - chassis_flow_mtf_time_ms;
        if (dt_ms > 0U && dt_ms < 100U)
        {
            flow_dt_s = (float)dt_ms * 0.001f;
        }
    }
    chassis_flow_mtf_time_ms = flow_time_ms;

    volatile MTF01_Data_t *mtf_middle = &mtf01_data[MTF01_MIDDLE_INDEX];
    volatile MTF01_Data_t *mtf_front = &mtf01_data[MTF01_FRONT_INDEX];

#if CHASSIS_FLOW_HOLD_USE_X_AXIS
    float middle_flow_raw = (float)mtf_middle->flow_vel_x;
#else
    float middle_flow_raw = (float)mtf_middle->flow_vel_y;
#endif
    float middle_height_m = (float)mtf_middle->distance_mm / 1000.0f;
    float middle_lateral_vel_mm_s = CHASSIS_FLOW_HOLD_DIR * middle_flow_raw * middle_height_m * 10.0f;
    float lateral_vel_mm_s = middle_lateral_vel_mm_s;

#if CHASSIS_FLOW_HOLD_DUAL_ENABLE
    if (mtf_front->distance_valid && mtf_front->flow_valid)
    {
#if CHASSIS_FLOW_HOLD_USE_X_AXIS
        float front_flow_raw = (float)mtf_front->flow_vel_x;
#else
        float front_flow_raw = (float)mtf_front->flow_vel_y;
#endif
        float front_height_m = (float)mtf_front->distance_mm / 1000.0f;
        float front_lateral_vel_mm_s = CHASSIS_FLOW_HOLD_DIR * front_flow_raw * front_height_m * 10.0f;
        float diff_mm_s = front_lateral_vel_mm_s - middle_lateral_vel_mm_s;
#if CHASSIS_FLOW_YAW_CORRECT_ENABLE
        float signed_diff_mm_s = diff_mm_s;
#endif
        float middle_abs_mm_s = middle_lateral_vel_mm_s > 0.0f ? middle_lateral_vel_mm_s : -middle_lateral_vel_mm_s;
        float front_abs_mm_s = front_lateral_vel_mm_s > 0.0f ? front_lateral_vel_mm_s : -front_lateral_vel_mm_s;
        uint8_t same_dir = ((middle_lateral_vel_mm_s >= 0.0f && front_lateral_vel_mm_s >= 0.0f) ||
                            (middle_lateral_vel_mm_s <= 0.0f && front_lateral_vel_mm_s <= 0.0f));

        if (diff_mm_s < 0.0f) diff_mm_s = -diff_mm_s;

#if CHASSIS_FLOW_YAW_CORRECT_ENABLE
        if ((diff_mm_s > CHASSIS_FLOW_YAW_DIFF_MIN_MM_S) &&
            (chassis_cmd_w_mdeg_s < 1000.0f) &&
            (chassis_cmd_w_mdeg_s > -1000.0f) &&
            !(CHASSIS_FLOW_YAW_DISABLE_IN_CLIMB && chassis_drive_pid_climb_mode))
        {
            float yaw_correct_wz = -CHASSIS_FLOW_YAW_DIR * signed_diff_mm_s * CHASSIS_FLOW_YAW_KP;
            if (yaw_correct_wz > CHASSIS_FLOW_YAW_MAX_WZ) yaw_correct_wz = CHASSIS_FLOW_YAW_MAX_WZ;
            if (yaw_correct_wz < -CHASSIS_FLOW_YAW_MAX_WZ) yaw_correct_wz = -CHASSIS_FLOW_YAW_MAX_WZ;
            chassis_flow_yaw_correct_wz = yaw_correct_wz;
        }
        else
        {
            chassis_flow_yaw_correct_wz = 0.0f;
        }
#else
        chassis_flow_yaw_correct_wz = 0.0f;
#endif

        if ((front_abs_mm_s > CHASSIS_FLOW_HOLD_FRONT_ONLY_MM_S) &&
            (middle_abs_mm_s < CHASSIS_FLOW_HOLD_MIDDLE_STILL_MM_S))
        {
            lateral_vel_mm_s = middle_lateral_vel_mm_s;
        }
        else if (same_dir && diff_mm_s < CHASSIS_FLOW_HOLD_DIFF_MAX_MM_S)
        {
            lateral_vel_mm_s =
                CHASSIS_FLOW_HOLD_MIDDLE_WEIGHT * middle_lateral_vel_mm_s +
                CHASSIS_FLOW_HOLD_FRONT_WEIGHT * front_lateral_vel_mm_s;
        }
    }
    else
    {
        chassis_flow_yaw_correct_wz = 0.0f;
    }
#else
    chassis_flow_yaw_correct_wz = 0.0f;
#endif
    chassis_flow_lateral_vel_f =
        CHASSIS_FLOW_FILTER_ALPHA * lateral_vel_mm_s +
        (1.0f - CHASSIS_FLOW_FILTER_ALPHA) * chassis_flow_lateral_vel_f;
    lateral_vel_mm_s = chassis_flow_lateral_vel_f;

    chassis_flow_lateral_mm += lateral_vel_mm_s * flow_dt_s;

    float correct_mm_s = -chassis_flow_lateral_mm * CHASSIS_FLOW_HOLD_KP;

    if (correct_mm_s > CHASSIS_FLOW_HOLD_MAX_MM_S) correct_mm_s = CHASSIS_FLOW_HOLD_MAX_MM_S;
    if (correct_mm_s < -CHASSIS_FLOW_HOLD_MAX_MM_S) correct_mm_s = -CHASSIS_FLOW_HOLD_MAX_MM_S;

    chassis_flow_correct_mm_s = correct_mm_s;
    return chassis_flow_correct_mm_s;
#else
    return 0.0f;
#endif
}


// ==================== 底盘任务函数 ====================
void sbus_chassis_task(void)
{
#if RC_SBUS_ENABLE
    if (sbus_ch[RC_SBUS_ENABLE_CH] < RC_SBUS_ENABLE_THRESHOLD)
    {
        if (sbus_ch[7])
        {
            chassis_cmd_last_tick = 0;
            sbus_ch[7] = 0;
        }
        if (action_status != ACTION_STATUS_RUNNING)
        {
            chassis_drive_pid_restore_requested();
        }
        return;
    }
    sbus_ch[7] = 1;

    if (action_status == ACTION_STATUS_RUNNING)
    {
        return;
    }

    chassis_drive_pid_use_climb(sbus_ch[RC_SBUS_CLIMB_PID_CH] > RC_SBUS_CLIMB_PID_THRESHOLD);

    if (chassis_force_lock_until_motion && action_status != ACTION_STATUS_RUNNING)
    {
        chassis_force_lock_until_motion = 0;
        chassis_cmd_zero_now();
    }

    int16_t ch_forward = (int16_t)sbus_ch[1] - RC_SBUS_CH_CENTER;
    int16_t ch_left = RC_SBUS_CH_CENTER - (int16_t)sbus_ch[0];
    int16_t ch_wz = RC_SBUS_CH_CENTER - (int16_t)sbus_ch[3];

    if (ch_forward < RC_SBUS_CH_DEADBAND && ch_forward > -RC_SBUS_CH_DEADBAND) ch_forward = 0;
    if (ch_left < RC_SBUS_CH_DEADBAND && ch_left > -RC_SBUS_CH_DEADBAND) ch_left = 0;
    if (ch_wz < RC_SBUS_CH_DEADBAND && ch_wz > -RC_SBUS_CH_DEADBAND) ch_wz = 0;

    float forward = (float)ch_forward * RC_CHASSIS_LINEAR_SCALE_MM_S;
    float left = (float)ch_left * RC_CHASSIS_LINEAR_SCALE_MM_S;
    float wz = (float)ch_wz * RC_CHASSIS_WZ_SCALE_MDEG_S;

    chassis_cmd_vx_mm_s = left;
    chassis_cmd_vy_mm_s = forward;
    chassis_cmd_w_mdeg_s = wz;
    chassis_cmd_last_tick = HAL_GetTick();
#endif
}

void chassis_control_task(void)
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    float yaw_hold_kp = 0.0f;
    float yaw_hold_kd = 0.0f;
    float yaw_hold_max_wz = 0.0f;

    if (chassis_force_lock_until_motion)
    {
        chassis_cmd_zero_now();
        omni_move(0.0f, 0.0f, 0.0f);
        return;
    }

    // 上位机超过一段时间没有继续发底盘速度，自动停车
    if (HAL_GetTick() - chassis_cmd_last_tick > CHASSIS_CMD_TIMEOUT_MS)
    {
        chassis_cmd_vx_mm_s = 0.0f;
        chassis_cmd_vy_mm_s = 0.0f;
        chassis_cmd_w_mdeg_s = 0.0f;
    }

    float flow_left_comp_mm_s = chassis_flow_lateral_hold_mm_s();

    vx = (chassis_cmd_vx_mm_s + flow_left_comp_mm_s) / 1000.0f;
    vy = chassis_cmd_vy_mm_s / 1000.0f;
    wz = chassis_cmd_w_mdeg_s * MDEG_TO_RAD;

    uint8_t need_yaw_hold =
    hwt101_is_valid() &&
    (wz < 0.001f && wz > -0.001f) &&
    ((vx > 0.02f || vx < -0.02f) || (vy > 0.02f || vy < -0.02f));

if (need_yaw_hold)
{
    if (!chassis_yaw_hold_enable)
    {
        chassis_yaw_hold_target_deg = hwt101_get_yaw_deg();
        chassis_yaw_hold_enable = 1;
    }

    if (chassis_drive_pid_climb_mode)
    {
        yaw_hold_kp = HWT101_YAW_HOLD_CLIMB_KP;
        yaw_hold_kd = HWT101_YAW_HOLD_CLIMB_KD;
        yaw_hold_max_wz = HWT101_YAW_HOLD_CLIMB_MAX_WZ;
    }
    else
    {
        yaw_hold_kp = HWT101_YAW_HOLD_KP;
        yaw_hold_kd = HWT101_YAW_HOLD_KD;
        yaw_hold_max_wz = HWT101_YAW_HOLD_MAX_WZ;
    }

    float yaw_error_deg = angle_diff_deg(chassis_yaw_hold_target_deg, hwt101_get_yaw_deg());
    float yaw_correct_wz =
        HWT101_YAW_CORRECT_DIR *
        (yaw_error_deg * yaw_hold_kp - hwt101_gyro_z_dps * yaw_hold_kd);

    if (yaw_correct_wz > yaw_hold_max_wz) yaw_correct_wz = yaw_hold_max_wz;
    if (yaw_correct_wz < -yaw_hold_max_wz) yaw_correct_wz = -yaw_hold_max_wz;

    float yaw_slew_step = HWT101_YAW_HOLD_SLEW_RAD_S2 * 0.001f;
    chassis_yaw_hold_correct_wz =
        chassis_ramp_f(chassis_yaw_hold_correct_wz, yaw_correct_wz, yaw_slew_step, yaw_slew_step);

    wz += chassis_yaw_hold_correct_wz;
}
else
{
    chassis_yaw_hold_enable = 0;
    chassis_yaw_hold_correct_wz = 0.0f;
}

    wz += chassis_flow_yaw_correct_wz;

    // 限幅，防止上位机发错值
    if (vx > MAX_CHASSIS_SPEED) vx = MAX_CHASSIS_SPEED;
    if (vx < -MAX_CHASSIS_SPEED) vx = -MAX_CHASSIS_SPEED;

    if (vy > MAX_CHASSIS_SPEED) vy = MAX_CHASSIS_SPEED;
    if (vy < -MAX_CHASSIS_SPEED) vy = -MAX_CHASSIS_SPEED;

    if (wz > MAX_CHASSIS_OMEGA) wz = MAX_CHASSIS_OMEGA;
    if (wz < -MAX_CHASSIS_OMEGA) wz = -MAX_CHASSIS_OMEGA;

    omni_move(vx, vy, wz);
}

// ==================== 霍尔传感器 ====================
#if 0
static uint8_t lift1_home_triggered(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_RESET;
}

static uint8_t lift2_home_triggered(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET;
}

#endif

#if 0
static uint8_t lift1_top_limit_triggered(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET;
}

static uint8_t lift2_top_limit_triggered(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_RESET;
}
#endif

#if UART2_DEBUG_MTF || UART3_DEBUG_MTF
static char uart2_flow_sign(int16_t value)
{
    if (value > 20) return '+';
    if (value < -20) return '-';
    return '0';
}
#endif

#if UART2_DEBUG_MTF || UART3_DEBUG_MTF
void uart2_mtf_feedback_task(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    volatile MTF01_Data_t *mtf;
    char msg[16];
    uint16_t pos = 0;
#if UART3_DEBUG_MTF
    uint8_t index = UART3_DEBUG_MTF_INDEX;
#else
    uint8_t index = UART2_DEBUG_MTF_INDEX;
#endif

    if (index >= MTF01_COUNT)
    {
        return;
    }

    if (now - last_tick < UART2_FB_PERIOD_MS)
    {
        return;
    }

    last_tick = now;
    mtf = &mtf01_data[index];

    msg[pos++] = '$';
    msg[pos++] = 'M';
    msg[pos++] = 'T';
    msg[pos++] = 'F';
    msg[pos++] = (char)('1' + index);
    msg[pos++] = ',';
    msg[pos++] = uart2_flow_sign(mtf->flow_vel_x);
    msg[pos++] = ',';
    msg[pos++] = uart2_flow_sign(mtf->flow_vel_y);
    msg[pos++] = ',';
    msg[pos++] = mtf->distance_valid ? '1' : '0';
    msg[pos++] = ',';
    msg[pos++] = mtf->flow_valid ? '1' : '0';
    msg[pos++] = '\r';
    msg[pos++] = '\n';

#if UART2_DEBUG_MTF
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, pos, 5);
#endif
#if UART3_DEBUG_MTF
    serial_write((uint8_t *)msg, pos);
#endif
}
#endif

#if 0
void lift12_home_task(void)
{
    static uint32_t home_start_tick = 0;
    static uint8_t home_step = 0;

    if (lift12_home_request)
    {
        lift12_home_request = 0;
        lift12_home_active = 1;
        lift1_zero_inited = 0;
        lift2_zero_inited = 0;
        lift1_cmd = 0;
        lift2_cmd = 0;
        home_step = 0;
        home_start_tick = HAL_GetTick();
    }

    if (!lift12_home_active)
    {
        return;
    }

    if (home_step == 0)
    {
        if (HAL_GetTick() - home_start_tick < LIFT_ZERO_DELAY_MS)
        {
            return;
        }

        motor5.Set_Target_Angle(motor5.Get_Now_Angle() + LIFT1_MOTOR5_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
        motor6.Set_Target_Angle(motor6.Get_Now_Angle() + LIFT1_MOTOR6_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
        motor7.Set_Target_Angle(motor7.Get_Now_Angle() + LIFT2_MOTOR7_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
        motor8.Set_Target_Angle(motor8.Get_Now_Angle() + LIFT2_MOTOR8_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);

        home_step = 1;
        home_start_tick = HAL_GetTick();
        return;
    }

    if (HAL_GetTick() - home_start_tick > LIFT_HOME_TIMEOUT_MS)
    {
        lift12_home_active = 0;
        lift12_home_fail = 1;
        motor5.Set_Target_Angle(motor5.Get_Now_Angle());
        motor6.Set_Target_Angle(motor6.Get_Now_Angle());
        motor7.Set_Target_Angle(motor7.Get_Now_Angle());
        motor8.Set_Target_Angle(motor8.Get_Now_Angle());
        return;
    }

    if (HAL_GetTick() - home_start_tick >= LIFT12_BOOT_HOME_WAIT_MS)
    {
        lift1_zero_angle_5 = motor5.Get_Now_Angle();
        lift1_zero_angle_6 = motor6.Get_Now_Angle();
        lift2_zero_angle_7 = motor7.Get_Now_Angle();
        lift2_zero_angle_8 = motor8.Get_Now_Angle();

        motor5.Set_Target_Angle(lift1_zero_angle_5);
        motor6.Set_Target_Angle(lift1_zero_angle_6);
        motor7.Set_Target_Angle(lift2_zero_angle_7);
        motor8.Set_Target_Angle(lift2_zero_angle_8);

        lift1_zero_inited = 1;
        lift2_zero_inited = 1;
        lift12_home_active = 0;
        lift12_home_done = 1;
    }
}

// ==================== 升降台1任务函数 ====================
#endif

void lift1_control_task(void)
{
    float target_angle_5;
    float target_angle_6;
    float target_angle;
    static uint32_t home_tick = 0;

    if (lift12_home_active)
    {
        if (lift12_home_active == 1)
        {
            motor5.Set_Target_Angle(motor5.Get_Now_Angle() + LIFT1_MOTOR5_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
            motor6.Set_Target_Angle(motor6.Get_Now_Angle() + LIFT1_MOTOR6_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
            motor7.Set_Target_Angle(motor7.Get_Now_Angle() + LIFT2_MOTOR7_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
            motor8.Set_Target_Angle(motor8.Get_Now_Angle() + LIFT2_MOTOR8_HOME_DIR * LIFT12_BOOT_HOME_ANGLE);
            home_tick = HAL_GetTick();
            lift12_home_active = 2;
        }
        else if (HAL_GetTick() - home_tick >= 3000U)
        {
            lift1_zero_angle_5 = motor5.Get_Now_Angle();
            lift1_zero_angle_6 = motor6.Get_Now_Angle();
            lift2_zero_angle_7 = motor7.Get_Now_Angle();
            lift2_zero_angle_8 = motor8.Get_Now_Angle();
            lift1_zero_inited = lift2_zero_inited = 1;
            lift12_home_active = 0;
        }
        return;
    }

    if (!lift1_zero_inited)
    {
        motor5.Set_Target_Angle(motor5.Get_Now_Angle());
        motor6.Set_Target_Angle(motor6.Get_Now_Angle());
        return;
    }

    if (lift1_cmd == 1) target_angle = LIFT1_LOW_ANGLE;
    else if (lift1_cmd == 2) target_angle = LIFT1_HIGH_ANGLE;
    else if (lift1_cmd == 3) target_angle = LIFT1_THIRD_ANGLE;
    else target_angle = 0.0f;

    target_angle_5 = lift1_zero_angle_5 + LIFT1_MOTOR5_STAGE_DIR * target_angle * LIFT1_MOTOR5_STAGE_SCALE;
    target_angle_6 = lift1_zero_angle_6 + LIFT1_MOTOR6_STAGE_DIR * target_angle * LIFT1_MOTOR6_STAGE_SCALE;

    motor5.Set_Target_Angle(target_angle_5);
    motor6.Set_Target_Angle(target_angle_6);
}

// ==================== 升降台2任务函数 ====================
void lift2_control_task(void)
{
    float target_angle_7 = 0.0f;
    float target_angle_8 = 0.0f;
    float target_angle = 0.0f;

    if (lift12_home_active)
    {
        return;
    }

    if (!lift2_zero_inited)
    {
        motor7.Set_Target_Angle(motor7.Get_Now_Angle());
        motor8.Set_Target_Angle(motor8.Get_Now_Angle());
        return;
    }

    if (lift2_cmd == 1) target_angle = LIFT2_LOW_ANGLE;
    else if (lift2_cmd == 2) target_angle = LIFT2_HIGH_ANGLE;
    else if (lift2_cmd == 3) target_angle = LIFT2_THIRD_ANGLE;

    target_angle_7 = lift2_zero_angle_7 + LIFT2_MOTOR7_STAGE_DIR * target_angle * LIFT2_MOTOR7_STAGE_SCALE;
    target_angle_8 = lift2_zero_angle_8 + LIFT2_MOTOR8_STAGE_DIR * target_angle * LIFT2_MOTOR8_STAGE_SCALE;

    motor7.Set_Target_Angle(target_angle_7);
    motor8.Set_Target_Angle(target_angle_8);
}
// ==================== 2006 角度控制任务 ====================
void can2_2006_angle_task(void)
{
    if (!can2_2006_zero_inited)
    {
        can2_2006_zero_angle_12 = motor12.Get_Now_Angle();
can2_2006_target_angle_12 = 0.0f;
motor12.Set_Target_Angle(can2_2006_zero_angle_12);

        if (HAL_GetTick() > LIFT_ZERO_DELAY_MS)
        {
            can2_2006_zero_inited = 1;
        }
        return;
    }

    motor12.Set_Target_Angle(can2_2006_zero_angle_12 +
                             CAN2_2006_ANGLE_DIR * can2_2006_target_angle_12);
}

void lift3_control_task(void)
{
    float target_angle_13 = 0.0f;
    float target_angle_14 = 0.0f;
    float target_angle = 0.0f;

    if (!lift3_zero_inited)
    {
        lift3_zero_angle_13 = motor13.Get_Now_Angle();
        lift3_zero_angle_14 = motor14.Get_Now_Angle();

        motor13.Set_Target_Angle(lift3_zero_angle_13);
        motor14.Set_Target_Angle(lift3_zero_angle_14);

        if (HAL_GetTick() > LIFT_ZERO_DELAY_MS)
        {
            lift3_zero_inited = 1;
        }
        return;
    }

    switch (lift3_cmd)
    {
        case LIFT3_CMD_LOW:
            target_angle = LIFT3_LOW_ANGLE;
            break;

        case LIFT3_CMD_HIGH:
            target_angle = LIFT3_HIGH_ANGLE;
            break;

        case LIFT3_CMD_10CM:
            target_angle = LIFT3_10CM_ANGLE;
            break;

        case LIFT3_CMD_20CM:
            target_angle = LIFT3_20CM_ANGLE;
            break;

        case LIFT3_CMD_40CM:
            target_angle = LIFT3_40CM_ANGLE;
            break;

        case LIFT3_CMD_60CM:
            target_angle = LIFT3_60CM_ANGLE;
            break;

        default:
            target_angle = 0.0f;
            break;
    }

    target_angle_13 = lift3_zero_angle_13 + LIFT3_MOTOR13_DIR * target_angle;
    target_angle_14 = lift3_zero_angle_14 + LIFT3_MOTOR14_DIR * target_angle;

    motor13.Set_Target_Angle(target_angle_13);
    motor14.Set_Target_Angle(target_angle_14);
}

void side_wheel_control_task(void)
{
    float rpm = side_wheel_target_rpm;
    float omega = 0.0f;
    uint8_t climb_follow = 0;

#if SIDE_WHEEL_CLIMB_FOLLOW_ENABLE
    if (chassis_drive_pid_climb_mode)
    {
        float forward_mm_s = chassis_cmd_vy_mm_s;
        climb_follow = 1;

        if (forward_mm_s < SIDE_WHEEL_CLIMB_MIN_MM_S &&
            forward_mm_s > -SIDE_WHEEL_CLIMB_MIN_MM_S)
        {
            rpm = 0.0f;
        }
        else
        {
            rpm = forward_mm_s * SIDE_WHEEL_CLIMB_RPM_PER_MM_S;
        }
    }
#endif

    if (!climb_follow && HAL_GetTick() - side_wheel_cmd_last_tick > SIDE_WHEEL_CMD_TIMEOUT_MS)
    {
        side_wheel_enable = 0;
        side_wheel_target_rpm = 0.0f;
    }

    if (!climb_follow && !side_wheel_enable)
    {
        motor9.Set_Target_Omega(0.0f);
        motor10.Set_Target_Omega(0.0f);
        motor11.Set_Target_Omega(0.0f);
        motor15.Set_Target_Omega(0.0f);
        return;
    }

    if (rpm > SIDE_WHEEL_MAX_RPM) rpm = SIDE_WHEEL_MAX_RPM;
    if (rpm < -SIDE_WHEEL_MAX_RPM) rpm = -SIDE_WHEEL_MAX_RPM;

    omega = rpm * SIDE_WHEEL_RPM_TO_RADPS;

    motor9.Set_Target_Omega(SIDE_WHEEL9_DIR * omega);
    motor10.Set_Target_Omega(SIDE_WHEEL10_DIR * omega);
    motor11.Set_Target_Omega(SIDE_WHEEL11_DIR * omega);
    motor15.Set_Target_Omega(SIDE_WHEEL15_DIR * omega);
}

void uart_init_task(void)
{
    HAL_UART_Receive_IT(&huart1, &sbus_rx_data, 1);
    HAL_UART_Receive_IT(&huart3, &uart6_rx_data, 1);
    HAL_UART_Receive_IT(&huart6, &mtf01_rx_data_1, 1);
    HAL_UART_Receive_IT(&huart7, &hwt101_rx_data, 1);
    HAL_UART_Receive_IT(&huart8, &mtf01_rx_data_2, 1);
}



// 根据四个轮子的实际转速，反解底盘实际速度
// 电机反馈 Get_Now_Omega() 单位：rad/s
//
// 当前 omni_move() 正解为：
// wheel_lf = vy - vx - K*w
// wheel_lb = vy + vx - K*w
// wheel_rb = vy - vx + K*w
// wheel_rf = vy + vx + K*w
//
// 反解为：
// vy = (v_lf + v_lb + v_rb + v_rf) / 4
// vx = (-v_lf + v_lb - v_rb + v_rf) / 4
// w  = (-v_lf - v_lb + v_rb + v_rf) / (4*K)
void chassis_feedback_update(void)
{
    uint32_t now = HAL_GetTick();

    float omega_1 = MOTOR1_DIR * motor1.Get_Now_Omega(); // 左前
    float omega_2 = MOTOR2_DIR * motor2.Get_Now_Omega(); // 左后
    float omega_3 = MOTOR3_DIR * motor3.Get_Now_Omega(); // 右后
    float omega_4 = MOTOR4_DIR * motor4.Get_Now_Omega(); // 右前

    float v_lf = omega_1 * WHEEL_RADIUS;
    float v_lb = omega_2 * WHEEL_RADIUS;
    float v_rb = omega_3 * WHEEL_RADIUS;
    float v_rf = omega_4 * WHEEL_RADIUS;

    chassis_now_vy = (v_lf + v_lb + v_rb + v_rf) * 0.25f;
    chassis_now_vx = (-v_lf + v_lb - v_rb + v_rf) * 0.25f;
    chassis_now_wz = (-v_lf - v_lb + v_rb + v_rf) / (4.0f * CHASSIS_K);

    // 用角速度积分得到当前旋转角度
    // 注意：这是轮速里程计角度，长时间运行会累计误差
    if (chassis_fb_last_update_tick == 0)
    {
        chassis_fb_last_update_tick = now;
    }
    else
    {
        float dt = (float)(now - chassis_fb_last_update_tick) / 1000.0f;
        chassis_fb_last_update_tick = now;

        float yaw_mid = chassis_yaw + chassis_now_wz * dt * 0.5f;
        float cos_yaw = cosf(yaw_mid);
        float sin_yaw = sinf(yaw_mid);
        float forward_m_s = chassis_now_vy;
        float left_m_s = chassis_now_vx;

        chassis_odom_x += (forward_m_s * cos_yaw - left_m_s * sin_yaw) * dt;
        chassis_odom_y += (forward_m_s * sin_yaw + left_m_s * cos_yaw) * dt;
        chassis_yaw += chassis_now_wz * dt;
    }
}


// USART3 周期回传 WheelOdom：pos_x/pos_y/yaw/vx/vy/omega。
void uart6_feedback_task(void)
{
#if !CHASSIS_ODOM_FB_ENABLE && !CHASSIS_VEL_FB_ENABLE
    return;
#else
    uint32_t now = HAL_GetTick();
    static uint32_t last_tick = 0;

    if (uart3_motor_debug_enable)
    {
        return;
    }

    if (now - last_tick < CHASSIS_VEL_FB_PERIOD_MS)
    {
        return;
    }
    last_tick = now;

    Packet_WheelOdom pkt;
    float fb_vx = chassis_now_vy;
    float fb_vy = chassis_now_vx;
    float fb_wz = chassis_now_wz;
#if CHASSIS_VEL_FB_LOCK_ZERO
    if (chassis_force_lock_until_motion)
    {
        fb_vx = 0.0f;
        fb_vy = 0.0f;
        fb_wz = 0.0f;
    }
#endif

    pkt.pos_x = chassis_odom_x;
    pkt.pos_y = chassis_odom_y;
    pkt.yaw = chassis_yaw;
    pkt.vx = chassis_vel_fb_deadband(fb_vx);
    pkt.vy = chassis_vel_fb_deadband(fb_vy);
    pkt.omega = fb_wz;

    send_WheelOdom(&pkt);
#endif
}

static void debug_append_i32(char *buf, uint16_t *pos, int32_t value)
{
    char tmp[12];
    uint8_t len = 0;
    uint32_t v;

    if (value < 0)
    {
        buf[(*pos)++] = '-';
        v = (uint32_t)(-value);
    }
    else
    {
        v = (uint32_t)value;
    }

    do
    {
        tmp[len++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v > 0U);

    while (len > 0U)
    {
        buf[(*pos)++] = tmp[--len];
    }
}

static void debug_append_field(char *buf, uint16_t *pos, int32_t value)
{
    buf[(*pos)++] = ',';
    debug_append_i32(buf, pos, value);
}

void uart3_motor_debug_task(void)
{
    uint32_t now = HAL_GetTick();
    static uint32_t last_tick = 0;
    char msg[160];
    uint16_t pos = 0;

    if (!uart3_motor_debug_enable)
    {
        return;
    }

    if (now - last_tick < UART6_FB_PERIOD_MS)
    {
        return;
    }
    last_tick = now;

    msg[pos++] = '$';
    msg[pos++] = 'M';
    msg[pos++] = 'O';
    msg[pos++] = 'T';

    debug_append_field(msg, &pos, (int32_t)motor1.Get_Out());
    debug_append_field(msg, &pos, (int32_t)motor2.Get_Out());
    debug_append_field(msg, &pos, (int32_t)motor3.Get_Out());
    debug_append_field(msg, &pos, (int32_t)motor4.Get_Out());

    debug_append_field(msg, &pos, (int32_t)(motor1.Get_Target_Omega() * 100.0f));
    debug_append_field(msg, &pos, (int32_t)(motor2.Get_Target_Omega() * 100.0f));
    debug_append_field(msg, &pos, (int32_t)(motor3.Get_Target_Omega() * 100.0f));
    debug_append_field(msg, &pos, (int32_t)(motor4.Get_Target_Omega() * 100.0f));

    debug_append_field(msg, &pos, (int32_t)(motor1.Get_Now_Omega() * 100.0f));
    debug_append_field(msg, &pos, (int32_t)(motor2.Get_Now_Omega() * 100.0f));
    debug_append_field(msg, &pos, (int32_t)(motor3.Get_Now_Omega() * 100.0f));
    debug_append_field(msg, &pos, (int32_t)(motor4.Get_Now_Omega() * 100.0f));

    msg[pos++] = '\r';
    msg[pos++] = '\n';

    serial_write((uint8_t *)msg, pos);
}

// USART2 周期输出 1号光流模块数据
// 输出格式：$MTF1,distance_mm,flow_x,flow_y,quality,distance_valid,flow_valid,update_tick,rx_count,last_byte\r\n
void motor_pid_init(void)
{
    // ==================== 底盘 4 个 3508：角度外环 + 速度内环 ====================
    chassis_drive_pid_use_climb(0);

    // ==================== 两个升降台 4 个 3508 ====================
    motor5.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, LIFT12_MAX_OMEGA, 0.001f, 0.0f);
    motor6.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, LIFT12_MAX_OMEGA, 0.001f, 0.0f);
    motor7.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, LIFT12_MAX_OMEGA, 0.001f, 0.0f);
    motor8.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, LIFT12_MAX_OMEGA, 0.001f, 0.0f);

    motor5.PID_Omega.Init(1800.0f, 1500.0f, 0.0f, 0.0f, 10000.0f, 16384.0f, 0.001f, 0.0f);
    motor6.PID_Omega.Init(1800.0f, 1500.0f, 0.0f, 0.0f, 10000.0f, 16384.0f, 0.001f, 0.0f);
    motor7.PID_Omega.Init(1800.0f, 1500.0f, 0.0f, 0.0f, 10000.0f, 16384.0f, 0.001f, 0.0f);
    motor8.PID_Omega.Init(1800.0f, 1500.0f, 0.0f, 0.0f, 10000.0f, 16384.0f, 0.001f, 0.0f);

    // ==================== CAN1 上两个 2006 普通轮 ====================
    motor9.PID_Omega.Init(1000.0f, 1000.0f, 0.0f, 0.0f, 2000.0f, 8000.0f, 0.001f, 0.0f);

    motor10.PID_Omega.Init(1000.0f, 1000.0f, 0.0f, 0.0f, 2000.0f, 8000.0f, 0.001f, 0.0f);
    motor11.PID_Omega.Init(1000.0f, 1000.0f, 0.0f, 0.0f, 2000.0f, 8000.0f, 0.001f, 0.0f);
    motor15.PID_Omega.Init(1000.0f, 1000.0f, 0.0f, 0.0f, 2000.0f, 8000.0f, 0.001f, 0.0f);

    motor12.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, ANGLE_2006_MAX_OMEGA, 0.001f, 0.0f);
    motor12.PID_Omega.Init(1200.0f, 800.0f, 0.0f, 0.0f, 3000.0f, 12000.0f, 0.001f, 0.0f);

    motor13.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, LIFT3_MAX_OMEGA, 0.001f, 0.0f);
    motor14.PID_Angle.Init(4.0f, 0.0f, 0.0f, 0.0f, 0.0f, LIFT3_MAX_OMEGA, 0.001f, 0.0f);

    motor13.PID_Omega.Init(1200.0f, 800.0f, 0.0f, 0.0f, 3000.0f, 12000.0f, 0.001f, 0.0f);
    motor14.PID_Omega.Init(1200.0f, 800.0f, 0.0f, 0.0f, 3000.0f, 12000.0f, 0.001f, 0.0f);
}


void motor_can_init(void)
{
    // ==================== CAN1：底盘 3508 ====================
    motor1.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_ANGLE, GEAR_RATIO);
    motor2.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_ANGLE, GEAR_RATIO);
    motor3.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_ANGLE, GEAR_RATIO);
    motor4.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_ANGLE, GEAR_RATIO);

    // ==================== CAN1：两个 2006 普通轮 ====================
    motor9.Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_OMEGA, 36.0f, 10000.0f);
    motor10.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 36.0f, 10000.0f);
    motor11.Init(&hcan1, CAN_Motor_ID_0x207, Control_Method_OMEGA, 36.0f, 10000.0f);
    motor12.Init(&hcan1, CAN_Motor_ID_0x208, Control_Method_ANGLE, GEAR_RATIO);

    // ==================== CAN2：两个升降台 3508 ====================
    motor5.Init(&hcan2, CAN_Motor_ID_0x201, Control_Method_ANGLE, GEAR_RATIO);
    motor6.Init(&hcan2, CAN_Motor_ID_0x202, Control_Method_ANGLE, GEAR_RATIO);
    motor7.Init(&hcan2, CAN_Motor_ID_0x203, Control_Method_ANGLE, GEAR_RATIO);
    motor8.Init(&hcan2, CAN_Motor_ID_0x204, Control_Method_ANGLE, GEAR_RATIO);

    motor15.Init(&hcan2, CAN_Motor_ID_0x205, Control_Method_OMEGA, 36.0f, 10000.0f);
    motor13.Init(&hcan2, CAN_Motor_ID_0x206, Control_Method_ANGLE, GEAR_RATIO);
    motor14.Init(&hcan2, CAN_Motor_ID_0x207, Control_Method_ANGLE, GEAR_RATIO);
}

void motor_pid_task(void)
{
    motor1.TIM_PID_PeriodElapsedCallback();
    motor2.TIM_PID_PeriodElapsedCallback();
    motor3.TIM_PID_PeriodElapsedCallback();
    motor4.TIM_PID_PeriodElapsedCallback();

    motor5.TIM_PID_PeriodElapsedCallback();
    motor6.TIM_PID_PeriodElapsedCallback();
    motor7.TIM_PID_PeriodElapsedCallback();
    motor8.TIM_PID_PeriodElapsedCallback();

    motor9.TIM_PID_PeriodElapsedCallback();
    motor10.TIM_PID_PeriodElapsedCallback();
    motor11.TIM_PID_PeriodElapsedCallback();
#if SIDE_WHEEL_FEEDBACK_TIMEOUT_MS > 0U
    if (side_wheel15_feedback_last_tick != 0U &&
        HAL_GetTick() - side_wheel15_feedback_last_tick <= SIDE_WHEEL_FEEDBACK_TIMEOUT_MS)
    {
        motor15.Set_Control_Method(Control_Method_OMEGA);
    }
    else
    {
        motor15.Set_Target_Torque(0.0f);
        motor15.Set_Control_Method(Control_Method_OPENLOOP);
    }
#endif
    motor15.TIM_PID_PeriodElapsedCallback();
    motor12.TIM_PID_PeriodElapsedCallback();
    motor13.TIM_PID_PeriodElapsedCallback();
    motor14.TIM_PID_PeriodElapsedCallback();
}

 // ==================== 底盘四个万向轮前进/后退指定距离====================
#if 0
static float speed_percent_to_linear(float speed)
{
    return (speed / 100.0f) * MAX_CHASSIS_SPEED;
}

static float speed_percent_to_omega(float speed)
{
    return (speed / 100.0f) * MAX_CHASSIS_OMEGA;
}

// ==================== 动作组函数====================
uint8_t parse_upper_cmd(char *line, int32_t *x_mm, int32_t *y_mm, int32_t *z_deg)
{
    char *p = line;
    char *end = 0;

    *x_mm = strtol(p, &end, 10);
    if (end == p || *end != ',') return 0;

    p = end + 1;
    *y_mm = strtol(p, &end, 10);
    if (end == p || *end != ',') return 0;

    p = end + 1;
    *z_deg = strtol(p, &end, 10);
    if (end == p) return 0;

    return 1;
}



void (*last_fxfunction)(float speed);

void move_front(float speed) { omni_move(0.0f, speed_percent_to_linear(speed), 0.0f); }
void move_back(float speed)  { omni_move(0.0f, -speed_percent_to_linear(speed), 0.0f); }
void move_left(float speed)  { omni_move(speed_percent_to_linear(speed), 0.0f, 0.0f); }
void move_right(float speed) { omni_move(-speed_percent_to_linear(speed), 0.0f, 0.0f); }
void turn_left(float speed)  { omni_move(0.0f, 0.0f, speed_percent_to_omega(speed)); }
void turn_right(float speed) { omni_move(0.0f, 0.0f, -speed_percent_to_omega(speed)); }
void stop(float speed)       { omni_move(0.0f, 0.0f, 0.0f); }
void Direction_Init(void)    { last_fxfunction = stop; }
#endif






/**
 * @brief HAL库UART接收DMA空闲中断
 * @param Buffer 接收缓冲区
 * @param Length 数据长度
 * @return void
 * @note 处理串口调参和运动指令。
 */
#if 0
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
        // 电机调PID
        case(0):
        {
            motor1.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
                        motor4.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						motor5.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
           motor6.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());
						
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(1):
        {
            motor1.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
            motor5.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
						motor6.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(2):
        {
            motor1.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor2.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor3.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor4.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
           motor5.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
						motor6.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(3):
        {
           motor1.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
					motor4.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
           	motor5.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
					motor6.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(4):
        {
            motor1.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
           motor5.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
						motor6.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());
           
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
        case(5):
        {
            motor1.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor2.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor3.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor4.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
           	motor5.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
						motor6.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());
					// last_fxfunction(torque);  // 禁用：避免调参时重新执行运动
        }
        break;
				case(6):
        {
            torque=serialplot.Get_Variable_Value();
						// last_fxfunction(torque);  // 禁用：避免设置扭矩时重新执行运动
        }
        break;
				case(7):
        {
            fx=serialplot.Get_Variable_Value();
            if(fx==0.0f){stop(0); last_fxfunction=stop;}
            else if(fx==1.0f){move_front(torque); last_fxfunction=move_front;}
            else if(fx==2.0f){move_back(torque); last_fxfunction=move_back;}
            else if(fx==3.0f){move_right(torque); last_fxfunction=move_right;}
            else if(fx==4.0f){move_left(torque); last_fxfunction=move_left;}
            else if(fx==5.0f){turn_left(torque); last_fxfunction=turn_left;}
            else if(fx==6.0f){turn_right(torque); last_fxfunction=turn_right;}
            else{last_fxfunction(torque);}
    }
}
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  /* USART2 disabled for current build. */
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
 
  BSP_Init(BSP_DC24_LU_ON | BSP_DC24_LD_ON | BSP_DC24_RU_ON | BSP_DC24_RD_ON);
  
   CAN_Init(&hcan1, CAN1_Call_Back);
CAN_Init(&hcan2, CAN2_Call_Back);

uart_init_task();
motor_pid_init();
motor_can_init();
gripper_servo_boot_init();
flip_servo_boot_init();
   
		
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  static uint32_t last_tick = 0;
  while (1)
  {
     if (protocol_heartbeat_tx_pending)
    {
        Packet_Heartbeat pkt;
        pkt.count = protocol_heartbeat_ack_count;
        send_Heartbeat(&pkt);
        protocol_heartbeat_tx_pending = 0;
    }

	 if (protocol_handshake_tx_pending > 0)
    {
        Packet_Handshake pkt;
        pkt.protocol_hash = protocol_handshake_hash;
        send_Handshake(&pkt);
        protocol_handshake_tx_pending--;
    }

    if (protocol_action_cmd_pending)
    {
        uint8_t id = protocol_action_cmd_id;
        protocol_action_cmd_pending = 0;
        if (id == 0)
        {
            action_group_stop();
        }
        else
        {
            action_group_start(id);
        }
    }

	 if (uart6_rx_complete)
    {
        uart6_cmd_parse(uart6_rx_line);
        uart6_rx_complete = 0;
    }
    else if (uart6_rx_index > 0 && (HAL_GetTick() - uart6_rx_last_tick > 20))
    {
        uart6_rx_line[uart6_rx_index] = '\0';
        uart6_cmd_parse(uart6_rx_line);
        uart6_rx_index = 0;
    }

    // 1ms 控制周期：先更新目标，再算 PID，再发送 CAN，最后反馈状态
    uart3_tx_task();
    bus_servo_uart4_tx_task();
    gripper_servo_boot_task();
    flip_servo_boot_task();

    uint32_t current_tick = HAL_GetTick();
    if (current_tick - last_tick >= 1)
    {
        last_tick = current_tick;
        action_group_task();
gripper_pick_action_task();
gripper_release_action_task();

chassis_gyro_turn_task();
chassis_distance_control_task();
sbus_chassis_task();
chassis_control_task();

lift1_control_task();
lift2_control_task();
lift3_control_task();

if (side_distance_enable)
{
    side_distance_control_task();
}
else
{
    side_wheel_control_task();
}

can2_2006_angle_task();

motor_pid_task();
TIM_CAN_PeriodElapsedCallback();
chassis_feedback_update();
uart6_feedback_task();
uart2_forward_task();

#if UART2_DEBUG_MTF || UART3_DEBUG_MTF
uart2_mtf_feedback_task();
#endif
#if ACTION_FB_ENABLE
if (!uart3_motor_debug_enable)
{
    action_feedback_task();
}
#endif
    }
        
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
