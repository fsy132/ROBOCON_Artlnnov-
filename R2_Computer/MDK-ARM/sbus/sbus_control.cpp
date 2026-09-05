#include "sbus_control.h"
#include "sbus.h" // 必须包含 sbus.h 以访问 SBUS_CH_Struct, ROLL, PITCH, THROTTLE, YAW, SPEED_MODE 等
#include <stdio.h> // 包含 sprintf (虽然现在在 main.c 中使用，但保留以防万一)
#include <string.h> // 包含 strlen (虽然现在在 main.c 中使用，但保留以防万一)
#include <math.h>
// --- 移除：静态变量存储串口句柄 ---
// static UART_HandleTypeDef *s_huart_debug = NULL;

// --- 移除：初始化串口句柄的函数 ---
// void SbusControl_SetDebugUartHandle(UART_HandleTypeDef *huart) {
//     s_huart_debug = huart;
// }

// 全局变量定义
ProcessedSbusValue_t G_SbusValue = {0};

// --- 移除：定义您想要映射到 Vx, Vy, W 的通道 (保留在 main.c 或 sbus_control.h 中) ---
// #define SBUS_CH_VX  ROLL      // CH4 -> ROLL
// #define SBUS_CH_VY  PITCH     // CH2 -> PITCH
// #define SBUS_CH_W   THROTTLE  // CH3 -> THROTTLE
// #define SBUS_CH_CMD SPEED_MODE // CH6 -> SPEED_MODE
// #define SBUS_CH_AUX1 7        // CH7
// #define SBUS_CH_AUX2 8        // CH8

// --- 移除：定义控制范围的最大值 (保留在 sbus_control.h 中) ---
// #define CTRL_RANGE_MAX 100.0f

// --- 移除：定义 SBUS 原始值范围 (保留在 sbus_control.c 中或作为局部常量) ---
#define SBUS_RAW_MIN 200
#define SBUS_RAW_MID 1028 // 使用 SBUS_RANGE_MIDDLE
#define SBUS_RAW_MAX 1807

// --- 移除：定义您使用的通道 (保留在 main.c 或 sbus_control.h 中) ---
// #define SBUS_CH_VX  ROLL      // CH4 -> ROLL
// #define SBUS_CH_VY  PITCH     // CH2 -> PITCH
// #define SBUS_CH_W   THROTTLE  // CH3 -> THROTTLE
// #define SBUS_CH_CMD SPEED_MODE // CH6 -> SPEED_MODE
// #define SBUS_CH_AUX1 7        // CH7
// #define SBUS_CH_AUX2 8        // CH8

// --- 新增：映射原始值到标准化范围的辅助函数 ---
// 此函数针对摇杆类输入，以中间值为0点，两边对称映射
static float map_stick_to_ctrl(uint16_t raw_value, uint16_t raw_min, uint16_t raw_mid, uint16_t raw_max, float ctrl_range) {
    float raw_range_upper = (float)(raw_max - raw_mid); // 上半部分的原始范围
    float raw_range_lower = (float)(raw_mid - raw_min); // 下半部分的原始范围
    float ctrl_value = 0.0f;

    if (raw_value >= raw_mid) {
        // 值在中间或以上，映射到 0 到 +ctrl_range
        if (raw_value >= raw_max) {
            ctrl_value = ctrl_range;
        } else {
            float normalized = ((float)(raw_value - raw_mid)) / raw_range_upper;
            ctrl_value = normalized * ctrl_range;
        }
    } else {
        // 值在中间以下，映射到 -ctrl_range 到 0
        if (raw_value <= raw_min) {
            ctrl_value = -ctrl_range;
        } else {
            float normalized = ((float)(raw_value - raw_mid)) / raw_range_lower; // 这里 raw_value - raw_mid 是负数
            ctrl_value = normalized * ctrl_range; // ctrl_range 是正数，所以结果是负数
        }
    }
    return ctrl_value;
}

// 全局变量定义
// ProcessedSbusValue_t G_SbusValue = {0}; // 已在上方定义

void SbusControl_Init(void) {
    G_SbusValue.Vx = 0.0f;
    G_SbusValue.Vy = 0.0f;
    G_SbusValue.W = 0.0f;
    G_SbusValue.CmdEnable = 0;
    G_SbusValue.Aux2 = 0.0f;
    G_SbusValue.CmdType = CMD_STOP;
}

// ==================== FS-I6X SBUS 开关量摇杆控制 ====================
// 通道约定：
// CH1：左右横移
// CH2：前进 / 后退
// CH4：原地旋转
// CH5：使能开关，可选
//
// 输出约定：
// G_SbusValue.Vx = -1 / 0 / +1
// G_SbusValue.Vy = -1 / 0 / +1
// G_SbusValue.W  = -1 / 0 / +1

#define SBUS_RAW_MIN             200
#define SBUS_RAW_MID             1028
#define SBUS_RAW_MAX             1807

// 摇杆超过这个阈值才认为“推到极限附近”
// 1028 ± 600，大约推到 75% 以上才触发
#define SBUS_STICK_ON_THRESHOLD  600

// 是否启用 CH5 作为安全开关
// 0：不使用开关，只要 SBUS 在线就允许运动
// 1：使用 CH5，CH5 高电平才允许运动
#define SBUS_USE_ENABLE_SWITCH   0

// 如果方向反了，改这里的正负号
#define SBUS_STRAFE_SIGN         1.0f    // CH1，左右横移方向
#define SBUS_FORWARD_SIGN        1.0f    // CH2，前后方向
#define SBUS_YAW_SIGN            1.0f    // CH4，旋转方向

static int8_t sbus_stick_to_dir(uint16_t raw_value)
{
    if (raw_value > SBUS_RAW_MID + SBUS_STICK_ON_THRESHOLD)
    {
        return 1;
    }
    else if (raw_value < SBUS_RAW_MID - SBUS_STICK_ON_THRESHOLD)
    {
        return -1;
    }

    return 0;
}

void SbusControl_ProcessData(void)
{
    if (!SBUS_CH.ConnectState)
    {
        G_SbusValue.Vx = 0.0f;
        G_SbusValue.Vy = 0.0f;
        G_SbusValue.W = 0.0f;
        G_SbusValue.CmdEnable = 0;
        G_SbusValue.Aux2 = -100.0f;
        G_SbusValue.CmdType = CMD_STOP;
        return;
    }

    // FS-I6X 常用通道：
    // CH1：右摇杆左右
    // CH2：右摇杆上下
    // CH4：左摇杆左右
    uint16_t raw_ch1 = SBUS_CH.CH1;
    uint16_t raw_ch2 = SBUS_CH.CH2;
    uint16_t raw_ch4 = SBUS_CH.CH4;
    uint16_t raw_ch5 = SBUS_CH.CH5;

    int8_t strafe_dir = sbus_stick_to_dir(raw_ch1);
    int8_t forward_dir = sbus_stick_to_dir(raw_ch2);
    int8_t yaw_dir = sbus_stick_to_dir(raw_ch4);

    G_SbusValue.Vx = (float)strafe_dir * SBUS_STRAFE_SIGN;
    G_SbusValue.Vy = (float)forward_dir * SBUS_FORWARD_SIGN;
    G_SbusValue.W = (float)yaw_dir * SBUS_YAW_SIGN;

#if SBUS_USE_ENABLE_SWITCH
    // CH5 高于中值认为使能
    if (raw_ch5 > SBUS_RAW_MID)
    {
        G_SbusValue.CmdEnable = 1;
    }
    else
    {
        G_SbusValue.CmdEnable = 0;
    }
#else
    G_SbusValue.CmdEnable = 1;
#endif

    if (G_SbusValue.CmdEnable)
    {
        G_SbusValue.CmdType = CMD_MOVE;
    }
    else
    {
        G_SbusValue.CmdType = CMD_STOP;
        G_SbusValue.Vx = 0.0f;
        G_SbusValue.Vy = 0.0f;
        G_SbusValue.W = 0.0f;
    }
}
