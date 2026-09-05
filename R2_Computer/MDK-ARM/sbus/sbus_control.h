#ifndef SBUS_CONTROL_H
#define SBUS_CONTROL_H

#include "sbus.h" // 包含 SBUS 库
// --- 移除：#include "usart.h"

// 定义控制范围的最大值
#define CTRL_RANGE_MAX 100.0f

// 定义命令类型枚举
typedef enum {
    CMD_STOP = 0,
    CMD_MOVE = 1,
    // 可以添加更多命令类型
} CommandType_t;

typedef struct {
    float Vx;           // 横向速度，+左移，-右移
    float Vy;           // 纵向速度，+前进，-后退
    float W;            // 角速度，+逆时针，-顺时针
    int CmdEnable;
    float Aux2;
    CommandType_t CmdType;
} ProcessedSbusValue_t;

// 全局变量声明
extern ProcessedSbusValue_t G_SbusValue;

// 函数声明
void SbusControl_Init(void);
void SbusControl_ProcessData(void);

// --- 移除：用于初始化串口句柄的函数声明 ---
// void SbusControl_SetDebugUartHandle(UART_HandleTypeDef *huart);

#endif // SBUS_CONTROL_H
