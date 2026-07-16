// Generated at: 2026-07-07T18:48:55+08:00
#include "protocol.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


// 解析器状态定义
typedef enum {
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_WAIT_ID,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC
} State;

static State rx_state = STATE_WAIT_HEADER1;
static uint8_t rx_buffer[256]; // 定义的最大包长
static uint16_t rx_cnt = 0;
static uint8_t rx_data_len = 0;
static uint8_t rx_id = 0;
static uint8_t rx_checksum = 0;

// CRC8 校验函数 (查表法, 多项式 0x31)
static uint8_t checksum_update(uint8_t current, uint8_t byte) {
    return CRC8_TABLE[current ^ byte];
}

uint8_t calculate_checksum(const uint8_t* data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs = checksum_update(cs, data[i]);
    }
    return cs;
}

/* USER CODE BEGIN Private_Variables */
/* USER CODE END Private_Variables */


// 用户需要实现的回调函数
__weak void on_receive_Heartbeat(const Packet_Heartbeat* pkt) {
    // Default system behavior: ack the latest heartbeat with the same count.
    send_Heartbeat(pkt);
/* USER CODE BEGIN on_receive_Heartbeat */
/* USER CODE END on_receive_Heartbeat */
}
__weak void on_receive_Handshake(const Packet_Handshake* pkt) {
/* USER CODE BEGIN on_receive_Handshake */
/* USER CODE END on_receive_Handshake */
}
__weak void on_receive_CmdVel(const Packet_CmdVel* pkt) {
/* USER CODE BEGIN on_receive_CmdVel */
/* USER CODE END on_receive_CmdVel */
}
__weak void on_receive_MeilinCmd(const Packet_MeilinCmd* pkt) {
/* USER CODE BEGIN on_receive_MeilinCmd */
/* USER CODE END on_receive_MeilinCmd */
}
__weak void on_receive_ConfrontClimbCmd(const Packet_ConfrontClimbCmd* pkt) {
/* USER CODE BEGIN on_receive_ConfrontClimbCmd */
/* USER CODE END on_receive_ConfrontClimbCmd */
}
__weak void on_receive_WeaponFlipCmd(const Packet_WeaponFlipCmd* pkt) {
/* USER CODE BEGIN on_receive_WeaponFlipCmd */
/* USER CODE END on_receive_WeaponFlipCmd */
}
__weak void on_receive_GripperStatus(const Packet_GripperStatus* pkt) {
/* USER CODE BEGIN on_receive_GripperStatus */
/* USER CODE END on_receive_GripperStatus */
}
__weak void on_receive_AssemblyStatus(const Packet_AssemblyStatus* pkt) {
/* USER CODE BEGIN on_receive_AssemblyStatus */
/* USER CODE END on_receive_AssemblyStatus */
}
__weak void on_receive_WheelOdom(const Packet_WheelOdom* pkt) {
/* USER CODE BEGIN on_receive_WheelOdom */
/* USER CODE END on_receive_WheelOdom */
}
__weak void on_receive_ActionGroupCmd(const Packet_ActionGroupCmd* pkt) {
/* USER CODE BEGIN on_receive_ActionGroupCmd */
/* USER CODE END on_receive_ActionGroupCmd */
}
__weak void on_receive_ActionGroupFeedback(const Packet_ActionGroupFeedback* pkt) {
/* USER CODE BEGIN on_receive_ActionGroupFeedback */
/* USER CODE END on_receive_ActionGroupFeedback */
}
__weak void on_receive_StartButton(const Packet_StartButton* pkt) {
/* USER CODE BEGIN on_receive_StartButton */
/* USER CODE END on_receive_StartButton */
}
__weak void on_receive_R1Signal(const Packet_R1Signal* pkt) {
/* USER CODE BEGIN on_receive_R1Signal */
/* USER CODE END on_receive_R1Signal */
}
__weak void on_receive_GameCmd(const Packet_GameCmd* pkt) {
/* USER CODE BEGIN on_receive_GameCmd */
/* USER CODE END on_receive_GameCmd */
}

/* USER CODE BEGIN Code_0 */
/* USER CODE END Code_0 */


/**
 * @brief 协议解析状态机，在串口中断或轮询中调用此函数处理每个接收到的字节
 * @param byte 接收到的单个字节
 */
void protocol_fsm_feed(uint8_t byte) {
    switch (rx_state) {
        case STATE_WAIT_HEADER1:
            if (byte == FRAME_HEADER1) {
                rx_state = STATE_WAIT_HEADER2;
                rx_checksum = 0; // 校验重置，不包含 Frame Header
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == FRAME_HEADER2) {
                rx_state = STATE_WAIT_ID;
            } else {
                rx_state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_ID:
            rx_id = byte;
            rx_checksum = checksum_update(0, rx_id); // 校验包含 ID
            rx_state = STATE_WAIT_LEN;
            break;
            
        case STATE_WAIT_LEN:
            rx_data_len = byte;
            rx_checksum = checksum_update(rx_checksum, rx_data_len); // 校验包含 Len
            rx_cnt = 0;
            if (rx_data_len > 0) {
                rx_state = STATE_WAIT_DATA;
            } else {
                rx_state = STATE_WAIT_CRC;
            }
            break;
            
        case STATE_WAIT_DATA:
            if (rx_cnt < sizeof(rx_buffer)) {
                rx_buffer[rx_cnt++] = byte;
                rx_checksum = checksum_update(rx_checksum, byte);
                if (rx_cnt >= rx_data_len) {
                    rx_state = STATE_WAIT_CRC;
                }
            } else {
                rx_state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_CRC:
            if (byte == rx_checksum) {
                // 校验通过，分发数据
                switch (rx_id) {
                    case PACKET_ID_HEARTBEAT:
                        if (rx_data_len == sizeof(Packet_Heartbeat)) {
                            on_receive_Heartbeat((Packet_Heartbeat*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_HANDSHAKE:
                        if (rx_data_len == sizeof(Packet_Handshake)) {
                            on_receive_Handshake((Packet_Handshake*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_CMDVEL:
                        if (rx_data_len == sizeof(Packet_CmdVel)) {
                            on_receive_CmdVel((Packet_CmdVel*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_MEILINCMD:
                        if (rx_data_len == sizeof(Packet_MeilinCmd)) {
                            on_receive_MeilinCmd((Packet_MeilinCmd*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_CONFRONTCLIMBCMD:
                        if (rx_data_len == sizeof(Packet_ConfrontClimbCmd)) {
                            on_receive_ConfrontClimbCmd((Packet_ConfrontClimbCmd*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_WEAPONFLIPCMD:
                        if (rx_data_len == sizeof(Packet_WeaponFlipCmd)) {
                            on_receive_WeaponFlipCmd((Packet_WeaponFlipCmd*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_GRIPPERSTATUS:
                        if (rx_data_len == sizeof(Packet_GripperStatus)) {
                            on_receive_GripperStatus((Packet_GripperStatus*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_ASSEMBLYSTATUS:
                        if (rx_data_len == sizeof(Packet_AssemblyStatus)) {
                            on_receive_AssemblyStatus((Packet_AssemblyStatus*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_WHEELODOM:
                        if (rx_data_len == sizeof(Packet_WheelOdom)) {
                            on_receive_WheelOdom((Packet_WheelOdom*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_ACTIONGROUPCMD:
                        if (rx_data_len == sizeof(Packet_ActionGroupCmd)) {
                            on_receive_ActionGroupCmd((Packet_ActionGroupCmd*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_ACTIONGROUPFEEDBACK:
                        if (rx_data_len == sizeof(Packet_ActionGroupFeedback)) {
                            on_receive_ActionGroupFeedback((Packet_ActionGroupFeedback*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_STARTBUTTON:
                        if (rx_data_len == sizeof(Packet_StartButton)) {
                            on_receive_StartButton((Packet_StartButton*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_R1SIGNAL:
                        if (rx_data_len == sizeof(Packet_R1Signal)) {
                            on_receive_R1Signal((Packet_R1Signal*)rx_buffer);
                        }
                        break;
                    case PACKET_ID_GAMECMD:
                        if (rx_data_len == sizeof(Packet_GameCmd)) {
                            on_receive_GameCmd((Packet_GameCmd*)rx_buffer);
                        }
                        break;

                    default:
                        break;
                }
            }
            rx_state = STATE_WAIT_HEADER1;
            break;
            
        default:
            rx_state = STATE_WAIT_HEADER1;
            break;
    }
}

// --- 发送函数 ---
// 外部依赖：用户必须实现 void serial_write(const uint8_t* data, uint16_t len);
extern void serial_write(const uint8_t* data, uint16_t len);

void send_Heartbeat(const Packet_Heartbeat* pkt) {
    uint8_t buffer[4 + sizeof(Packet_Heartbeat) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_HEARTBEAT;
    buffer[idx++] = sizeof(Packet_Heartbeat);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_Heartbeat));
    idx += sizeof(Packet_Heartbeat);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_Handshake(const Packet_Handshake* pkt) {
    uint8_t buffer[4 + sizeof(Packet_Handshake) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_HANDSHAKE;
    buffer[idx++] = sizeof(Packet_Handshake);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_Handshake));
    idx += sizeof(Packet_Handshake);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_CmdVel(const Packet_CmdVel* pkt) {
    uint8_t buffer[4 + sizeof(Packet_CmdVel) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_CMDVEL;
    buffer[idx++] = sizeof(Packet_CmdVel);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_CmdVel));
    idx += sizeof(Packet_CmdVel);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_MeilinCmd(const Packet_MeilinCmd* pkt) {
    uint8_t buffer[4 + sizeof(Packet_MeilinCmd) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_MEILINCMD;
    buffer[idx++] = sizeof(Packet_MeilinCmd);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_MeilinCmd));
    idx += sizeof(Packet_MeilinCmd);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_ConfrontClimbCmd(const Packet_ConfrontClimbCmd* pkt) {
    uint8_t buffer[4 + sizeof(Packet_ConfrontClimbCmd) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_CONFRONTCLIMBCMD;
    buffer[idx++] = sizeof(Packet_ConfrontClimbCmd);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_ConfrontClimbCmd));
    idx += sizeof(Packet_ConfrontClimbCmd);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_WeaponFlipCmd(const Packet_WeaponFlipCmd* pkt) {
    uint8_t buffer[4 + sizeof(Packet_WeaponFlipCmd) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_WEAPONFLIPCMD;
    buffer[idx++] = sizeof(Packet_WeaponFlipCmd);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_WeaponFlipCmd));
    idx += sizeof(Packet_WeaponFlipCmd);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_GripperStatus(const Packet_GripperStatus* pkt) {
    uint8_t buffer[4 + sizeof(Packet_GripperStatus) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_GRIPPERSTATUS;
    buffer[idx++] = sizeof(Packet_GripperStatus);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_GripperStatus));
    idx += sizeof(Packet_GripperStatus);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_AssemblyStatus(const Packet_AssemblyStatus* pkt) {
    uint8_t buffer[4 + sizeof(Packet_AssemblyStatus) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_ASSEMBLYSTATUS;
    buffer[idx++] = sizeof(Packet_AssemblyStatus);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_AssemblyStatus));
    idx += sizeof(Packet_AssemblyStatus);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_WheelOdom(const Packet_WheelOdom* pkt) {
    uint8_t buffer[4 + sizeof(Packet_WheelOdom) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_WHEELODOM;
    buffer[idx++] = sizeof(Packet_WheelOdom);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_WheelOdom));
    idx += sizeof(Packet_WheelOdom);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_ActionGroupCmd(const Packet_ActionGroupCmd* pkt) {
    uint8_t buffer[4 + sizeof(Packet_ActionGroupCmd) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_ACTIONGROUPCMD;
    buffer[idx++] = sizeof(Packet_ActionGroupCmd);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_ActionGroupCmd));
    idx += sizeof(Packet_ActionGroupCmd);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_ActionGroupFeedback(const Packet_ActionGroupFeedback* pkt) {
    uint8_t buffer[4 + sizeof(Packet_ActionGroupFeedback) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_ACTIONGROUPFEEDBACK;
    buffer[idx++] = sizeof(Packet_ActionGroupFeedback);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_ActionGroupFeedback));
    idx += sizeof(Packet_ActionGroupFeedback);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_StartButton(const Packet_StartButton* pkt) {
    uint8_t buffer[4 + sizeof(Packet_StartButton) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_STARTBUTTON;
    buffer[idx++] = sizeof(Packet_StartButton);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_StartButton));
    idx += sizeof(Packet_StartButton);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_R1Signal(const Packet_R1Signal* pkt) {
    uint8_t buffer[4 + sizeof(Packet_R1Signal) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_R1SIGNAL;
    buffer[idx++] = sizeof(Packet_R1Signal);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_R1Signal));
    idx += sizeof(Packet_R1Signal);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}
void send_GameCmd(const Packet_GameCmd* pkt) {
    uint8_t buffer[4 + sizeof(Packet_GameCmd) + 1];
    uint16_t idx = 0;
    
    buffer[idx++] = FRAME_HEADER1;
    buffer[idx++] = FRAME_HEADER2;
    buffer[idx++] = PACKET_ID_GAMECMD;
    buffer[idx++] = sizeof(Packet_GameCmd);
    
    memcpy(&buffer[idx], pkt, sizeof(Packet_GameCmd));
    idx += sizeof(Packet_GameCmd);
    
    buffer[idx] = calculate_checksum(&buffer[2], idx - 2);
    idx++;
    
    serial_write(buffer, idx);
}

/* USER CODE BEGIN Code_1 */
/* USER CODE END Code_1 */

/*
// --- 建议的消息发送模板 (以 Heartbeat 为例) ---
// 建议在定时器回调或主循环中以固定频率调用

void heartbeat_timer_callback(void) {
    static uint32_t hb_count = 0;
    Packet_Heartbeat pkt;
    pkt.count = hb_count++;
    send_Heartbeat(&pkt);
}
*/

#ifdef __cplusplus
}
#endif
