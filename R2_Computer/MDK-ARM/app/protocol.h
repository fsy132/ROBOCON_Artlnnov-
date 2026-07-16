// Generated at: 2026-07-07T18:48:55+08:00
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

// 协议哈希校验码
#define PROTOCOL_HASH 0x2DD00572

// 校验算法: CRC8
#define CHECKSUM_ALGO_CRC8 1

// 握手配置
#define CFG_REQUIRE_HANDSHAKE 0

// 心跳配置
#define CFG_ENABLE_HEARTBEAT 1

/* USER CODE BEGIN Private_Defines */
/* USER CODE END Private_Defines */

// 帧头定义
#define FRAME_HEADER1 90
#define FRAME_HEADER2 165

// 数据包ID定义
typedef enum {
    PACKET_ID_HEARTBEAT = 0,
    PACKET_ID_HANDSHAKE = 255,
    PACKET_ID_CMDVEL = 1,
    PACKET_ID_MEILINCMD = 4,
    PACKET_ID_CONFRONTCLIMBCMD = 11,
    PACKET_ID_WEAPONFLIPCMD = 10,
    PACKET_ID_GRIPPERSTATUS = 16,
    PACKET_ID_ASSEMBLYSTATUS = 17,
    PACKET_ID_WHEELODOM = 37,
    PACKET_ID_ACTIONGROUPCMD = 9,
    PACKET_ID_ACTIONGROUPFEEDBACK = 20,
    PACKET_ID_STARTBUTTON = 18,
    PACKET_ID_R1SIGNAL = 19,
    PACKET_ID_GAMECMD = 8,
} PacketID;

#pragma pack(1)
typedef struct {
    uint32_t count;
} Packet_Heartbeat;

typedef struct {
    uint32_t protocol_hash;
} Packet_Handshake;

typedef struct {
    float vx;
    float vy;
    float rotation;
} Packet_CmdVel;

typedef struct {
    float next_block;
    float climb_mode;
    float slope_angle;
    float block_height;
    float detour;
} Packet_MeilinCmd;

typedef struct {
    uint8_t mode;
} Packet_ConfrontClimbCmd;

typedef struct {
    uint8_t flip;
} Packet_WeaponFlipCmd;

typedef struct {
    uint8_t status;
} Packet_GripperStatus;

typedef struct {
    uint8_t status;
} Packet_AssemblyStatus;

typedef struct {
    float pos_x;
    float pos_y;
    float yaw;
    float vx;
    float vy;
    float omega;
} Packet_WheelOdom;

typedef struct {
    uint8_t action_id;
} Packet_ActionGroupCmd;

typedef struct {
    uint8_t status;
} Packet_ActionGroupFeedback;

typedef struct {
    uint8_t pressed;
} Packet_StartButton;

typedef struct {
    uint8_t signal;
} Packet_R1Signal;

typedef struct {
    uint8_t cmd;
} Packet_GameCmd;

#pragma pack()

// 协议辅助函数声明
uint8_t calculate_checksum(const uint8_t* data, uint8_t len);
void protocol_fsm_feed(uint8_t byte);

// 用户可覆盖的接收回调与自动生成的发送函数声明
void on_receive_Heartbeat(const Packet_Heartbeat* pkt);
void send_Heartbeat(const Packet_Heartbeat* pkt);
void on_receive_Handshake(const Packet_Handshake* pkt);
void send_Handshake(const Packet_Handshake* pkt);
void on_receive_CmdVel(const Packet_CmdVel* pkt);
void send_CmdVel(const Packet_CmdVel* pkt);
void on_receive_MeilinCmd(const Packet_MeilinCmd* pkt);
void send_MeilinCmd(const Packet_MeilinCmd* pkt);
void on_receive_ConfrontClimbCmd(const Packet_ConfrontClimbCmd* pkt);
void send_ConfrontClimbCmd(const Packet_ConfrontClimbCmd* pkt);
void on_receive_WeaponFlipCmd(const Packet_WeaponFlipCmd* pkt);
void send_WeaponFlipCmd(const Packet_WeaponFlipCmd* pkt);
void on_receive_GripperStatus(const Packet_GripperStatus* pkt);
void send_GripperStatus(const Packet_GripperStatus* pkt);
void on_receive_AssemblyStatus(const Packet_AssemblyStatus* pkt);
void send_AssemblyStatus(const Packet_AssemblyStatus* pkt);
void on_receive_WheelOdom(const Packet_WheelOdom* pkt);
void send_WheelOdom(const Packet_WheelOdom* pkt);
void on_receive_ActionGroupCmd(const Packet_ActionGroupCmd* pkt);
void send_ActionGroupCmd(const Packet_ActionGroupCmd* pkt);
void on_receive_ActionGroupFeedback(const Packet_ActionGroupFeedback* pkt);
void send_ActionGroupFeedback(const Packet_ActionGroupFeedback* pkt);
void on_receive_StartButton(const Packet_StartButton* pkt);
void send_StartButton(const Packet_StartButton* pkt);
void on_receive_R1Signal(const Packet_R1Signal* pkt);
void send_R1Signal(const Packet_R1Signal* pkt);
void on_receive_GameCmd(const Packet_GameCmd* pkt);
void send_GameCmd(const Packet_GameCmd* pkt);

/* USER CODE BEGIN User_Types */
/* USER CODE END User_Types */

// CRC8查找表 (多项式 0x31)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC,
};

#ifdef __cplusplus
}
#endif
