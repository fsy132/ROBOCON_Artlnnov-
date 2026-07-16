#include "mtf01.h"
volatile MTF01_Data_t mtf01_data[MTF01_COUNT];
/*
说明： 用户使用micolink_decode作为串口数据处理函数即可

距离有效值最小为10(mm),为0说明此时距离值不可用
光流速度值单位：cm/s@1m
飞控中只需要将光流速度值*高度，即可得到真实水平位移速度
计算公式：实际速度(cm/s)=光流速度*高度(m)
*/

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);

void mtf01_decode(uint8_t index, uint8_t data)
{
    static MICOLINK_MSG_t msg[MTF01_COUNT];

    if (index >= MTF01_COUNT)
    {
        return;
    }

    if (micolink_parse_char(&msg[index], data) == false)
    {
        return;
    }

    switch (msg[index].msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            if (msg[index].len != sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t))
            {
                break;
            }

            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
            memcpy(&payload, msg[index].payload, sizeof(payload));

            mtf01_data[index].time_ms = payload.time_ms;
            mtf01_data[index].distance_mm = payload.distance;
            mtf01_data[index].strength = payload.strength;
            mtf01_data[index].precision = payload.precision;
            mtf01_data[index].tof_status = payload.tof_status;
            mtf01_data[index].flow_vel_x = payload.flow_vel_x;
            mtf01_data[index].flow_vel_y = payload.flow_vel_y;
            mtf01_data[index].flow_quality = payload.flow_quality;
            mtf01_data[index].flow_status = payload.flow_status;

            mtf01_data[index].distance_valid = (payload.distance >= 10 && payload.tof_status == 1);
            mtf01_data[index].flow_valid = (payload.flow_status == 1);
            mtf01_data[index].update_tick++;
            break;
        }

        default:
            break;
    }
}

bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:     //帧头
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;
        
    case 1:     // 设备ID
        msg->dev_id = data;
        msg->status++;
        break;
    
    case 2:     // 系统ID
        msg->sys_id = data;
        msg->status++;
        break;
    
    case 3:     // 消息ID
        msg->msg_id = data;
        msg->status++;
        break;
    
    case 4:     // 包序列
        msg->seq = data;
        msg->status++;
        break;
    
    case 5:     // 负载长度
        msg->len = data;
        if(msg->len == 0)
            msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;
        
    case 6:     // 数据负载接收
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;
        
    case 7:     // 帧校验
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return true;
        }
        
    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}
