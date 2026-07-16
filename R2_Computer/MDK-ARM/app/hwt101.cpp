#include "hwt101.h"

#include "app_config.h"
#include "app_shared.h"
#include "main.h"

static int16_t hwt101_i16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

float angle_diff_deg(float target_deg, float now_deg)
{
    float diff = target_deg - now_deg;

    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}

float hwt101_get_yaw_deg(void)
{
    return angle_diff_deg(hwt101_yaw_deg, hwt101_yaw_zero_deg);
}

uint8_t hwt101_is_valid(void)
{
    if (!hwt101_angle_valid)
    {
        return 0;
    }

    return (HAL_GetTick() - hwt101_last_update_tick <= HWT101_YAW_VALID_TIMEOUT_MS);
}

void hwt101_set_zero(void)
{
    if (hwt101_is_valid())
    {
        hwt101_yaw_zero_deg = hwt101_yaw_deg;
    }
}

void hwt101_decode(uint8_t data)
{
    static uint8_t frame[HWT101_FRAME_LEN];
    static uint8_t index = 0;

    if (index == 0 && data != HWT101_FRAME_HEAD)
    {
        return;
    }

    frame[index++] = data;

    if (index < HWT101_FRAME_LEN)
    {
        return;
    }

    index = 0;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        sum += frame[i];
    }

    if (sum != frame[10])
    {
        return;
    }

    if (frame[1] == HWT101_FRAME_GYRO)
    {
        int16_t wz_raw = hwt101_i16(frame[6], frame[7]);
        hwt101_gyro_z_dps = (float)wz_raw / 32768.0f * HWT101_GYRO_RANGE_DPS;
    }
    else if (frame[1] == HWT101_FRAME_ANGLE)
    {
        int16_t yaw_raw = hwt101_i16(frame[6], frame[7]);
        hwt101_yaw_deg = (float)yaw_raw / 32768.0f * 180.0f;

        hwt101_angle_valid = 1;
        hwt101_last_update_tick = HAL_GetTick();
    }
}

void chassis_gyro_turn_start(float relative_deg)
{
    if (!hwt101_is_valid())
    {
        chassis_gyro_turn_enable = 0;
        return;
    }

    chassis_gyro_turn_target_deg = hwt101_get_yaw_deg() + relative_deg;

    while (chassis_gyro_turn_target_deg > 180.0f) chassis_gyro_turn_target_deg -= 360.0f;
    while (chassis_gyro_turn_target_deg < -180.0f) chassis_gyro_turn_target_deg += 360.0f;

    chassis_gyro_turn_enable = 1;
    chassis_yaw_hold_enable = 0;
}

uint8_t chassis_gyro_turn_task(void)
{
    if (!chassis_gyro_turn_enable)
    {
        return 0;
    }

    if (!hwt101_is_valid())
    {
        chassis_gyro_turn_enable = 0;
        chassis_cmd_w_mdeg_s = 0.0f;
        return 1;
    }

    float err_deg = angle_diff_deg(chassis_gyro_turn_target_deg, hwt101_get_yaw_deg());

    if (err_deg < HWT101_TURN_DONE_DEG && err_deg > -HWT101_TURN_DONE_DEG)
    {
        chassis_gyro_turn_enable = 0;
        chassis_cmd_vx_mm_s = 0.0f;
        chassis_cmd_vy_mm_s = 0.0f;
        chassis_cmd_w_mdeg_s = 0.0f;
        chassis_cmd_last_tick = HAL_GetTick();
        return 1;
    }

    float wz_mdeg_s = err_deg * HWT101_TURN_KP_MDEG;

    if (wz_mdeg_s > HWT101_TURN_MAX_MDEG_S) wz_mdeg_s = HWT101_TURN_MAX_MDEG_S;
    if (wz_mdeg_s < -HWT101_TURN_MAX_MDEG_S) wz_mdeg_s = -HWT101_TURN_MAX_MDEG_S;

    chassis_cmd_vx_mm_s = 0.0f;
    chassis_cmd_vy_mm_s = 0.0f;
    chassis_cmd_w_mdeg_s = wz_mdeg_s;
    chassis_cmd_last_tick = HAL_GetTick();

    return 0;
}
