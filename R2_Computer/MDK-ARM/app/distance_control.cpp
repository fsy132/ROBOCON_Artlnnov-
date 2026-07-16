#include "distance_control.h"

#include "app_config.h"
#include "app_shared.h"
#include "main.h"
#include "mtf01.h"

void chassis_distance_start(uint8_t mtf_index, float distance_mm)
{
    if (mtf_index >= MTF01_COUNT)
    {
        chassis_distance_enable = 0;
        chassis_distance_failed = 1;
        return;
    }

    chassis_distance_mtf_index = mtf_index;
    chassis_distance_target_mm = distance_mm;
    chassis_distance_now_mm = 0.0f;
    chassis_distance_enable = 1;
    chassis_distance_failed = 0;
    chassis_distance_last_tick = 0;
    chassis_distance_mtf_last_update_tick = mtf01_data[mtf_index].update_tick;
    chassis_distance_mtf_last_update_ms = HAL_GetTick();
}

void chassis_distance_control_task(void)
{
    uint32_t now = HAL_GetTick();
    volatile MTF01_Data_t *mtf = &mtf01_data[chassis_distance_mtf_index];

    if (!chassis_distance_enable)
    {
        return;
    }

    if (!mtf->distance_valid || !mtf->flow_valid)
    {
        chassis_cmd_vx_mm_s = 0.0f;
        chassis_cmd_vy_mm_s = 0.0f;
        chassis_cmd_w_mdeg_s = 0.0f;
        chassis_distance_last_tick = 0;
        chassis_distance_failed = 1;
        return;
    }

    if (mtf->update_tick != chassis_distance_mtf_last_update_tick)
    {
        chassis_distance_mtf_last_update_tick = mtf->update_tick;
        chassis_distance_mtf_last_update_ms = now;
    }
    else if (now - chassis_distance_mtf_last_update_ms > 50)
    {
        chassis_cmd_vx_mm_s = 0.0f;
        chassis_cmd_vy_mm_s = 0.0f;
        chassis_cmd_w_mdeg_s = 0.0f;
        chassis_distance_last_tick = 0;
        chassis_distance_failed = 1;
        return;
    }

    if (chassis_distance_last_tick == 0)
    {
        chassis_distance_last_tick = now;
        return;
    }

    float dt = (float)(now - chassis_distance_last_tick) / 1000.0f;
    chassis_distance_last_tick = now;

    float height_m = (float)mtf->distance_mm / 1000.0f;
    float vel_y_mm_s = (float)mtf->flow_vel_y * height_m * 10.0f;

    chassis_distance_now_mm += vel_y_mm_s * dt;

    float error_mm = chassis_distance_target_mm - chassis_distance_now_mm;

    if (error_mm < CHASSIS_DISTANCE_STOP_MM && error_mm > -CHASSIS_DISTANCE_STOP_MM)
    {
        chassis_distance_enable = 0;
        chassis_cmd_vx_mm_s = 0.0f;
        chassis_cmd_vy_mm_s = 0.0f;
        chassis_cmd_w_mdeg_s = 0.0f;
        chassis_cmd_last_tick = HAL_GetTick();
        return;
    }

    float speed_mm_s = error_mm * CHASSIS_DISTANCE_KP;

    if (speed_mm_s > CHASSIS_DISTANCE_MAX_MM_S) speed_mm_s = CHASSIS_DISTANCE_MAX_MM_S;
    if (speed_mm_s < -CHASSIS_DISTANCE_MAX_MM_S) speed_mm_s = -CHASSIS_DISTANCE_MAX_MM_S;

    if (speed_mm_s > 0.0f && speed_mm_s < CHASSIS_DISTANCE_MIN_MM_S) speed_mm_s = CHASSIS_DISTANCE_MIN_MM_S;
    if (speed_mm_s < 0.0f && speed_mm_s > -CHASSIS_DISTANCE_MIN_MM_S) speed_mm_s = -CHASSIS_DISTANCE_MIN_MM_S;

    chassis_cmd_vx_mm_s = 0.0f;
    chassis_cmd_vy_mm_s = speed_mm_s;
    chassis_cmd_w_mdeg_s = 0.0f;
    chassis_cmd_last_tick = HAL_GetTick();
}

void side_distance_control_task(void)
{
    const float kp = 0.35f;
    const float stop_error_mm = 25.0f;
    const float max_rpm = 300.0f;
    const float min_rpm = 30.0f;

    uint32_t now = HAL_GetTick();
    volatile MTF01_Data_t *mtf = &mtf01_data[active_mtf01_index];

    if (!side_distance_enable)
    {
        return;
    }

    if (!mtf->distance_valid || !mtf->flow_valid)
    {
        motor9.Set_Target_Omega(0.0f);
        motor10.Set_Target_Omega(0.0f);
        motor11.Set_Target_Omega(0.0f);
        motor15.Set_Target_Omega(0.0f);
        side_distance_last_tick = 0;
        side_distance_failed = 1;
        return;
    }

    if (mtf->update_tick != mtf01_last_update_tick)
    {
        mtf01_last_update_tick = mtf->update_tick;
        mtf01_last_update_ms = now;
    }
    else if (now - mtf01_last_update_ms > 50)
    {
        motor9.Set_Target_Omega(0.0f);
        motor10.Set_Target_Omega(0.0f);
        motor11.Set_Target_Omega(0.0f);
        motor15.Set_Target_Omega(0.0f);
        side_distance_last_tick = 0;
        side_distance_failed = 1;
        return;
    }

    if (side_distance_last_tick == 0)
    {
        side_distance_last_tick = now;
        return;
    }

    float dt = (float)(now - side_distance_last_tick) / 1000.0f;
    side_distance_last_tick = now;

    float height_m = (float)mtf->distance_mm / 1000.0f;
    float vel_y_mm_s = (float)mtf->flow_vel_y * height_m * 10.0f;

    side_distance_now_mm += vel_y_mm_s * dt;

    float error_mm = side_distance_target_mm - side_distance_now_mm;

    if (error_mm < stop_error_mm && error_mm > -stop_error_mm)
    {
        side_distance_enable = 0;
        motor9.Set_Target_Omega(0.0f);
        motor10.Set_Target_Omega(0.0f);
        motor11.Set_Target_Omega(0.0f);
        motor15.Set_Target_Omega(0.0f);
        return;
    }

    float rpm = kp * error_mm;

    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -max_rpm) rpm = -max_rpm;

    if (rpm > 0.0f && rpm < min_rpm) rpm = min_rpm;
    if (rpm < 0.0f && rpm > -min_rpm) rpm = -min_rpm;

    float omega = rpm * SIDE_WHEEL_RPM_TO_RADPS;

    motor9.Set_Target_Omega(SIDE_WHEEL9_DIR * omega);
    motor10.Set_Target_Omega(SIDE_WHEEL10_DIR * omega);
    motor11.Set_Target_Omega(SIDE_WHEEL11_DIR * omega);
    motor15.Set_Target_Omega(SIDE_WHEEL15_DIR * omega);
}

void side_distance_select_mtf(uint8_t index)
{
    if (index >= MTF01_COUNT)
    {
        return;
    }

    active_mtf01_index = index;

    side_distance_now_mm = 0.0f;
    side_distance_last_tick = 0;
    mtf01_last_update_tick = mtf01_data[active_mtf01_index].update_tick;
    mtf01_last_update_ms = HAL_GetTick();
}

void side_distance_start(uint8_t mtf_index, float distance_mm)
{
    if (mtf_index >= MTF01_COUNT)
    {
        side_distance_enable = 0;
        side_distance_failed = 1;
        motor9.Set_Target_Omega(0.0f);
        motor10.Set_Target_Omega(0.0f);
        motor11.Set_Target_Omega(0.0f);
        motor15.Set_Target_Omega(0.0f);
        return;
    }

    side_distance_select_mtf(mtf_index);

    side_distance_target_mm = distance_mm;
    side_distance_now_mm = 0.0f;
    side_distance_enable = 1;
    side_distance_failed = 0;
    side_distance_last_tick = 0;
}
