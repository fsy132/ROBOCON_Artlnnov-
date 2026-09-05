#pragma once

#include <stdint.h>

void hwt101_decode(uint8_t data);
float hwt101_get_yaw_deg(void);
float angle_diff_deg(float target_deg, float now_deg);
uint8_t hwt101_is_valid(void);
void hwt101_set_zero(void);
void chassis_gyro_turn_start(float relative_deg);
uint8_t chassis_gyro_turn_task(void);
