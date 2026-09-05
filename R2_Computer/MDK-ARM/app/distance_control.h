#pragma once

#include <stdint.h>

void chassis_distance_start(uint8_t mtf_index, float distance_mm);
void chassis_distance_control_task(void);
void side_distance_select_mtf(uint8_t index);
void side_distance_start(uint8_t mtf_index, float distance_mm);
void side_distance_control_task(void);
