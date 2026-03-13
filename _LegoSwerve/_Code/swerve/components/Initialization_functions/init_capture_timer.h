#ifndef INIT_CAPTURE_TIMER_H
#define INIT_CAPTURE_TIMER_H

#include "driver/mcpwm_cap.h"
#include "motor_data.h"
#include "motor_config.h"

void init_capture_timer(motor_data_t *motor_data, Motor_Config_t *motor_config);

#endif // INIT_CAPTURE_TIMER_H