#ifndef INIT_PWM_OPERATOR_H
#define INIT_PWM_OPERATOR_H

#include "driver/mcpwm_prelude.h"
#include "motor_data.h"
#include "motor_config.h"

#define TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        5000    // 5000 ticks, 5ms

void init_pwm_operator(motor_data_t *motor_data, const Motor_Config_t *motor_config);

#endif // INIT_PWM_OPERATOR_H