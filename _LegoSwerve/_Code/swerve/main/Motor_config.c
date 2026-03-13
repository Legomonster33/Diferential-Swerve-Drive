#include "motor_config.h"

Motor_Config_t motor_1_config = {
    .pwm_pin = 25,
    .hall_pin = 32,
    .max_rpm = 11500,
    .min_rpm = -9500,
    .kp = 0.2f,
    .ki = 0.005f,
    .kd = 0.000f
};

Motor_Config_t motor_2_config = {
    .pwm_pin = 26,
    .hall_pin = 33,
    .max_rpm = 11500,
    .min_rpm = -9500,
    .kp = 0.2f,
    .ki = 0.005f,
    .kd = 0.000f
};