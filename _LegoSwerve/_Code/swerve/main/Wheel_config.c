#include "wheel_config.h"

Wheel_Config_t wheel_config = {
    .wheel_rpm_ratio = 1.0f,
    .max_rpm = 11500,
    .min_rpm = -9500,
    .kp = 4.0f,
    .ki = 0.05f,
    .kd = 0.000f
};