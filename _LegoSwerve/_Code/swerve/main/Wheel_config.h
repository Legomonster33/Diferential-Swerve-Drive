#ifndef WHEEL_CONFIG_H
#define WHEEL_CONFIG_H

typedef struct {
    float wheel_rpm_ratio;
    float max_rpm;
    float min_rpm;
    float kp;
    float ki;
    float kd;
} Wheel_Config_t;

extern Wheel_Config_t wheel_config;

#endif // WHEEL_CONFIG_H