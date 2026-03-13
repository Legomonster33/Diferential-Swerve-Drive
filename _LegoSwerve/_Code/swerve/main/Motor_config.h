#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

typedef struct {
    int pwm_pin;
    int hall_pin;
    int max_rpm;
    int min_rpm;
    float kp;
    float ki;
    float kd;
} MotorConfig;

MotorConfig motor_1_config;
MotorConfig motor_2_config;

#endif // MOTOR_CONFIG_H