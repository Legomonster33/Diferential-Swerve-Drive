#ifndef WHEEL_DATA_H
#define WHEEL_DATA_H

#include <stdint.h>
#include "pid_ctrl.h"

typedef struct {
    pid_ctrl_block_handle_t pid_ctrl;
    float target_wheel_surface_speed;
    float current_wheel_surface_speed;
    float target_motor_rpm;
    uint16_t target_angle;
    uint16_t current_angle;
    uint16_t real_angle;
    float angle_error;
    float motor_rpm_differential;
    bool drive_reverse;
    
} wheel_data_t;

#endif