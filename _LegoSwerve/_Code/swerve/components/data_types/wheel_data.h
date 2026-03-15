#ifndef WHEEL_DATA_H
#define WHEEL_DATA_H

#include <stdint.h>
#include "pid_ctrl.h"

typedef struct {
    pid_ctrl_block_handle_t pid_ctrl;
    float target_wheel_rpm;
    float target_motor_rpm;
    int target_angle;
    int current_angle;
    float angle_error;
    float motor_rpm_differential;
    
} wheel_data_t;

#endif