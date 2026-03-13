#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

#include <stdint.h>
#include "pid_ctrl.h"
#include "hall_data.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_cap.h"

typedef struct {
    hall_data_t hall_data;
    pid_ctrl_block_handle_t pid_ctrl;
    mcpwm_cmpr_handle_t pwm_comparator;
    float rpm;
    float target_rpm;
    float error;
    float new_speed;
    float feedforward;
    float pid_output;
    
} motor_data_t;

#endif