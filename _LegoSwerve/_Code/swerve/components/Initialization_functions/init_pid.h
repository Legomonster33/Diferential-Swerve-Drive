#ifndef INIT_PID_H  
#define INIT_PID_H

#include "pid_ctrl.h"
#include "motor_data.h"
#include "motor_config.h"

void init_pid(pid_ctrl_block_handle_t *pid_ctrl, float kp, float ki, float kd, float max_output, float min_output);

#endif // init_pid_H
