#ifndef INIT_PID_H  
#define INIT_PID_H

#include "pid_ctrl.h"
#include "motor_data.h"

void init_pid(pid_ctrl_block_handle_t *pid_ctrl, float kp, float ki, float kd, float max_output, float min_output, pid_calculate_type_t cal_type);

#endif // init_pid_H
