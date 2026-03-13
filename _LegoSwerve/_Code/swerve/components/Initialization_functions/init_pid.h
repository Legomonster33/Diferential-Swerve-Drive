#ifndef INIT_PID_H
#define INIT_PID_H

#include "pid_ctrl.h"
#include "motor_config.h"

void init_pid(pid_ctrl_block_handle_t *pid_handle, const MotorConfig *motor_config);

#endif // INIT_PID_H
