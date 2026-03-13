#ifndef INIT_PID_H
#define INIT_PID_H

#include "pid_ctrl.h"
#include "motor_data.h"
#include "motor_config.h"

void init_pid(motor_data_t *motor_data, const Motor_Config_t *motor_config);

#endif // INIT_PID_H
