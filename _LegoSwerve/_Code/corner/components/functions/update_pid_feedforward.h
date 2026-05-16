#ifndef UPDATE_PID_FEEDFORWARD_H
#define UPDATE_PID_FEEDFORWARD_H

#include "motor_data.h"
#include "Motor_config.h"
#include "pid_ctrl.h"
#include "map_target_rpm_to_speed.h"

void update_pid_feedforward(motor_data_t *motor_data, Motor_Config_t *motor_config, int32_t rpm_speed_lookuptable[LUT_SIZE]);

#endif // UPDATE_PID_FEEDFORWARD_H