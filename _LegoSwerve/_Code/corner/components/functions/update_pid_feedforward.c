#include "update_pid_feedforward.h"


#include "pid_ctrl.h"

void update_pid_feedforward(motor_data_t *motor_data, Motor_Config_t *motor_config, int32_t rpm_speed_lookuptable[LUT_SIZE]) {
    motor_data->error = (motor_data->target_rpm - motor_data->rpm);

    motor_data->feedforward = map_target_rpm_to_speed(motor_data->target_rpm, rpm_speed_lookuptable);

    pid_compute(motor_data->pid_ctrl, motor_data->error, &motor_data->pid_output);
}