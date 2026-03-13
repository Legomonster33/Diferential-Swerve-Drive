#include "pid_ctrl.h"
#include "motor_data.h"
#include "init_pid.h"
#include "map_speed_to_pulsewidth.h"
#include "esp_log.h"

static const char *TAG = "PID_INIT";

void init_pid(motor_data_t *motor_data, const Motor_Config_t *motor_config) {
    ESP_LOGI(TAG, "Create PID control block");

    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = motor_config->kp,
        .ki = motor_config->ki,
        .kd = motor_config->kd,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = MAX_SPEED / 5,
        .min_output   = MIN_SPEED / 5,
        .max_integral = 100000,
        .min_integral = -100000,
    };

    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &motor_data->pid_ctrl));
}