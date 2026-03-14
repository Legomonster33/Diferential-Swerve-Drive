#include "pid_ctrl.h"
#include "init_pid.h"
#include "map_speed_to_pulsewidth.h"
#include "esp_log.h"

static const char *TAG = "PID_INIT";

void init_pid(pid_ctrl_block_handle_t *pid_ctrl, float kp, float ki, float kd, float max_output, float min_output) {
    ESP_LOGI(TAG, "Create PID control block");

    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output = max_output,
        .min_output = min_output,
        .max_integral = 100000,
        .min_integral = -100000,
    };

    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, pid_ctrl));
}