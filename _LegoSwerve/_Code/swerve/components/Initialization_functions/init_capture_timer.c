#include "init_capture_timer.h"
#include "esp_log.h"
#include "driver/mcpwm_cap.h"
#include "hall_data.h" // Ensure hall_data_t is defined

static const char *TAG = "InitCaptureTimer";

static bool IRAM_ATTR hall_trigger_function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    hall_data_t *hall_data = (hall_data_t *)user_data;

    hall_data->total_trigger_count++;

    hall_data->hall_timestamps_index = (hall_data->hall_timestamps_index + 1) % HALL_BUFFER_SIZE;

    hall_data->hall_timestamps[hall_data->hall_timestamps_index] = edata->cap_value;

    return false;
}

void init_capture_timer(motor_data_t *motor_data, Motor_Config_t *motor_config)
{
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_APB,
        .group_id = motor_config->capture_group_id, // Use group_id from motor_config
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = motor_config->hall_pin,
        .prescale = 1,
        .flags.neg_edge = false,
        .flags.pos_edge = true,
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hall_trigger_function,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, &motor_data->hall_data));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
}

