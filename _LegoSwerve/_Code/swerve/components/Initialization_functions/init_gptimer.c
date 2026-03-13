#include "esp_log.h"
#include "driver/gptimer.h"

static const char *TAG = "GPTIMER_INIT";

static bool IRAM_ATTR timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    bool *isr_flag = (bool *)user_ctx;
    *isr_flag = true; // Set flag for main loop
    return false;     // No context switch needed
}

void init_gptimer_200hz(bool *isr_flag) {
    gptimer_handle_t gptimer_200_hz = NULL;

    ESP_LOGI(TAG, "Create gptimer for 200Hz");
    gptimer_config_t gp_timer_config_200hz = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&gp_timer_config_200hz, &gptimer_200_hz));

    gptimer_alarm_config_t alarm_config_200_hz = {
        .alarm_count = 5000,                  // 200 Hz → 5000 ticks at 1 MHz
        .reload_count = 0,                   // Must be >0 if auto_reload is enabled
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_200_hz, &alarm_config_200_hz));

    ESP_LOGI(TAG, "Register ISR for 200Hz");
    gptimer_event_callbacks_t gpt_200_hz_cbs = {
        .on_alarm = timer_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_200_hz, &gpt_200_hz_cbs, isr_flag));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer_200_hz));
    ESP_ERROR_CHECK(gptimer_start(gptimer_200_hz));
}