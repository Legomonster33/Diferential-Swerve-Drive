/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define MIN_PULSEWIDTH 1000  // Minimum pulse width in microsecond
#define DEADBAND_LOWER    1450 // Tested, dont change
#define DEADBAND_UPPER    1540 // Tested, dont change
#define MAX_PULSEWIDTH 2000  // Maximum pulse width in microsecond
#define CENTER_PULSE ((MIN_PULSEWIDTH + MAX_PULSEWIDTH) / 2) // Center pulse width in microsecond

#define MIN_SPEED        -100
#define MAX_SPEED        100

#define MOTOR_1_GPIO             25        // GPIO connects to the PWM signal line

#define HALL_1_GPIO              32        // GPIO connects to the hall sensor output

#define TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        5000    // 5000 ticks, 5ms





static inline uint32_t map_speed_to_pulsewidth(int speed)
{
    uint32_t pulsewidth = CENTER_PULSE;
    
    if (speed == 0) {
        pulsewidth = CENTER_PULSE;
    } 
    
    if (speed < MIN_SPEED || speed > MAX_SPEED) {
        ESP_LOGW(TAG, "Speed should be between %d and %d", MIN_SPEED, MAX_SPEED);
        pulsewidth = CENTER_PULSE;
    }

    if (speed < 0) {
        pulsewidth = (DEADBAND_LOWER+(speed*(DEADBAND_LOWER-MIN_PULSEWIDTH))/-MIN_SPEED);
    } 

    if (speed > 0) {
        pulsewidth = (DEADBAND_UPPER+(speed*(MAX_PULSEWIDTH-DEADBAND_UPPER))/MAX_SPEED);
    }
    
    return pulsewidth;
}



static gptimer_handle_t gptimer = NULL;

typedef struct {
uint32_t hall_timestamps[64];
uint32_t hall_timestamp_index;
uint64_t gptimer_last_timestamp;
} hall_data_t;

static hall_data_t hall_1_data = {0};

//ISR runs when hall sensor, both edges.
static bool hall_trigger_function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    // *************************************************************
    // ISR should be as short as possible. (single line of code)
    // Set a flag that is used in the 'app_main' routine
    // maybe the ISR could increase a count, perhaps


    uint32_t edgetimestamp = edata->cap_value;

    hall_data_t *hall_data = (hall_data_t *)user_data;

    hall_data->hall_timestamps[hall_data->hall_timestamp_index] = edgetimestamp;

    hall_data->hall_timestamp_index = (hall_data->hall_timestamp_index + 1) % 64;

    gptimer_get_raw_count(gptimer, &hall_data->gptimer_last_timestamp);

    ESP_EARLY_LOGI(TAG, "Hall sensor triggered! Timestamp: %u", edgetimestamp);

    return false; // return true to yield at the end of ISR
}






void app_main(void)
{

    ESP_LOGI(TAG, "Create gptimer");
    gptimer_config_t gp_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&gp_timer_config, &gptimer));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));








    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_APB,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HALL_1_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hall_trigger_function,
    };

    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, &hall_1_data));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));









    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = MOTOR_1_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, map_speed_to_pulsewidth(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    
    
    
    



    int speed = 0;
    int step = 1;
    while (1) {
        // ESP_LOGI(TAG, "Speed: %d", speed);

        hall_data_t local_copy = hall_1_data;


        for (int i = 0; i < 64; i++) {
            ESP_LOGI(TAG, "Hall timestamp[%d]: %u us",i,local_copy.hall_timestamps[i]);
        }
        
        ESP_LOGI(TAG, "GPTimer last timestamp: %llu us", local_copy.gptimer_last_timestamp);

        uint64_t current_timestamp;

        gptimer_get_raw_count(gptimer, &current_timestamp);

        ESP_LOGI(TAG, "Current GPTimer timestamp: %llu us", current_timestamp);



        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, map_speed_to_pulsewidth(speed)));

        /*if (speed == 0 || speed == MAX_SPEED || speed == MIN_SPEED || speed == 1 || speed == -1) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        } 
        else */
        {
        vTaskDelay(pdMS_TO_TICKS(5000));}
        
        if (speed >= MAX_SPEED || speed <= MIN_SPEED) {
            step *= -1;
        }
        
        speed += step;
    }
}
