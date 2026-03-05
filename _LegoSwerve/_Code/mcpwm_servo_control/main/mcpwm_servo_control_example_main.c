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

#define MIN_SPEED        -1000
#define MAX_SPEED        1000

#define MOTOR_1_GPIO             25        // GPIO connects to the PWM signal line

#define HALL_1_GPIO              32        // GPIO connects to the hall sensor output

#define TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        5000    // 5000 ticks, 5ms


#define PULSES_PER_REV 6
#define TIMER_FREQ_HZ 80000000ULL   // 80 MHz timer (microseconds)

#define MOTOR_STALL_TICKS 200000




bool main_isr_flag = false;

static gptimer_handle_t gptimer_200_hz = NULL;

static gptimer_handle_t gptimer_timestamping = NULL;

typedef struct {
    uint32_t hall_timestamps[64];
    uint32_t hall_timestamps_index;
    uint32_t hall_timestamps_last_index;
    uint32_t total_trigger_count;
    uint32_t last_total_trigger_count;
    uint32_t difference_total_triggers_last_update;
    uint64_t gptimer_last_isr_timestamp;
    uint64_t gptimer_last_update_timestamp;
    enum {
        MODE_SINGLE_PERIOD,
        MODE_MULTIPLE_PERIOD,
    }mode;
} hall_data_t;

static hall_data_t hall_1_data = {0};



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









uint32_t calculate_rpm(hall_data_t *hall_data)
{
    uint32_t rpm = 0;


    uint64_t dt_since_last_pulse = hall_data->gptimer_last_update_timestamp - hall_data->gptimer_last_isr_timestamp;

    /*
    ESP_LOGI(TAG, "gptimer_last_update_timestamp: %llu ticks", hall_data->gptimer_last_update_timestamp);
    ESP_LOGI(TAG, "gptimer_last_isr_timestamp: %llu ticks", hall_data->gptimer_last_isr_timestamp);
    ESP_LOGI(TAG, "deltaTime since last update vs since last pulse: %llu ticks", dt_since_last_pulse);
    */

    if (dt_since_last_pulse > MOTOR_STALL_TICKS) {
        rpm = 0;
        return rpm;
    }

    //ESP_LOGI(TAG,"nonzero rpm");
    
    switch (hall_data->mode){

        case MODE_SINGLE_PERIOD:
        {
            uint32_t current_index  = hall_data->hall_timestamps_index;
            
            uint32_t newer_index = (current_index + 63) % 64; // index of last written timestamp
            
            uint32_t older_index = (current_index + 62) % 64; // index of the timestamp before last written

            uint32_t newer = hall_data->hall_timestamps[newer_index];
            uint32_t older = hall_data->hall_timestamps[older_index];

            uint32_t dt;

            dt = newer - older;
                
            if (newer != older){
                rpm =((60ULL * TIMER_FREQ_HZ)/(dt * PULSES_PER_REV));
                }
        break;
        
        }

        case MODE_MULTIPLE_PERIOD:
    {
            uint32_t current_index  = hall_data->hall_timestamps_index;

                // calculate number of new pulses since last update

            uint32_t new_count = hall_data->difference_total_triggers_last_update;

            //P_LOGI(TAG, "New pulse count since last update: %u", new_count); // always 0 for some reason, must redo math

                
                uint64_t total_dt = 0;
                uint32_t valid_periods = 0;

                // loop over new periods
                for (uint32_t i = 1; i < new_count; i++)
                {
                    uint32_t newer = hall_data->hall_timestamps[(current_index - i + 64) % 64];
                    uint32_t older = hall_data->hall_timestamps[(current_index - i - 1 + 64) % 64];

                    uint32_t dt = newer - older; // unsigned subtraction handles wrap automatically

                    if (dt > 0)
                    {
                        total_dt += dt;
                        valid_periods++;
                    }
                }

                if (valid_periods > 0)
                {
                    uint64_t avg_dt = total_dt / valid_periods;

                    rpm = (uint32_t)(
                        (60ULL * TIMER_FREQ_HZ) /
                        (avg_dt * PULSES_PER_REV)
                    );
                }

    break;

        }
    }

        return rpm;
}




//200hz ISR, trigger by GPtimer.
static bool IRAM_ATTR timer_isr(gptimer_handle_t timer,const gptimer_alarm_event_data_t *edata,void *user_ctx)
{
    main_isr_flag = true;                   // set flag for main loop
    return false;                            // no context switch needed
}



//ISR runs when hall sensor.
static bool hall_trigger_function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{

    uint32_t edgetimestamp = edata->cap_value;
    uint64_t current_gptimer_timestamp;

    hall_data_t *hall_data = (hall_data_t *)user_data;


    hall_data->total_trigger_count++;


    hall_data->hall_timestamps[hall_data->hall_timestamps_index] = edgetimestamp;

    hall_data->hall_timestamps_index = (hall_data->hall_timestamps_index + 1) % 64;


    gptimer_get_raw_count(gptimer_timestamping, &current_gptimer_timestamp);

    hall_data->gptimer_last_isr_timestamp = current_gptimer_timestamp;


    return false;
            

}



void app_main(void)
{

    ESP_LOGI(TAG, "Create gptimer for 200hz");
    gptimer_config_t gp_timer_config_200hz = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&gp_timer_config_200hz, &gptimer_200_hz));

    
    gptimer_alarm_config_t alarm_config_200_hz = {
    .alarm_count = 5000,                    // 200 Hz → 5000 ticks at 1 MHz
    .reload_count = 0,                   // must be >0 if auto_reload is enabled
    .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_200_hz, &alarm_config_200_hz));


    ESP_LOGI(TAG, "Register ISR");
    gptimer_event_callbacks_t gpt_200_hz_cbs = {
    .on_alarm = timer_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_200_hz, &gpt_200_hz_cbs, NULL));  // user_ctx optional

    
    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer_200_hz));
    ESP_ERROR_CHECK(gptimer_start(gptimer_200_hz));


    

    ESP_LOGI(TAG, "Create gptimer for timestamping");
    gptimer_config_t gp_timer_config_timestamping = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&gp_timer_config_timestamping, &gptimer_timestamping));


    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer_timestamping));
    ESP_ERROR_CHECK(gptimer_start(gptimer_timestamping));




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
        .flags.neg_edge = false,
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

    ESP_LOGI(TAG, "Create MOTOR_1_PWM_DUTY and generator from the operator");
    mcpwm_cmpr_handle_t MOTOR_1_PWM_DUTY = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &MOTOR_1_PWM_DUTY));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = MOTOR_1_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(MOTOR_1_PWM_DUTY, map_speed_to_pulsewidth(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MOTOR_1_PWM_DUTY, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    
    
    
    



    int speed = 0; //put to 6 for slowest
    int step = 1;

    uint32_t rpm = 0;

    int loopcount = 0;

    int speed_pause = 0;

    uint64_t current_gptimer_timestamp = 0;

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise wdtd triggers
        if (main_isr_flag){
        

        gptimer_get_raw_count(gptimer_timestamping, &current_gptimer_timestamp);

        hall_1_data.gptimer_last_update_timestamp = current_gptimer_timestamp;

        hall_1_data.difference_total_triggers_last_update = hall_1_data.total_trigger_count - hall_1_data.last_total_trigger_count;
        
        hall_1_data.last_total_trigger_count = hall_1_data.total_trigger_count;

        hall_1_data.hall_timestamps_last_index = hall_1_data.hall_timestamps_index;
        




        


        
            
        // at high rpms, its only counting 2 or 3 pulses because its going into fixed interval mode, but the interval is way too short, needs fixing.

        uint32_t pulses_this_loop = hall_1_data.difference_total_triggers_last_update;



        if (pulses_this_loop < 2) {
        hall_1_data.mode = MODE_SINGLE_PERIOD;       // very low RPM
        }
        else {
        hall_1_data.mode = MODE_MULTIPLE_PERIOD;     // medium RPM
        }


        hall_data_t local_copy = hall_1_data;


        /*
        else {
        hall_1_data.mode = MODE_FIXED_INTERVAL;      // high RPM
        }
        */




        rpm = calculate_rpm(&local_copy);


        



        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(MOTOR_1_PWM_DUTY, map_speed_to_pulsewidth(speed)));

        
        
        // Adjust speed
            if (speed_pause > 0) {
                speed_pause--;
            } 


            else {
                speed += step;

                if (speed >= MAX_SPEED/2 || speed <= MIN_SPEED/2) {
                    step *= -1;
                    speed = (speed >= MAX_SPEED/2) ? MAX_SPEED/2 : MIN_SPEED/2;
                    speed_pause = 200;
                }
                
                else if (speed == 0 || speed == 1 || speed == -1 || speed % 100 == 0) {
                    speed_pause = 100;
                }
            }


        if (loopcount % 50== 0) { 

                    
            

            /*
            for (int i = 0; i < 64; i++) {
                ESP_LOGI(TAG, "Hall timestamp[%d]: %u ticks",i,local_copy.hall_timestamps[i]);
                }
            */      

            //ESP_LOGI(TAG, "Total hall triggers: %u", local_copy.total_trigger_count);
           
            
            ESP_LOGI(TAG, "triggers since last update: %u", pulses_this_loop);

            ESP_LOGI(TAG, "Current mode: %d", hall_1_data.mode);

            ESP_LOGI(TAG, "Calculated RPM: %u", rpm);

            ESP_LOGI(TAG, "Speed: %d", speed);

            
           
            ESP_LOGI(TAG, "Last isr timestamp: %llu ticks", local_copy.gptimer_last_isr_timestamp);

            ESP_LOGI(TAG, "Last update timestamp: %llu ticks", local_copy.gptimer_last_update_timestamp);


        }

        loopcount++;
        
        main_isr_flag = false;
    
    }
    }
}
