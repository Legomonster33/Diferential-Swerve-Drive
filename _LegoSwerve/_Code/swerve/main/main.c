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
#include "pid_ctrl.h"
#include "sdkconfig.h"
#include <math.h>
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp32/rom/ets_sys.h"


#include "hall_data.h"
#include "motor_data.h"

#include "calculate_rpm.h"
#include "map_speed_to_pulsewidth.h"
#include "map_target_rpm_to_speed.h"

static const char *TAG = "Swerve:";



#define MOTOR_1_GPIO             25        // GPIO connects to the PWM signal line

#define HALL_1_GPIO              32        // GPIO connects to the hall sensor output

#define TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        5000    // 5000 ticks, 5ms





bool main_isr_flag = false;

static gptimer_handle_t gptimer_200_hz = NULL;

static gptimer_handle_t gptimer_timestamping = NULL;


static hall_data_t hall_1_data = {0};

static motor_data_t motor_1_data = {0};

static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED;




//200hz ISR, trigger by GPtimer.
static bool IRAM_ATTR timer_isr(gptimer_handle_t timer,const gptimer_alarm_event_data_t *edata,void *user_ctx)
{
    main_isr_flag = true;                   // set flag for main loop
    return false;                            // no context switch needed
}



//ISR runs when hall sensor.
static bool IRAM_ATTR hall_trigger_function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    uint32_t edgetimestamp = edata->cap_value;
    hall_data_t *hall_data = (hall_data_t *)user_data;

    hall_data->total_trigger_count++;

    hall_data->hall_timestamps_index = (hall_data->hall_timestamps_index + 1) % HALL_BUFFER_SIZE;

    hall_data->hall_timestamps[hall_data->hall_timestamps_index] = edgetimestamp;

    return false;
            

}



void app_main(void)
{
    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t Motor_1_pid_runtime_param = {
        .kp = 0.2,
        .ki = 0.005,
        .kd = 0.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = MAX_SPEED/5,
        .min_output   = MIN_SPEED/5,
        .max_integral = 100000,
        .min_integral = -100000,
    };
    pid_ctrl_block_handle_t Motor_1_pid_ctrl = NULL;
    pid_ctrl_config_t Motor_1_pid_config = {
        .init_param = Motor_1_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&Motor_1_pid_config, &Motor_1_pid_ctrl));








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
    
    
    
    




    int loopcount = 0;

    int speed_pause = 0;

    //uint64_t current_gptimer_timestamp = 0;

    float step = 5; // RPM step for testing

    motor_1_data.target_rpm = 0;
    motor_1_data.rpm = 0;

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise wdtd triggers

        
        
        


        if (main_isr_flag){
        

        taskENTER_CRITICAL(&my_mux);
        hall_data_t Hall_1_local_copy = hall_1_data;
        taskEXIT_CRITICAL(&my_mux);



        hall_1_data.last_total_trigger_count = Hall_1_local_copy.total_trigger_count;
        hall_1_data.hall_timestamps_last_index = Hall_1_local_copy.hall_timestamps_index;

        if (Hall_1_local_copy.total_trigger_count == Hall_1_local_copy.last_total_trigger_count) {
            hall_1_data.ticks_since_last_trigger += 1;
        } else {
            hall_1_data.ticks_since_last_trigger = 0;
        }


        float smoothing_factor = 0.9f - (motor_1_data.target_rpm * 0.85f / 2000.0f);

        if (smoothing_factor < 0.1f) smoothing_factor = 0.1f;
        if (smoothing_factor > 0.9f) smoothing_factor = 0.9f;

        float measured_rpm = calculate_rpm(hall_1_data, &motor_1_data);


        motor_1_data.rpm = (1.0f - smoothing_factor) * motor_1_data.rpm + smoothing_factor * measured_rpm;

        
        
        if (motor_1_data.new_speed < 0) {   // negate rpm if speed output is negative.
            motor_1_data.rpm = -motor_1_data.rpm;
        }
        




        motor_1_data.error = (motor_1_data.target_rpm - motor_1_data.rpm);

        motor_1_data.feedforward = map_target_rpm_to_speed(motor_1_data.target_rpm);

        pid_compute(Motor_1_pid_ctrl, motor_1_data.error, &motor_1_data.pid_output);

        


        motor_1_data.new_speed = motor_1_data.feedforward + motor_1_data.pid_output;

        //motor_1_data.new_speed = 500; // for testing



        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(MOTOR_1_PWM_DUTY, map_speed_to_pulsewidth(motor_1_data.new_speed)));

        

        
        // Adjust speed

        
        if (speed_pause > 0) {
                speed_pause--;
        } 


        else {
            motor_1_data.target_rpm += step;

            if (motor_1_data.target_rpm >= MAX_RPM || motor_1_data.target_rpm <= MIN_RPM) {
                    step *= -1;
                    motor_1_data.target_rpm = (motor_1_data.target_rpm >= MAX_RPM) ? MAX_RPM : MIN_RPM;
                    speed_pause = 400;
                }
            else if (motor_1_data.target_rpm == 0) {
                    speed_pause = 400;
                }
            }
        
        
        
        
        printf("/*%.1f, %.1f, %.1f, %.1f, %.1f, %.1f*/\r\n", motor_1_data.rpm, motor_1_data.target_rpm, motor_1_data.new_speed, motor_1_data.pid_output,motor_1_data.feedforward,motor_1_data.error);

        
        if (loopcount % 200== 0) { 

            
            //for (int i = 0; i < 64; i++) {
            //    ESP_LOGI(TAG, "Hall timestamp[%d]: %u ticks",i,Hall_1_local_copy.hall_timestamps[i]);}
                  
            //ESP_LOGI(TAG, "Total hall triggers: %u", Hall_1_local_copy.total_trigger_count);
            //ESP_LOGI(TAG, "Current mode: %d", hall_1_data.mode);
            
            //ESP_LOGI(TAG, "Speed: %f", motor_1_data.new_speed);
            //ESP_LOGI(TAG, "Error: %f", motor_1_data.error);

            //ESP_LOGI(TAG, "Calculated RPM: %f", motor_1_data.rpm);
            //ESP_LOGI(TAG, "Target RPM: %f", motor_1_data.target_rpm);

            //ESP_LOGI(TAG, "Last isr timestamp: %llu ticks", Hall_1_local_copy.gptimer_last_isr_timestamp);
            //ESP_LOGI(TAG, "Last update timestamp: %llu ticks", Hall_1_local_copy.gptimer_last_update_timestamp);
        }
        

        loopcount++;
        
        main_isr_flag = false;
    
    }
    }
}
