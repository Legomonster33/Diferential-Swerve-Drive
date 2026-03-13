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
#include "motor_config.h"

#include "update_rpm.h"
#include "update_pid_feedforward.h"
#include "calculate_rpm.h"
#include "map_speed_to_pulsewidth.h"
#include "map_target_rpm_to_speed.h"

#include "init_pid.h"
#include "init_gptimer.h"
#include "init_capture_timer.h"
#include "init_pwm_operator.h"

//static const char *TAG = "Swerve:";









//static gptimer_handle_t gptimer_timestamping = NULL;

bool main_isr_flag = false;

motor_data_t motor_1_data = {0};
motor_data_t motor_2_data = {0};

//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL



//200hz ISR, trigger by GPtimer.

/*
static bool IRAM_ATTR timer_isr(gptimer_handle_t timer,const gptimer_alarm_event_data_t *edata,void *user_ctx)
{
    main_isr_flag = true;                   // set flag for main loop
    return false;                            // no context switch needed
}
*/

//ISR runs when hall sensor.
// static bool IRAM_ATTR hall_trigger_function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
// {
//     hall_data_t *hall_data = (hall_data_t *)user_data;

//     hall_data->total_trigger_count++;

//     hall_data->hall_timestamps_index = (hall_data->hall_timestamps_index + 1) % HALL_BUFFER_SIZE;

//     hall_data->hall_timestamps[hall_data->hall_timestamps_index] = edata->cap_value;

//     return false;
// }



void app_main(void)
{
    


    init_gptimer_200hz(&main_isr_flag);


    init_pwm_operator(&motor_1_data, &motor_1_config);
    init_pwm_operator(&motor_2_data, &motor_2_config);

    init_capture_timer(&motor_1_data, &motor_1_config);
    init_capture_timer(&motor_2_data, &motor_2_config); 

    init_pid(&motor_1_data, &motor_1_config);
    init_pid(&motor_2_data, &motor_2_config);






    int speed_pause = 0;

    //uint64_t current_gptimer_timestamp = 0;



    motor_1_data.target_rpm = 0;
    motor_2_data.target_rpm = 0;

    motor_1_data.rpm = 0;
    motor_2_data.rpm = 0;

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        

        if (main_isr_flag){
        
            update_rpm(&motor_1_data);
            update_rpm(&motor_2_data);
            
            update_pid_feedforward(&motor_1_data, &motor_1_config);
            update_pid_feedforward(&motor_2_data, &motor_2_config);

            motor_1_data.new_speed = motor_1_data.feedforward + motor_1_data.pid_output;
            motor_2_data.new_speed = motor_2_data.feedforward + motor_2_data.pid_output;

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_1_data.pwm_comparator, map_speed_to_pulsewidth(motor_1_data.new_speed)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_2_data.pwm_comparator, map_speed_to_pulsewidth(motor_2_data.new_speed)));

            

            
            // Adjust speed

            
            if (speed_pause > 0) {
                speed_pause--;
                }else {
                    
                    motor_1_data.target_rpm = rand() % (motor_1_config.max_rpm - motor_1_config.min_rpm) + motor_1_config.min_rpm; //random target rpm between min_rpm and max_rpm
                    motor_2_data.target_rpm = rand() % (motor_2_config.max_rpm - motor_2_config.min_rpm) + motor_2_config.min_rpm; //random target rpm between min_rpm and max_rpm

                    speed_pause = 1000;
                }
                    
                
            
            

            printf("/*%.0f,%.0f,%.0f,%.0f,", motor_1_data.rpm, motor_1_data.target_rpm, motor_1_data.new_speed,motor_1_data.error);
            printf("%.0f,%.0f,%.0f,%.0f*/\r\n", motor_2_data.rpm, motor_2_data.target_rpm, motor_2_data.new_speed,motor_2_data.error);
            

            
            
            main_isr_flag = false;
        
    }
    }
}
