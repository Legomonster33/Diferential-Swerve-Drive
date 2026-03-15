/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "pid_ctrl.h"
#include <math.h>
#include "esp_system.h"

#include "hall_data.h"
#include "motor_data.h"
#include "motor_config.h"

#include "wheel_config.h"
#include "wheel_data.h"

#include "update_rpm.h"
#include "update_pid_feedforward.h"
#include "map_speed_to_pulsewidth.h"

#include "init_pid.h"
#include "init_gptimer.h"
#include "init_capture_timer.h"
#include "init_pwm_operator.h"

#define WHEEL_RPM_RATIO 1.0f 

bool main_isr_flag = false;

motor_data_t motor_1_data = {0};
motor_data_t motor_2_data = {0};

wheel_data_t wheel_data = {0};

//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL


static inline int wrap_to_2048(int x)
{
    while (x > 2048)  x -= 4096;
    while (x < -2048) x += 4096;
    return x;
}


void app_main(void)
{
    
    init_gptimer_200hz(&main_isr_flag);

    init_pwm_operator(&motor_1_data, &motor_1_config);
    init_pwm_operator(&motor_2_data, &motor_2_config);

    init_capture_timer(&motor_1_data, &motor_1_config);
    init_capture_timer(&motor_2_data, &motor_2_config); 

    init_pid(&motor_1_data.pid_ctrl, motor_1_config.kp, motor_1_config.ki, motor_1_config.kd, MAX_SPEED / 5, MIN_SPEED / 5);
    init_pid(&motor_2_data.pid_ctrl, motor_2_config.kp, motor_2_config.ki, motor_2_config.kd, MAX_SPEED / 5, MIN_SPEED / 5);

    init_pid(&wheel_data.pid_ctrl, wheel_config.kp, wheel_config.ki, wheel_config.kd, wheel_config.max_rpm, wheel_config.min_rpm);

    motor_1_data.target_rpm = 0;
    motor_2_data.target_rpm = 0;

    motor_1_data.rpm = 0;
    motor_2_data.rpm = 0;

    uint32_t loop_counter = 0;

    wheel_data.current_angle = 0; // to be read from the sensor.
    wheel_data.target_angle = 0; // to be set by the orchestrator.
    wheel_data.target_wheel_rpm = 3000; // set by orchestrator.

    

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        

        if (main_isr_flag){
            loop_counter ++;

            if (loop_counter == 10){ //runs at 20hz
                loop_counter = 0;

                //read_sensor_data(&wheel_data.current_angle);
                if (wheel_data.current_angle < 360) { // fake sensor data, it just loops.
                    wheel_data.current_angle++;
                } else {
                    wheel_data.current_angle = 0;
                }


                wheel_data.target_motor_rpm = wheel_data.target_wheel_rpm * wheel_config.wheel_rpm_ratio;

                /* compute shortest angular error */
                int delta = wrap_to_2048(wheel_data.target_angle - wheel_data.current_angle);

                /* swerve optimization: flip wheel if >90° */
                if (abs(delta) > 1024) {

                    wheel_data.target_wheel_rpm = -wheel_data.target_wheel_rpm;
                    wheel_data.target_motor_rpm = -wheel_data.target_motor_rpm;

                    wheel_data.target_angle = (wheel_data.target_angle + 2048) & 0xFFF; // wrap 0-4095

                    delta = wrap_to_2048(wheel_data.target_angle - wheel_data.current_angle);
                }

                /* PID uses shortest path error */
                wheel_data.angle_error = delta;

                pid_compute(wheel_data.pid_ctrl, wheel_data.angle_error, &wheel_data.motor_rpm_differential);

                motor_1_data.target_rpm = wheel_data.target_motor_rpm + wheel_data.motor_rpm_differential;
                motor_2_data.target_rpm = wheel_data.target_motor_rpm - wheel_data.motor_rpm_differential;

            }


            /*
            if (speed_pause > 0) {
                speed_pause--;
                }else {
                    
                    wheel_data.target_wheel_rpm += 500;
                    if (wheel_data.target_wheel_rpm > 3000) {
                        wheel_data.target_wheel_rpm = 0;
                    }
                    speed_pause = 1000;
                }
            */





            update_rpm(&motor_1_data);
            update_rpm(&motor_2_data);
            
            update_pid_feedforward(&motor_1_data, &motor_1_config);
            update_pid_feedforward(&motor_2_data, &motor_2_config);

            motor_1_data.new_speed = motor_1_data.feedforward + motor_1_data.pid_output;
            motor_2_data.new_speed = motor_2_data.feedforward + motor_2_data.pid_output;

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_1_data.pwm_comparator, map_speed_to_pulsewidth(motor_1_data.new_speed)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_2_data.pwm_comparator, map_speed_to_pulsewidth(motor_2_data.new_speed)));

            printf("/*%.0f,%.0f,%.0f,%.0f,", motor_1_data.rpm, motor_1_data.target_rpm, motor_1_data.new_speed,motor_1_data.error);
            printf("%.0f,%.0f,%.0f,%.0f,", motor_2_data.rpm, motor_2_data.target_rpm, motor_2_data.new_speed,motor_2_data.error);
            printf("%.0f,%.0f,%.0f,%.0f*/\n", wheel_data.current_angle, wheel_data.target_angle, wheel_data.angle_error, wheel_data.motor_rpm_differential);

            
            
            main_isr_flag = false;
        
    }
    }
}
