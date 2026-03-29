/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "pid_ctrl.h"
#include <math.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "pid_ctrl.h"

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

#include "lcd_functions.h"

#include "i2c_master_init.h"



#define WHEEL_RPM_RATIO 1.0f 

bool main_isr_flag = false;

motor_data_t motor_1_data = {0};
motor_data_t motor_2_data = {0};

wheel_data_t wheel_data = {0};

//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL

#define BUTTON_1_GPIO_NUM 16
#define BUTTON_2_GPIO_NUM 17

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_encoder_dev_handle;
i2c_master_dev_handle_t i2c_lcd_dev_handle;


void app_main(void){

    i2c_master_init(&i2c_bus_handle, &i2c_encoder_dev_handle, &i2c_lcd_dev_handle);
    // Initialize the LCD
    LCD_Init();




    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_1_GPIO_NUM) | (1ULL << BUTTON_2_GPIO_NUM),
        .pull_down_en = 1,
        .pull_up_en = 0
    };

    gpio_config(&io_conf);


    init_gptimer_200hz(&main_isr_flag);

    init_pwm_operator(&motor_1_data, &motor_1_config);
    init_pwm_operator(&motor_2_data, &motor_2_config);

    init_capture_timer(&motor_1_data, &motor_1_config);
    init_capture_timer(&motor_2_data, &motor_2_config); 

    init_pid(&motor_1_data.pid_ctrl, motor_1_config.kp, motor_1_config.ki, motor_1_config.kd, MAX_SPEED / 5, MIN_SPEED / 5, PID_CAL_TYPE_INCREMENTAL);
    init_pid(&motor_2_data.pid_ctrl, motor_2_config.kp, motor_2_config.ki, motor_2_config.kd, MAX_SPEED / 5, MIN_SPEED / 5, PID_CAL_TYPE_INCREMENTAL);

    init_pid(&wheel_data.pid_ctrl, wheel_config.kp, wheel_config.ki, wheel_config.kd, wheel_config.max_rpm, wheel_config.min_rpm, PID_CAL_TYPE_POSITIONAL);

    motor_1_data.target_rpm = 0;
    motor_2_data.target_rpm = 0;

    motor_1_data.rpm = 0;
    motor_2_data.rpm = 0;

    uint32_t loop_counter_20hz = 0;
    uint32_t loop_counter_1hz = 0;

    wheel_data.current_angle = 0; // to be read from the sensor.
    wheel_data.target_angle = 0; // to be set by the orchestrator.
    wheel_data.target_wheel_rpm = 3000; // set by orchestrator.


    uint8_t raw_angle_low_byte;
    uint8_t raw_angle_high_byte;
    uint8_t raw_angle_low_byte_address = 0x0D;
    uint8_t raw_angle_high_byte_address = 0x0C;



    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        

        if (main_isr_flag){
            loop_counter_20hz ++;
            loop_counter_1hz ++;

            
            if (gpio_get_level(BUTTON_1_GPIO_NUM) == 1) {
                    wheel_data.target_angle = (wheel_data.target_angle + 4096 + 1) % 4096;
                }
                if (gpio_get_level(BUTTON_2_GPIO_NUM) == 1) {
                    wheel_data.target_angle = (wheel_data.target_angle + 4096 - 1) % 4096;
                }
            

            if (loop_counter_1hz == 100){
                loop_counter_1hz = 0;
                char display_str[21];
                LCD_Clear_Line(1);
                LCD_SetCursor(1, 0);
                sprintf(display_str, "Tgt Ang:%-4d", wheel_data.target_angle);
                LCD_Print(display_str);
                LCD_Clear_Line(2);
                LCD_SetCursor(2, 0);
                sprintf(display_str, "Ang:%-4d", wheel_data.current_angle);
                LCD_Print(display_str);
                LCD_Clear_Line(3);
                LCD_SetCursor(3, 0);
                sprintf(display_str, "M1:%5.0f M2:%5.0f", motor_1_data.rpm, motor_2_data.rpm);
                LCD_Print(display_str);
                LCD_Clear_Line(4);
                LCD_SetCursor(4, 0);
                sprintf(display_str, "E:%6.0f D:%6.0f", wheel_data.angle_error, wheel_data.motor_rpm_differential);
                LCD_Print(display_str);
            }

            if (loop_counter_20hz == 10){ //runs at 20hz
                loop_counter_20hz = 0;


                
                
                
                


                i2c_master_transmit_receive(i2c_encoder_dev_handle,&raw_angle_low_byte_address,1,&raw_angle_low_byte,1,I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
                i2c_master_transmit_receive(i2c_encoder_dev_handle,&raw_angle_high_byte_address,1,&raw_angle_high_byte,1,I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
                wheel_data.current_angle = (raw_angle_high_byte << 8) | raw_angle_low_byte;


                wheel_data.target_motor_rpm = wheel_data.target_wheel_rpm * wheel_config.wheel_rpm_ratio;


                // shortest-path angle error
                int32_t diff = (int32_t)wheel_data.target_angle - (int32_t)wheel_data.current_angle;
                if (diff > 2047) diff -= 4096;
                else if (diff < -2048) diff += 4096;
                wheel_data.angle_error = diff;


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
            
            
            //printf("/*%.0f,%.0f,%.0f,%.0f,", motor_1_data.rpm, motor_1_data.target_rpm, motor_1_data.new_speed,motor_1_data.error);
            //printf("%.0f,%.0f,%.0f,%.0f,", motor_2_data.rpm, motor_2_data.target_rpm, motor_2_data.new_speed,motor_2_data.error);
            //printf("%.0d,%.0d,%.0f,%.0f*/\n", wheel_data.current_angle, wheel_data.target_angle, wheel_data.angle_error, wheel_data.motor_rpm_differential);
            
            
            
            
            main_isr_flag = false;
        
    }
    }
}
