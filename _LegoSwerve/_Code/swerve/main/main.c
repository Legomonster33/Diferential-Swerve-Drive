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
#include "driver/spi_slave.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

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



#define RCV_HOST    SPI2_HOST

#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5




bool main_isr_flag = false;

motor_data_t motor_1_data = {0};
motor_data_t motor_2_data = {0};

wheel_data_t wheel_data = {0};

//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL

#define DIPSWITCH0 16
#define DIPSWITCH2 17

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_encoder_dev_handle;
i2c_master_dev_handle_t i2c_lcd_dev_handle;

void app_main(void){

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 1,
        .flags = 0,
    };
    
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    char *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, 129, 0);
    char *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, 129, 0);

    spi_slave_transaction_t spi_transaction_data = {0};


    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DIPSWITCH0) | (1ULL << DIPSWITCH2),
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    i2c_master_init(&i2c_bus_handle, &i2c_encoder_dev_handle, &i2c_lcd_dev_handle);
    
    LCD_Init();







    init_gptimer_200hz(&main_isr_flag);

    init_pwm_operator(&motor_1_data, &motor_1_config);
    init_pwm_operator(&motor_2_data, &motor_2_config);

    init_capture_timer(&motor_1_data, &motor_1_config);
    init_capture_timer(&motor_2_data, &motor_2_config); 

    init_pid(&motor_1_data.pid_ctrl, motor_1_config.kp, motor_1_config.ki, motor_1_config.kd, MAX_SPEED / 5, MIN_SPEED / 5, PID_CAL_TYPE_INCREMENTAL);
    init_pid(&motor_2_data.pid_ctrl, motor_2_config.kp, motor_2_config.ki, motor_2_config.kd, MAX_SPEED / 5, MIN_SPEED / 5, PID_CAL_TYPE_INCREMENTAL);

    init_pid(&wheel_data.pid_ctrl, wheel_config.kp, wheel_config.ki, wheel_config.kd, wheel_config.max_rpm, wheel_config.min_rpm, PID_CAL_TYPE_INCREMENTAL);

    motor_1_data.target_rpm = 0;
    motor_2_data.target_rpm = 0;

    motor_1_data.rpm = 0;
    motor_2_data.rpm = 0;

    uint32_t loop_counter_50hz = 0;
    uint32_t loop_counter_1hz = 0;
    uint32_t loop_counter_randtarget = 0;

    wheel_data.drive_reverse = false;
    wheel_data.current_angle = 0; // to be read from the sensor.
    wheel_data.target_angle = 0; // to be set by the orchestrator.
    wheel_data.target_wheel_rpm = 0; // set by orchestrator.


    uint8_t raw_angle_low_byte;
    uint8_t raw_angle_high_byte;
    uint8_t raw_angle_low_byte_address = 0x0D;
    uint8_t raw_angle_high_byte_address = 0x0C;


    int n = 0;

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        

        //if (main_isr_flag){
            loop_counter_50hz ++;
            loop_counter_1hz ++;
            loop_counter_randtarget ++;


            if (loop_counter_randtarget == 600) {
                loop_counter_randtarget = 0;
                wheel_data.target_angle = rand() % 4096;
                wheel_data.target_wheel_rpm = rand() % 3000;
            }

            if (loop_counter_1hz == 200){
                loop_counter_1hz = 0;
                char display_str[21];
                LCD_Clear_Line(1);
                LCD_SetCursor(1, 0);
                sprintf(display_str, "TA:%-4d", wheel_data.target_angle);
                LCD_Print(display_str);
                LCD_SetCursor(1, 10);
                sprintf(display_str, "TR:%4.0f", wheel_data.target_wheel_rpm);
                LCD_Print(display_str);
                LCD_Clear_Line(2);
                LCD_SetCursor(2, 0);
                sprintf(display_str, "Ang:%-4d", wheel_data.current_angle);
                LCD_Print(display_str);
                LCD_SetCursor(2, 10);
                sprintf(display_str, "DrvRev:%d", wheel_data.drive_reverse);
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


                
                
                
                


            i2c_master_transmit_receive(i2c_encoder_dev_handle,&raw_angle_low_byte_address,1,&raw_angle_low_byte,1,I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
            i2c_master_transmit_receive(i2c_encoder_dev_handle,&raw_angle_high_byte_address,1,&raw_angle_high_byte,1,I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
            wheel_data.current_angle = (raw_angle_high_byte << 8) | raw_angle_low_byte;


            


            // shortest-path angle error
            int32_t diff = (int32_t)wheel_data.target_angle - (int32_t)wheel_data.current_angle;

            if (diff > 2048) diff = -4096 + diff;
            else if (diff < -2048) diff = 4096 + diff;

            if (diff > 1024){
                wheel_data.target_angle = (8192 + 1024 - wheel_data.target_angle) % 4096;
                wheel_data.drive_reverse = !wheel_data.drive_reverse;
                diff = 1024 - diff;
            }
            else if (diff < -1024){
                wheel_data.target_angle = (8192 - 1024 - wheel_data.target_angle) % 4096;
                wheel_data.drive_reverse = !wheel_data.drive_reverse;
                diff = -1024 - diff;
            }
            
            wheel_data.angle_error = diff;


            wheel_data.target_motor_rpm = wheel_data.target_wheel_rpm * wheel_config.wheel_rpm_ratio * (wheel_data.drive_reverse ? -1.0f : 1.0f);

            pid_compute(wheel_data.pid_ctrl, wheel_data.angle_error, &wheel_data.motor_rpm_differential);

            
            motor_1_data.target_rpm = wheel_data.target_motor_rpm - wheel_data.motor_rpm_differential;
            motor_2_data.target_rpm = -wheel_data.target_motor_rpm - wheel_data.motor_rpm_differential;
            
                
                
                
            


            update_rpm(&motor_1_data);
            update_rpm(&motor_2_data);
            
            update_pid_feedforward(&motor_1_data, &motor_1_config);
            update_pid_feedforward(&motor_2_data, &motor_2_config);

            motor_1_data.new_speed = motor_1_data.feedforward + motor_1_data.pid_output;
            motor_2_data.new_speed = motor_2_data.feedforward + motor_2_data.pid_output;

            if (gpio_get_level(DIPSWITCH0) == 0) {
                motor_1_data.new_speed = 0;
                motor_2_data.new_speed = 0;
            }


            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_1_data.pwm_comparator, map_speed_to_pulsewidth(motor_1_data.new_speed)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_2_data.pwm_comparator, map_speed_to_pulsewidth(motor_2_data.new_speed)));
            
            
            printf("/*%.0f,%.0f,%.0f,%.0f,", motor_1_data.rpm, motor_1_data.target_rpm, motor_1_data.new_speed,motor_1_data.error);
            printf("%.0f,%.0f,%.0f,%.0f,", motor_2_data.rpm, motor_2_data.target_rpm, motor_2_data.new_speed,motor_2_data.error);
            printf("%.0d,%.0d,%.0f,%.0f*/\n", wheel_data.current_angle, wheel_data.target_angle, wheel_data.angle_error, wheel_data.motor_rpm_differential);
            
            

            memset(recvbuf, 0xA5, 129);
            sprintf(sendbuf, "This is the receiver, sending data for transmission number %04d.", n);
            n++;

            spi_transaction_data.length = 128 * 8;
            spi_transaction_data.tx_buffer = sendbuf;
            spi_transaction_data.rx_buffer = recvbuf;

            spi_slave_transmit(RCV_HOST, &spi_transaction_data, 0);

            printf("Received: %s\n", recvbuf); 
            
            
            
            //main_isr_flag = false;
        
    //}
    }
}
