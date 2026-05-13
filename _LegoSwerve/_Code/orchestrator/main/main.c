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
#include "wheel_data.h"


#include "init_pid.h"
#include "init_gptimer.h"

#include "lcd_functions.h"

#include "i2c_master_init.h"




bool main_isr_flag = false;


//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL

#define DIPSWITCH0 16
#define DIPSWITCH2 17

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t corner_0_dev_handle;
i2c_master_dev_handle_t corner_1_dev_handle;
i2c_master_dev_handle_t corner_2_dev_handle;
i2c_master_dev_handle_t corner_3_dev_handle;
i2c_master_dev_handle_t i2c_lcd_dev_handle;


wheel_data_t wheel_0_data = {0};
wheel_data_t wheel_1_data = {0};
wheel_data_t wheel_2_data = {0};
wheel_data_t wheel_3_data = {0};

void app_main(void){

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DIPSWITCH0) | (1ULL << DIPSWITCH2),
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    i2c_master_init(&i2c_bus_handle, &corner_0_dev_handle, &i2c_lcd_dev_handle);

    LCD_Init();


    init_gptimer_200hz(&main_isr_flag);

    uint32_t loop_counter_50hz = 0;
    uint32_t loop_counter_1hz = 0;
    uint32_t loop_counter_randtarget = 0;

    uint32_t i2cdatalen = sizeof(wheel_data_t);

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        

        if (main_isr_flag){
                
            i2c_master_receive(corner_0_dev_handle, (uint8_t *)&wheel_0_data, i2cdatalen, 1000 / portTICK_PERIOD_MS);

            //i2c_master_transmit(corner_0_dev_handle, (uint8_t *)&wheel_0_data, i2cdatalen, 1000 / portTICK_PERIOD_MS);

            //should add current rpm to wheel_data_t
            printf("Target RPM: %f, Angle: %d, Target Angle: %d\n", wheel_0_data.target_motor_rpm, wheel_0_data.current_angle, wheel_0_data.target_angle);

            main_isr_flag = false;
        
    }
    }
}
