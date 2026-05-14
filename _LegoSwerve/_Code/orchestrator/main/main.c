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
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/spi_master.h"

#include "hall_data.h"
#include "motor_data.h"
#include "wheel_data.h"


#include "init_pid.h"
#include "init_gptimer.h"

#include "lcd_functions.h"

#include "i2c_master_init.h"




#define SENDER_HOST SPI2_HOST
#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5



bool main_isr_flag = false;


//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL

#define DIPSWITCH0 16
#define DIPSWITCH2 17

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_lcd_dev_handle;


wheel_data_t wheel_0_data = {0};
wheel_data_t wheel_1_data = {0};
wheel_data_t wheel_2_data = {0};
wheel_data_t wheel_3_data = {0};

void app_main(void){

    spi_device_handle_t spi_handle;

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,
        .duty_cycle_pos = 128,      //50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 3,      //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 3
    };

    int n = 0;
    char sendbuf[128] = {0};
    char recvbuf[128] = {0};
    spi_transaction_t spi_transaction_data;
    memset(&spi_transaction_data, 0, sizeof(spi_transaction_data));

    spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    
    spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DIPSWITCH0) | (1ULL << DIPSWITCH2),
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    i2c_master_init(&i2c_bus_handle, &i2c_lcd_dev_handle);

    LCD_Init();


    init_gptimer_200hz(&main_isr_flag);

    //uint32_t loop_counter_50hz = 0;
    //uint32_t loop_counter_1hz = 0;
    //uint32_t loop_counter_randtarget = 0;


    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        if (main_isr_flag){
            
            int res = snprintf(sendbuf, sizeof(sendbuf),"Sender, transmission no. %04i. Last time, I received: \"%s\"", n, recvbuf);
            if (res >= sizeof(sendbuf)) {
                printf("Data truncated\n");
            }
            spi_transaction_data.length = sizeof(sendbuf) * 8;

            spi_transaction_data.tx_buffer = sendbuf;

            spi_transaction_data.rx_buffer = recvbuf;

            spi_device_transmit(spi_handle, &spi_transaction_data);
            printf("Received: %s\n", recvbuf);
            n++;
            
            
            main_isr_flag = false;
        
    }
    }
}
