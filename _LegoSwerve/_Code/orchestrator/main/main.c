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
#include <math.h>
#include "driver/spi_master.h"
#include "driver/rmt_rx.h"

#include "hall_data.h"
#include "motor_data.h"
#include "wheel_data.h"
#include "module_data.h"
#include "drivebase_data.h"


#include "init_pid.h"
#include "init_gptimer.h"

#include "lcd_functions.h"

#include "i2c_master_init.h"

#include "spi_transaction_data_t.h"



#define ENCODER_RESOLUTION 4096.0f
#define TWO_PI 6.28318530718f

#define SENDER_HOST SPI3_HOST
#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5



bool main_isr_flag = false;


//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL

#define DIPSWITCH0 16
#define DIPSWITCH2 17






#define RC_INPUT_PIN 36

#define RC_CH_COUNT 4

#define RC_CH_THROTTLE 0
#define RC_CH_STEERING  1
#define RC_CH_ROTATION  2
#define RC_CH_MODE      3


static const int rc_pins[RC_CH_COUNT] = {36, 39, 34, 35};

rmt_channel_handle_t rc_rx[RC_CH_COUNT];

rmt_symbol_word_t rc_raw[RC_CH_COUNT][64];

volatile uint32_t rc_pulse_us[RC_CH_COUNT] = {1500,1500,1500,1500};
volatile bool rc_new[RC_CH_COUNT] = {0};

volatile size_t rc_num_symbols = 0;

TaskHandle_t main_task_handle = NULL;



static bool rc_rx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_data
)
{
    rc_num_symbols = edata->num_symbols;

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(main_task_handle, &hpw);

    return hpw == pdTRUE;
}




i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_lcd_dev_handle;








void init_rc_pwm_rmt(void)
{
    for (int i = 0; i < RC_CH_COUNT; i++) {

        rmt_rx_channel_config_t cfg = {0};
        cfg.gpio_num = rc_pins[i];
        cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        cfg.resolution_hz = 1000000;
        cfg.mem_block_symbols = 64;
        cfg.flags.invert_in = false;
        cfg.flags.with_dma = false;

        rmt_new_rx_channel(&cfg, &rc_rx[i]);

        rmt_rx_event_callbacks_t cb = {
            .on_recv_done = rc_rx_done_callback
        };

        rmt_rx_register_event_callbacks(rc_rx[i], &cb, (void*)(uintptr_t)i);

        rmt_enable(rc_rx[i]);

        rmt_receive(
            rc_rx[i],
            rc_raw[i],
            64,
            &(rmt_receive_config_t){
                .signal_range_min_ns = 1000,
                .signal_range_max_ns = 3000000
            }
        );
    }
}






void process_rc_pwm(void)
{
    if (ulTaskNotifyTake(pdTRUE, 0)) {

        for (int ch = 0; ch < RC_CH_COUNT; ch++) {

            for (size_t i = 0; i < 64; i++) {

                rmt_symbol_word_t s = rc_raw[ch][i];

                if (s.level0 == 1) {

                    uint32_t us = s.duration0;

                    if (us >= 900 && us <= 2100) {
                        rc_pulse_us[ch] = us;
                        rc_new[ch] = true;
                        break;
                    }
                }
            }

            rmt_receive(
                rc_rx[ch],
                rc_raw[ch],
                64,
                &(rmt_receive_config_t){
                    .signal_range_min_ns = 1000,
                    .signal_range_max_ns = 3000000
                }
            );
        }
    }
}




drivebase_data_t drivebase_data = {
    .x_origin_mm = 20,
    .y_origin_mm = 20,
    .x_velocity_mm_s = 0,
    .target_x_velocity_mm_s = 0,
    .y_velocity_mm_s = 0,
    .target_y_velocity_mm_s = 0,
    .angle = 0,
    .target_angle = 0,
    .angular_velocity = 0,
    .target_angular_velocity = 0,
};

module_data_t module_0_data = {
    .cs_pin = 5,
    .x_pos_mm = 50,
    .y_pos_mm = 50,
    .current_angle = 0,
    .target_angle = 0,
    .current_surface_speed = 0,
    .target_surface_speed = 0,
};



void calculate_swerve_module_targets(drivebase_data_t *drivebase, module_data_t *module)
{
    float rx = module->x_pos_mm - drivebase->x_origin_mm;
    float ry = module->y_pos_mm - drivebase->y_origin_mm;

    float omega = drivebase->target_angular_velocity;

    float vx = drivebase->target_x_velocity_mm_s - (omega * ry);
    float vy = drivebase->target_y_velocity_mm_s + (omega * rx);

    float angle_rad = atan2f(vy, vx);

    if (angle_rad < 0) angle_rad += TWO_PI;

    module->target_angle = (uint16_t)((angle_rad * ENCODER_RESOLUTION) / TWO_PI);
    module->target_surface_speed = sqrtf((vx * vx) + (vy * vy));
}



void app_main(void){

    main_task_handle = xTaskGetCurrentTaskHandle();

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
        .queue_size = 1
    };


    

    spi_transaction_t spi_transaction_buffer;

    memset(&spi_transaction_buffer, 0, sizeof(spi_transaction_buffer));

    spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    
    spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);

    spi_transaction_data_t *sendbuf = spi_bus_dma_memory_alloc(SENDER_HOST, sizeof(spi_transaction_data_t), 0);
    spi_transaction_data_t *recvbuf = spi_bus_dma_memory_alloc(SENDER_HOST, sizeof(spi_transaction_data_t), 0);


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
    uint32_t loop_counter_randtarget = 0;


    init_rc_pwm_rmt();

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        if (main_isr_flag){
            loop_counter_randtarget++;


           process_rc_pwm();

            if (rc_new[0] || rc_new[1] || rc_new[2] || rc_new[3]) {

                rc_new[0] = rc_new[1] = rc_new[2] = rc_new[3] = false;

                printf("/*%ld, %ld, %ld, %ld*/\n",rc_pulse_us[0],rc_pulse_us[1],rc_pulse_us[2],rc_pulse_us[3]);

                // helper lambda-style mapping (manual inline)
                float y = ((float)((int32_t)rc_pulse_us[0] - 1500)) / 500.0f;
                float x = ((float)((int32_t)rc_pulse_us[1] - 1500)) / 500.0f;
                float w = ((float)((int32_t)rc_pulse_us[3] - 1500)) / 500.0f;

                // deadband (prevents drift)
                if (y > -0.15f && y < 0.15f) y = 0;
                if (x > -0.15f && x < 0.15f) x = 0;
                if (w > -0.15f && w < 0.15f) w = 0;

                // scale to your system units
                drivebase_data.target_y_velocity_mm_s = (y * 1000.0f)*0.1f + drivebase_data.target_y_velocity_mm_s*0.9; // simple low-pass filter to smooth out the control input
                drivebase_data.target_x_velocity_mm_s = (x * 1000.0f)*0.1f + drivebase_data.target_x_velocity_mm_s*0.9; // simple low-pass filter to smooth out the control input

                drivebase_data.target_angular_velocity = (w*10.0f)*0.1f + drivebase_data.target_angular_velocity*0.9; // simple low-pass filter to smooth out the control input
                // drivebase target angle is cooked
            }


            calculate_swerve_module_targets(&drivebase_data, &module_0_data);

            sendbuf->angle = module_0_data.target_angle;
            sendbuf->surface_speed = module_0_data.target_surface_speed;

            spi_transaction_buffer.length = sizeof(spi_transaction_data_t) * 8;
            spi_transaction_buffer.tx_buffer = sendbuf;
            spi_transaction_buffer.rx_buffer = recvbuf;

            spi_device_transmit(spi_handle, &spi_transaction_buffer);

            //printf("Drivebase target: vx=%f mm/s vy=%f mm/s omega=%ld rad/s\n",drivebase_data.target_x_velocity_mm_s,drivebase_data.target_y_velocity_mm_s,drivebase_data.target_angular_velocity);

            //printf("Module target: angle=%ld speed=%f mm/s\n",module_0_data.target_angle,module_0_data.target_surface_speed);

            //printf("sent data: angle=%ld surface_speed=%f\n", sendbuf->angle, sendbuf->surface_speed);

            //printf("Received data: angle=%ld surface_speed=%f\n",recvbuf->angle,recvbuf->surface_speed);
            
            main_isr_flag = false;
        
    }
    }
}
