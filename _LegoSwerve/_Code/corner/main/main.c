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

#include "spi_transaction_data_t.h"

#include "update_rpm.h"
#include "update_pid_feedforward.h"
#include "map_speed_to_pulsewidth.h"

#include "init_pid.h"
#include "init_gptimer.h"
#include "init_capture_timer.h"
#include "init_pwm_operator.h"



#include "lcd_functions.h"

#include "i2c_master_init.h"
#include "calculate_rpm.h"

#include "nvs_flash.h"


#define RCV_HOST    SPI3_HOST

#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5


#define CHARACTERIZATION_SETTLE_MS 500
#define CHARACTERIZATION_SAMPLES 50
#define CHARACTERIZATION_SAMPLE_DELAY_MS 5


bool main_isr_flag = false;

motor_data_t motor_1_data = {0};
motor_data_t motor_2_data = {0};

wheel_data_t wheel_data = {0};

//static portMUX_TYPE my_mux = portMUX_INITIALIZER_UNLOCKED; //if we want to use taskENTER_CRITICAL

#define DIPSWITCH0 16
#define DIPSWITCH1 17

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_encoder_dev_handle;
i2c_master_dev_handle_t i2c_lcd_dev_handle;

int32_t motor_1_rpm_speed_lut[LUT_SIZE] = {
-1000,-984,-968,-952,-937,-921,-905,-889,
-874,-858,-842,-826,-811,-795,-779,-763,
-748,-732,-716,-700,-685,-669,-653,-637,
-622,-606,-590,-574,-559,-543,-527,-511,
-496,-480,-464,-448,-433,-417,-401,-385,
-370,-354,-338,-322,-307,-291,-275,-259,
-244,-228,-212,-196,-181,-165,-149,-133,
-118,-102,-86,-70,-55,-39,-23,-7,
7,23,39,55,70,86,102,118,
133,149,165,181,196,212,228,244,
259,275,291,307,322,338,354,370,
385,401,417,433,448,464,480,496,
511,527,543,559,574,590,606,622,
637,653,669,685,700,716,732,748,
763,779,795,811,826,842,858,874,
889,905,921,937,952,968,984,1000
};


int32_t motor_2_rpm_speed_lut[LUT_SIZE] = {
-1000,-984,-968,-952,-937,-921,-905,-889,
-874,-858,-842,-826,-811,-795,-779,-763,
-748,-732,-716,-700,-685,-669,-653,-637,
-622,-606,-590,-574,-559,-543,-527,-511,
-496,-480,-464,-448,-433,-417,-401,-385,
-370,-354,-338,-322,-307,-291,-275,-259,
-244,-228,-212,-196,-181,-165,-149,-133,
-118,-102,-86,-70,-55,-39,-23,-7,
7,23,39,55,70,86,102,118,
133,149,165,181,196,212,228,244,
259,275,291,307,322,338,354,370,
385,401,417,433,448,464,480,496,
511,527,543,559,574,590,606,622,
637,653,669,685,700,716,732,748,
763,779,795,811,826,842,858,874,
889,905,921,937,952,968,984,1000
};


typedef struct {
    int32_t command;
    float measured_rpm;
} characterization_point_t;

characterization_point_t characterization_data[LUT_SIZE];



static void build_feedforward_lut(int32_t output_lut[LUT_SIZE]) {
    for (int i = 0; i < LUT_SIZE; i++) {
        float target_rpm = -10000.0f + ((20000.0f * i) / (LUT_SIZE - 1));
        int32_t result_command = 0;
        for (int j = 0; j < LUT_SIZE - 1; j++) {
            float rpm0 = characterization_data[j].measured_rpm;
            float rpm1 = characterization_data[j + 1].measured_rpm;
            if (fabs(rpm1 - rpm0) < 0.001f) continue;
            if ((target_rpm >= rpm0 && target_rpm <= rpm1) || (target_rpm >= rpm1 && target_rpm <= rpm0)) {
                float t = (target_rpm - rpm0) / (rpm1 - rpm0);
                result_command = characterization_data[j].command + t * (characterization_data[j + 1].command - characterization_data[j].command);
                break;
            }
        }
        output_lut[i] = result_command;
    }
}

static void characterize_motor(motor_data_t *motor, int32_t output_lut[LUT_SIZE]) {
    printf("Starting characterization...\n");
    for (int i = 0; i < LUT_SIZE; i++) {
        int32_t command = -1000 + ((2000 * i) / (LUT_SIZE - 1));
        motor->new_speed = command; motor->target_rpm = command * 10;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->pwm_comparator, map_speed_to_pulsewidth(command)));
        vTaskDelay(pdMS_TO_TICKS(CHARACTERIZATION_SETTLE_MS));
        float rpm_sum = 0.0f;
        for (int j = 0; j < CHARACTERIZATION_SAMPLES; j++) {
            hall_data_t Hall_1_local_copy = motor->hall_data;
            motor->hall_data.last_total_trigger_count = Hall_1_local_copy.total_trigger_count;
            motor->hall_data.hall_timestamps_last_index = Hall_1_local_copy.hall_timestamps_index;
            motor->hall_data.ticks_since_last_trigger = (Hall_1_local_copy.total_trigger_count == Hall_1_local_copy.last_total_trigger_count) ? motor->hall_data.ticks_since_last_trigger + 1 : 0;
            motor->rpm = (motor->new_speed < 0) ? -calculate_rpm(motor) : calculate_rpm(motor);
            rpm_sum += motor->rpm;
            vTaskDelay(pdMS_TO_TICKS(CHARACTERIZATION_SAMPLE_DELAY_MS));
        }
        characterization_data[i].command = command;
        characterization_data[i].measured_rpm = rpm_sum / CHARACTERIZATION_SAMPLES;
        printf("[%3d] CMD:%4ld RPM:%8.2f\n", i, characterization_data[i].command, characterization_data[i].measured_rpm);
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->pwm_comparator, map_speed_to_pulsewidth(0)));
    vTaskDelay(pdMS_TO_TICKS(1000));
    build_feedforward_lut(output_lut);
    printf("Characterization complete.\n");
}

esp_err_t save_lut_to_nvs(const char *key,int32_t *lut,size_t lut_size){
    nvs_handle_t nvs_handle;

    esp_err_t err =
        nvs_open("storage", NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(nvs_handle,key,lut,lut_size * sizeof(int32_t));

    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    return err;
}


esp_err_t load_lut_from_nvs(const char *key,int32_t *lut,size_t lut_size){
    nvs_handle_t nvs_handle;

    esp_err_t err = nvs_open("storage",NVS_READONLY,&nvs_handle);

    if (err != ESP_OK)
        return err;

    size_t required_size = 0;

    err = nvs_get_blob(nvs_handle,key,NULL,&required_size);

    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    if (required_size != lut_size * sizeof(int32_t)) {

        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_SIZE;
    }

    err = nvs_get_blob(nvs_handle,key,lut,&required_size);
    nvs_close(nvs_handle);

    return err;
}





void app_main(void){
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);


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


    spi_transaction_data_t *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, sizeof(spi_transaction_data_t), 0);
    spi_transaction_data_t *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, sizeof(spi_transaction_data_t), 0);



    spi_slave_transaction_t spi_transaction_buffer = {0};

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DIPSWITCH0) | (1ULL << DIPSWITCH1),
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
    wheel_data.target_wheel_surface_speed = 0; // set by orchestrator.


    uint8_t raw_angle_low_byte;
    uint8_t raw_angle_high_byte;
    uint8_t raw_angle_low_byte_address = 0x0D;
    uint8_t raw_angle_high_byte_address = 0x0C;

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("before characterization");
    printf("\nDIPSWITCH1 (characterization mode) = %d\n", gpio_get_level(DIPSWITCH1));

    if (gpio_get_level(DIPSWITCH1) == 1) {

        printf("\n====================\n");
        printf("CHARACTERIZING M1\n");
        printf("====================\n");

        characterize_motor(&motor_1_data,motor_1_rpm_speed_lut);

        vTaskDelay(pdMS_TO_TICKS(1000));

        printf("\n====================\n");
        printf("CHARACTERIZING M2\n");
        printf("====================\n");

        characterize_motor(&motor_2_data,motor_2_rpm_speed_lut);

        printf("\nDONE\n");

        ESP_ERROR_CHECK(save_lut_to_nvs("motor1_lut",motor_1_rpm_speed_lut,LUT_SIZE));

        ESP_ERROR_CHECK(save_lut_to_nvs("motor2_lut",motor_2_rpm_speed_lut,LUT_SIZE));

        printf("LUTs saved to NVS.\n");

    }else {

        load_lut_from_nvs("motor1_lut",motor_1_rpm_speed_lut,LUT_SIZE);

        load_lut_from_nvs("motor2_lut",motor_2_rpm_speed_lut,LUT_SIZE);
        printf("LUTs loaded from NVS.\n");
        }
    

    for (int i = 0; i < LUT_SIZE; i++) {
            printf("%ld, %ld\n", motor_1_rpm_speed_lut[i], motor_2_rpm_speed_lut[i]);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("after characterization");

    while (1) {
        vTaskDelay(1); //let idle task run, otherwise watchdog triggers

        wheel_data.real_angle = wheel_data.drive_reverse ? (2048 + wheel_data.current_angle) % 4096 : wheel_data.current_angle;
        wheel_data.current_wheel_surface_speed = (fabs(motor_1_data.rpm) + fabs(motor_2_data.rpm)) / (2 * wheel_config.wheel_rpm_ratio);
        
        sendbuf->angle = wheel_data.real_angle;
        sendbuf->surface_speed = wheel_data.current_wheel_surface_speed;

        spi_transaction_buffer.length = sizeof(spi_transaction_data_t) * 8;
        spi_transaction_buffer.tx_buffer = sendbuf;
        spi_transaction_buffer.rx_buffer = recvbuf;

        spi_slave_transmit(RCV_HOST, &spi_transaction_buffer, 0);
        
        wheel_data.target_angle = recvbuf->angle;

        wheel_data.target_wheel_surface_speed = recvbuf->surface_speed;
        
        //printf("Sent data: angle = %ld, surface_speed = %f\n", sendbuf->angle, sendbuf->surface_speed);
        //printf("received data: angle = %ld, surface_speed = %f\n", recvbuf->angle, recvbuf->surface_speed);

        loop_counter_50hz ++;
        loop_counter_1hz ++;
        loop_counter_randtarget ++;

        /*
        if (loop_counter_randtarget == 600) {
            loop_counter_randtarget = 0;
            wheel_data.target_angle = rand() % 4096;
            wheel_data.target_wheel_surface_speed = rand() % 100;
        }
        */

        if (loop_counter_1hz == 200){
            loop_counter_1hz = 0;
            char display_str[21];
            LCD_Clear_Line(1);
            LCD_SetCursor(1, 0);
            sprintf(display_str, "TA:%-4d", wheel_data.target_angle);
            LCD_Print(display_str);
            LCD_SetCursor(1, 10);
            sprintf(display_str, "TS:%4.0f", wheel_data.target_wheel_surface_speed);
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


        if (gpio_get_level(DIPSWITCH0) == 1) {


            // shortest-path angle error
            int32_t diff = (int32_t)wheel_data.target_angle - (int32_t)wheel_data.current_angle;

            if (diff > 2048) diff = -4096 + diff;
            else if (diff < -2048) diff = 4096 + diff;

            int32_t normal_diff = (int32_t)wheel_data.target_angle - (int32_t)wheel_data.current_angle;

            if (normal_diff > 2048) normal_diff -= 4096;
            if (normal_diff < -2048) normal_diff += 4096;


            // reversed target = target + 180 degrees
            int32_t reversed_target = (wheel_data.target_angle + 2048) % 4096;

            int32_t reversed_diff = reversed_target - (int32_t)wheel_data.current_angle;

            if (reversed_diff > 2048) reversed_diff -= 4096;
            if (reversed_diff < -2048) reversed_diff += 4096;


            // choose shortest path
            if (abs(reversed_diff) < abs(normal_diff)) {

                wheel_data.drive_reverse = true;
                wheel_data.angle_error = reversed_diff;

            } else {

                wheel_data.drive_reverse = false;
                wheel_data.angle_error = normal_diff;
            }
            

            wheel_data.target_motor_rpm = wheel_data.target_wheel_surface_speed * wheel_config.wheel_rpm_ratio * (wheel_data.drive_reverse ? -1.0f : 1.0f);



            
            pid_compute(wheel_data.pid_ctrl, wheel_data.angle_error, &wheel_data.motor_rpm_differential);
                    
                    

            motor_1_data.target_rpm = -wheel_data.target_motor_rpm - wheel_data.motor_rpm_differential;

            motor_2_data.target_rpm = wheel_data.target_motor_rpm - wheel_data.motor_rpm_differential;
                    
                
                
                
            


            update_rpm(&motor_1_data);
            update_rpm(&motor_2_data);
            
            update_pid_feedforward(&motor_1_data, &motor_1_config, motor_1_rpm_speed_lut);
            update_pid_feedforward(&motor_2_data, &motor_2_config, motor_2_rpm_speed_lut);

            motor_1_data.new_speed = motor_1_data.feedforward + motor_1_data.pid_output;
            motor_2_data.new_speed = motor_2_data.feedforward + motor_2_data.pid_output;

        }else {
            motor_1_data.new_speed = 0;
            motor_2_data.new_speed = 0;
        }



        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_1_data.pwm_comparator, map_speed_to_pulsewidth(motor_1_data.new_speed)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor_2_data.pwm_comparator, map_speed_to_pulsewidth(motor_2_data.new_speed)));
        
        printf("/*%.0f,%.0f,%.0f,%.0f,", motor_1_data.rpm, motor_1_data.target_rpm, motor_1_data.new_speed,motor_1_data.error);
        printf("%.0f,%.0f,%.0f,%.0f,", motor_2_data.rpm, motor_2_data.target_rpm, motor_2_data.new_speed,motor_2_data.error);
        printf("%.0f,%.0d,%.0d,%.0f,%.0f*/\n", wheel_data.target_wheel_surface_speed, wheel_data.current_angle, wheel_data.target_angle, wheel_data.angle_error, wheel_data.motor_rpm_differential);
        

        
            //main_isr_flag = false;
        
    //}
    }
}
