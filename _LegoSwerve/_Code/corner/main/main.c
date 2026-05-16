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



#define RCV_HOST    SPI3_HOST

#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5


#define CHARACTERIZATION_SETTLE_MS 500
#define CHARACTERIZATION_SAMPLES 100
#define CHARACTERIZATION_SAMPLE_DELAY_MS 10


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
-1000,-996,-992,-988,-984,-980,-976,-972,-968,-964,-960,-956,-952,-948,-944,-940,
-936,-932,-928,-924,-920,-916,-912,-908,-904,-900,-896,-892,-888,-884,-880,-876,
-872,-868,-864,-860,-856,-852,-848,-844,-840,-836,-832,-828,-824,-820,-816,-812,
-808,-804,-800,-796,-792,-788,-784,-780,-776,-772,-768,-764,-760,-756,-752,-748,
-744,-740,-736,-732,-728,-724,-720,-716,-712,-708,-704,-700,-696,-692,-688,-684,
-680,-676,-672,-668,-664,-660,-656,-652,-648,-644,-640,-636,-632,-628,-624,-620,
-616,-612,-608,-604,-600,-596,-592,-588,-584,-580,-576,-572,-568,-564,-560,-556,
-552,-548,-544,-540,-536,-532,-528,-524,-520,-516,-512,-508,-504,-500,-496,-492,
-488,-484,-480,-476,-472,-468,-464,-460,-456,-452,-448,-444,-440,-436,-432,-428,
-424,-420,-416,-412,-408,-404,-400,-396,-392,-388,-384,-380,-376,-372,-368,-364,
-360,-356,-352,-348,-344,-340,-336,-332,-328,-324,-320,-316,-312,-308,-304,-300,
-296,-292,-288,-284,-280,-276,-272,-268,-264,-260,-256,-252,-248,-244,-240,-236,
-232,-228,-224,-220,-216,-212,-208,-204,-200,-196,-192,-188,-184,-180,-176,-172,
-168,-164,-160,-156,-152,-148,-144,-140,-136,-132,-128,-124,-120,-116,-112,-108,
-104,-100,-96,-92,-88,-84,-80,-76,-72,-68,-64,-60,-56,-52,-48,-44,
-40,-36,-32,-28,-24,-20,-16,-12,-8,-4,0,4,8,12,16,20,
24,28,32,36,40,44,48,52,56,60,64,68,72,76,80,84,
88,92,96,100,104,108,112,116,120,124,128,132,136,140,144,148,
152,156,160,164,168,172,176,180,184,188,192,196,200,204,208,212,
216,220,224,228,232,236,240,244,248,252,256,260,264,268,272,276,
280,284,288,292,296,300,304,308,312,316,320,324,328,332,336,340,
344,348,352,356,360,364,368,372,376,380,384,388,392,396,400,404,
408,412,416,420,424,428,432,436,440,444,448,452,456,460,464,468,
472,476,480,484,488,492,496,500,504,508,512,516,520,524,528,532,
536,540,544,548,552,556,560,564,568,572,576,580,584,588,592,596,
600,604,608,612,616,620,624,628,632,636,640,644,648,652,656,660,
664,668,672,676,680,684,688,692,696,700,704,708,712,716,720,724,
728,732,736,740,744,748,752,756,760,764,768,772,776,780,784,788,
792,796,800,804,808,812,816,820,824,828,832,836,840,844,848,852,
856,860,864,868,872,876,880,884,888,892,896,900,904,908,912,916,
920,924,928,932,936,940,944,948,952,956,960,964,968,972,976,980,
984,988,992,996,1000
};


int32_t motor_2_rpm_speed_lut[LUT_SIZE] = {
-1000,-996,-992,-988,-984,-980,-976,-972,-968,-964,-960,-956,-952,-948,-944,-940,
-936,-932,-928,-924,-920,-916,-912,-908,-904,-900,-896,-892,-888,-884,-880,-876,
-872,-868,-864,-860,-856,-852,-848,-844,-840,-836,-832,-828,-824,-820,-816,-812,
-808,-804,-800,-796,-792,-788,-784,-780,-776,-772,-768,-764,-760,-756,-752,-748,
-744,-740,-736,-732,-728,-724,-720,-716,-712,-708,-704,-700,-696,-692,-688,-684,
-680,-676,-672,-668,-664,-660,-656,-652,-648,-644,-640,-636,-632,-628,-624,-620,
-616,-612,-608,-604,-600,-596,-592,-588,-584,-580,-576,-572,-568,-564,-560,-556,
-552,-548,-544,-540,-536,-532,-528,-524,-520,-516,-512,-508,-504,-500,-496,-492,
-488,-484,-480,-476,-472,-468,-464,-460,-456,-452,-448,-444,-440,-436,-432,-428,
-424,-420,-416,-412,-408,-404,-400,-396,-392,-388,-384,-380,-376,-372,-368,-364,
-360,-356,-352,-348,-344,-340,-336,-332,-328,-324,-320,-316,-312,-308,-304,-300,
-296,-292,-288,-284,-280,-276,-272,-268,-264,-260,-256,-252,-248,-244,-240,-236,
-232,-228,-224,-220,-216,-212,-208,-204,-200,-196,-192,-188,-184,-180,-176,-172,
-168,-164,-160,-156,-152,-148,-144,-140,-136,-132,-128,-124,-120,-116,-112,-108,
-104,-100,-96,-92,-88,-84,-80,-76,-72,-68,-64,-60,-56,-52,-48,-44,
-40,-36,-32,-28,-24,-20,-16,-12,-8,-4,0,4,8,12,16,20,
24,28,32,36,40,44,48,52,56,60,64,68,72,76,80,84,
88,92,96,100,104,108,112,116,120,124,128,132,136,140,144,148,
152,156,160,164,168,172,176,180,184,188,192,196,200,204,208,212,
216,220,224,228,232,236,240,244,248,252,256,260,264,268,272,276,
280,284,288,292,296,300,304,308,312,316,320,324,328,332,336,340,
344,348,352,356,360,364,368,372,376,380,384,388,392,396,400,404,
408,412,416,420,424,428,432,436,440,444,448,452,456,460,464,468,
472,476,480,484,488,492,496,500,504,508,512,516,520,524,528,532,
536,540,544,548,552,556,560,564,568,572,576,580,584,588,592,596,
600,604,608,612,616,620,624,628,632,636,640,644,648,652,656,660,
664,668,672,676,680,684,688,692,696,700,704,708,712,716,720,724,
728,732,736,740,744,748,752,756,760,764,768,772,776,780,784,788,
792,796,800,804,808,812,816,820,824,828,832,836,840,844,848,852,
856,860,864,868,872,876,880,884,888,892,896,900,904,908,912,916,
920,924,928,932,936,940,944,948,952,956,960,964,968,972,976,980,
984,988,992,996,1000
};


typedef struct {
    int32_t command;
    float measured_rpm;
} characterization_point_t;

characterization_point_t characterization_data[LUT_SIZE];



static void build_feedforward_lut(
    int32_t output_lut[LUT_SIZE])
{
    for (int i = 0; i < LUT_SIZE; i++) {

        float target_rpm =
            -10000.0f +
            ((20000.0f * i) / (LUT_SIZE - 1));

        int32_t result_command = 0;

        for (int j = 0; j < LUT_SIZE - 1; j++) {

            float rpm0 =
                characterization_data[j].measured_rpm;

            float rpm1 =
                characterization_data[j + 1].measured_rpm;

            // avoid divide-by-zero
            if (fabs(rpm1 - rpm0) < 0.001f)
                continue;

            if ((target_rpm >= rpm0 &&
                 target_rpm <= rpm1) ||

                (target_rpm >= rpm1 &&
                 target_rpm <= rpm0)) {

                float t =
                    (target_rpm - rpm0) /
                    (rpm1 - rpm0);

                result_command =
                    characterization_data[j].command +

                    t * (
                        characterization_data[j + 1].command -
                        characterization_data[j].command);

                break;
            }
        }

        output_lut[i] = result_command;
    }

    // print generated LUT
    printf("\nGenerated LUT:\n");

    printf("int32_t rpm_speed_lut[%d] = {\n",
           LUT_SIZE);

    for (int i = 0; i < LUT_SIZE; i++) {

        printf("%ld,", output_lut[i]);

        if ((i % 16) == 15)
            printf("\n");
    }

    printf("};\n");
}

static void characterize_motor(
    motor_data_t *motor,
    int32_t output_lut[LUT_SIZE])
{
    printf("Starting characterization...\n");

    // =====================================================
    // Sweep command range
    // =====================================================

    for (int i = 0; i < LUT_SIZE; i++) {

        int32_t command =
            -1000 + ((2000 * i) / (LUT_SIZE - 1));

        motor->new_speed = command;
        motor->target_rpm = command * 10;

        ESP_ERROR_CHECK(
            mcpwm_comparator_set_compare_value(
                motor->pwm_comparator,
                map_speed_to_pulsewidth(command)));

        // let motor stabilize
        vTaskDelay(pdMS_TO_TICKS(
            CHARACTERIZATION_SETTLE_MS));

        // average RPM
        float rpm_sum = 0.0f;
        
        for (int k = 0; k < 100; k++) {
            update_rpm(motor);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        for (int j = 0; j < CHARACTERIZATION_SAMPLES; j++) {

            update_rpm(motor);

            rpm_sum += motor->rpm;

            vTaskDelay(pdMS_TO_TICKS(
                CHARACTERIZATION_SAMPLE_DELAY_MS));
        }

        characterization_data[i].command = command;

        characterization_data[i].measured_rpm =
            rpm_sum / CHARACTERIZATION_SAMPLES;

        printf(
            "[%3d] CMD:%4ld RPM:%8.2f\n",
            i,
            characterization_data[i].command,
            characterization_data[i].measured_rpm);
    }

    // stop motor
    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(
            motor->pwm_comparator,
            map_speed_to_pulsewidth(0)));

    vTaskDelay(pdMS_TO_TICKS(1000));

    // =====================================================
    // Build RPM -> command LUT
    // =====================================================

    build_feedforward_lut(output_lut);

    printf("Characterization complete.\n");
}





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

    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("before characterization");
    printf("\nDIPSWITCH1 (characterization mode) = %d\n", gpio_get_level(DIPSWITCH1));

    if (gpio_get_level(DIPSWITCH1) == 1) {

        printf("\n====================\n");
        printf("CHARACTERIZING M1\n");
        printf("====================\n");

        characterize_motor(
            &motor_1_data,
            motor_1_rpm_speed_lut);

        vTaskDelay(pdMS_TO_TICKS(2000));

        printf("\n====================\n");
        printf("CHARACTERIZING M2\n");
        printf("====================\n");

        characterize_motor(
            &motor_2_data,
            motor_2_rpm_speed_lut);

        printf("\nDONE\n");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
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
