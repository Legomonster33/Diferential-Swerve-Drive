#include "i2c_slave_init.h"
#include "driver/i2c_slave.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

i2c_slave_dev_handle_t handle;

QueueHandle_t event_queue;

uint8_t command_data;

typedef enum {
    I2C_SLAVE_EVT_RX,
    I2C_SLAVE_EVT_TX
} i2c_slave_event_t;

uint32_t write_len = sizeof(wheel_data_t);
static bool i2c_slave_request_cb(i2c_slave_dev_handle_t handle, const i2c_slave_request_event_data_t *evt_data, void *wheel_data_ptr)
{
    BaseType_t xTaskWoken = 0;
    ESP_ERROR_CHECK(i2c_slave_write(handle, wheel_data_ptr, sizeof(wheel_data_t), &write_len, 1000));
    return xTaskWoken;
}

static bool i2c_slave_receive_cb(i2c_slave_dev_handle_t handle, const i2c_slave_rx_done_event_data_t *evt_data, void *wheel_data_ptr)
{
    BaseType_t xTaskWoken = 0;
    wheel_data_t *received_data = (wheel_data_t*)&evt_data;
    wheel_data_t *wheel_data = (wheel_data_t*)wheel_data_ptr;
    wheel_data->target_wheel_rpm = received_data->target_wheel_rpm;
    wheel_data->target_angle = received_data->target_angle;
    return xTaskWoken;
}


void i2c_new_slave_init(uint8_t slave_addr, wheel_data_t *wheel_data_ptr){
    
    i2c_slave_config_t i2c_slv_config = {
            .i2c_port = I2C_SLAVE_NUM,
            .clk_source = I2C_SLAVE_FREQ_HZ,
            .scl_io_num = I2C_SLAVE_SCL_IO,
            .sda_io_num = I2C_SLAVE_SDA_IO,
            .slave_addr = slave_addr,
            .send_buf_depth = 100,
            .receive_buf_depth = 100,
        };
    
    
    
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_slv_config, &handle));
    
    i2c_slave_event_callbacks_t cbs = {
        .on_receive = i2c_slave_receive_cb,
        .on_request = i2c_slave_request_cb,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(handle, &cbs, &wheel_data_ptr));
    
}
