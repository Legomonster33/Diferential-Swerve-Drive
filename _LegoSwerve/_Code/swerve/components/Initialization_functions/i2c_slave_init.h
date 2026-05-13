#ifndef I2C_SLAVE_INIT_H
#define I2C_SLAVE_INIT_H

#include "driver/i2c_slave.h"
#include "esp_err.h"
#include "wheel_data.h"


#define I2C_SLAVE_SCL_IO           19       /*!< GPIO number used for I2C SLAVE clock */
#define I2C_SLAVE_SDA_IO           18       /*!< GPIO number used for I2C SLAVE data  */
#define I2C_SLAVE_NUM              I2C_NUM_1
#define I2C_SLAVE_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY

void i2c_new_slave_init(uint8_t slave_addr, wheel_data_t *wheel_data_ptr);

#endif // I2C_SLAVE_INIT_H



// if this doesnt work, try writing to the i2c buffer in the main loop instead of using the slave callback, and see if the data is correctly read by the master. If it works, then the issue is with the slave callback function, and we can debug it further. If it doesn't work, then the issue might be with the i2c configuration or wiring.