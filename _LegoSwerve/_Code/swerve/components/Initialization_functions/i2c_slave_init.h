#ifndef I2C_SLAVE_INIT_H
#define I2C_SLAVE_INIT_H

#include "driver/i2c_slave.h"
#include "esp_err.h"


#define I2C_SLAVE_SCL_IO           19       /*!< GPIO number used for I2C SLAVE clock */
#define I2C_SLAVE_SDA_IO           18       /*!< GPIO number used for I2C SLAVE data  */
#define I2C_SLAVE_NUM              I2C_NUM_1
#define I2C_SLAVE_FREQ_HZ          CONFIG_I2C_SLAVE_FREQUENCY

void i2c_new_slave_init();

#endif // I2C_SLAVE_INIT_H