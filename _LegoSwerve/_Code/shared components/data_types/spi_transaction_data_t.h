#ifndef SPI_TRANSACTION_DATA_H
#define SPI_TRANSACTION_DATA_H

#include <stdint.h>

typedef struct {
    uint32_t angle;
    float surface_speed;
} spi_transaction_data_t;

#endif