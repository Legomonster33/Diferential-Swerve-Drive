#ifndef MODULE_DATA_H
#define MODULE_DATA_H

#include <stdint.h>

typedef struct {
      uint8_t cs_pin;
      float x_pos_mm;
      float y_pos_mm;
      uint32_t current_angle;
      uint32_t target_angle;
      float current_surface_speed;
      float target_surface_speed;
} module_data_t;

#endif