#ifndef DRIVEBASE_DATA_H
#define DRIVEBASE_DATA_H

#include <stdint.h>

typedef struct {
      float x_origin_mm;
      float y_origin_mm;
      float x_velocity_mm_s;
      float target_x_velocity_mm_s;
      float y_velocity_mm_s;
      float target_y_velocity_mm_s;
      uint32_t angle;
      uint32_t target_angle;
      uint32_t angular_velocity;
      uint32_t target_angular_velocity;
} drivebase_data_t;

#endif