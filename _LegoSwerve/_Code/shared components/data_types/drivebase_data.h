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
      int32_t angle;
      int32_t target_angle;
      int32_t angular_velocity;
      int32_t target_angular_velocity;
} drivebase_data_t;

#endif