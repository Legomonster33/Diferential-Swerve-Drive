#ifndef MAP_TARGET_RPM_TO_SPEED_H
#define MAP_TARGET_RPM_TO_SPEED_H
#include <stdint.h>

#define MAX_RPM 11500.0f
#define MIN_RPM -9500.0f

float map_target_rpm_to_speed(float target_rpm);

#endif