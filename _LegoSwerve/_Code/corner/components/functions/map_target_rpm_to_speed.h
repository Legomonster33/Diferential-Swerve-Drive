#ifndef MAP_TARGET_RPM_TO_SPEED_H
#define MAP_TARGET_RPM_TO_SPEED_H
#include <stdint.h>


#define LUT_SIZE 512
#define RPM_MIN -10000.0f
#define RPM_MAX  10000.0f

float map_target_rpm_to_speed(float target_rpm, int32_t rpm_speed_lookuptable[LUT_SIZE]);

#endif