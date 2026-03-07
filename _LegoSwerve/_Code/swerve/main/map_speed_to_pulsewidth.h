#ifndef MAP_SPEED_TO_PULSEWIDTH_H
#define MAP_SPEED_TO_PULSEWIDTH_H

#include "hall_data.h"

#define MIN_PULSEWIDTH 1000  // Minimum pulse width in microsecond
#define DEADBAND_LOWER    1450 // Tested, dont change
#define DEADBAND_UPPER    1540 // Tested, dont change
#define MAX_PULSEWIDTH 2000  // Maximum pulse width in microsecond
#define CENTER_PULSE ((MIN_PULSEWIDTH + MAX_PULSEWIDTH) / 2) // Center pulse width in microsecond

#define MIN_SPEED        -1000
#define MAX_SPEED        1000

uint32_t map_speed_to_pulsewidth(int speed);

#endif