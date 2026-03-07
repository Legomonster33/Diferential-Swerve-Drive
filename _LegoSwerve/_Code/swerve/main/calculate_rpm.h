#ifndef CALCULATE_RPM_H
#define CALCULATE_RPM_H

#include "hall_data.h"

#define PULSES_PER_REV 6
#define TIMER_FREQ_HZ 80000000ULL   // 1 MHz timer (microseconds)

#define MOTOR_STALL_TICKS 20000000

float calculate_rpm(hall_data_t hall_data);

#endif