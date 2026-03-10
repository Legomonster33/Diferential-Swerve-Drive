#ifndef CALCULATE_RPM_H
#define CALCULATE_RPM_H

#include "hall_data.h"
#include "motor_data.h"

#define PULSES_PER_REV 6
#define TIMER_FREQ_HZ 80000000ULL   // 1 MHz timer (microseconds)

#define TARGET_WINDOW_SEC 0.064f

#define MOTOR_STALL_TICKS 20000000

#define MIN_VALID_DT 80000

float calculate_rpm(hall_data_t hall_data, motor_data_t *motor_data);

#endif