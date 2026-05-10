#ifndef CALCULATE_RPM_H
#define CALCULATE_RPM_H

#include "hall_data.h"
#include "motor_data.h"

#define PULSES_PER_REV  1.84615384615f
#define TIMER_FREQ_HZ 80000000   // 1 MHz timer (microseconds)

#define TARGET_WINDOW_SEC 0.125f // target window, seconds. 

#define MOTOR_STALL_TICKS 50

#define MIN_VALID_DT 60000

float calculate_rpm(motor_data_t *motor_data);

#endif