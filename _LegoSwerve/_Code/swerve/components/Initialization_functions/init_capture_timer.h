#ifndef INIT_CAPTURE_TIMER_H
#define INIT_CAPTURE_TIMER_H

#include "driver/mcpwm_cap.h"
#include "motor_data.h"



void init_capture_timer(int hall_pin, hall_data_t *hall_data);

#endif // INIT_CAPTURE_TIMER_H