#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

#include <stdint.h>
#include "hall_data.h"

typedef struct {
    float rpm;
    float target_rpm;
    float error;
    float new_speed;
    float feedforward;
    float pid_output;
    hall_data_t hall_data;
} motor_data_t;

#endif