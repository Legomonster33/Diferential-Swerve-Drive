#ifndef MOTOR_DATA_H
#define MOTOR_DATA_H

#include <stdint.h>


typedef struct {
    float rpm;
    float target_rpm;
    float error;
    float new_speed;
} motor_data_t;

#endif