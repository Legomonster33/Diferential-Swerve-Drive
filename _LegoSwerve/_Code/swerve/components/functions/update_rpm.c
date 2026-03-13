#include "update_rpm.h"
#include "calculate_rpm.h"
#include "hall_data.h"

void update_rpm(motor_data_t *motor_data){
  hall_data_t Hall_1_local_copy = motor_data->hall_data; 

        motor_data->hall_data.last_total_trigger_count = Hall_1_local_copy.total_trigger_count;
        motor_data->hall_data.hall_timestamps_last_index = Hall_1_local_copy.hall_timestamps_index;

        if (Hall_1_local_copy.total_trigger_count == Hall_1_local_copy.last_total_trigger_count) {
            motor_data->hall_data.ticks_since_last_trigger += 1;
        } else {
            motor_data->hall_data.ticks_since_last_trigger = 0;
        }

        float smoothing_factor = 0.9f - (motor_data->target_rpm * 0.85f / 2000.0f);

        if (smoothing_factor < 0.1f) smoothing_factor = 0.1f;
        if (smoothing_factor > 0.9f) smoothing_factor = 0.9f;

        float measured_rpm = calculate_rpm(motor_data);

        motor_data->rpm = (1.0f - smoothing_factor) * motor_data->rpm + smoothing_factor * measured_rpm;

        if (motor_data->new_speed < 0) {   // negate rpm if speed output is negative.
            motor_data->rpm = -motor_data->rpm;
        }
}