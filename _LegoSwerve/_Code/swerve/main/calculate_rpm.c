#include "calculate_rpm.h"



float calculate_rpm(hall_data_t hall_data){
    float rpm = 0.0;


    uint64_t dt_since_last_pulse = hall_data.gptimer_last_update_timestamp - hall_data.gptimer_last_isr_timestamp;

    /*
    ESP_LOGI(TAG, "gptimer_last_update_timestamp: %llu ticks", hall_data->gptimer_last_update_timestamp);
    ESP_LOGI(TAG, "gptimer_last_isr_timestamp: %llu ticks", hall_data->gptimer_last_isr_timestamp);
    ESP_LOGI(TAG, "deltaTime since last update vs since last pulse: %llu ticks", dt_since_last_pulse);
    */

    if (dt_since_last_pulse > MOTOR_STALL_TICKS) {
        rpm = 0.0;
        //ESP_LOGI(TAG, "0 rpm");
        return rpm;
    }

    //ESP_LOGI(TAG,"nonzero rpm");
    
    switch (hall_data.mode){

        case MODE_SINGLE_PERIOD:{
            uint32_t current_index  = hall_data.hall_timestamps_index;
            
            uint32_t newer_index = (current_index + 63) % 64; // index of last written timestamp
            
            uint32_t older_index = (current_index + 62) % 64; // index of the timestamp before last written

            uint32_t newer = hall_data.hall_timestamps[newer_index];
            uint32_t older = hall_data.hall_timestamps[older_index];

            uint32_t dt = newer - older;

                
            if (newer != older){
                rpm =((60ULL * TIMER_FREQ_HZ)/(dt * PULSES_PER_REV));
                }
            /*
            if (rpm == 0.0) {
                ESP_LOGI(TAG, "Single period mode: newer timestamp: %u ticks, older timestamp: %u ticks, dt: %u ticks, rpm: %llu", newer, older, dt, rpm);
            }
            */
        break;
        
        }

        case MODE_MULTIPLE_PERIOD:
    {
            uint32_t current_index  = hall_data.hall_timestamps_index;

            uint32_t new_count = hall_data.difference_total_triggers_last_update;

            //P_LOGI(TAG, "New pulse count since last update: %u", new_count); // always 0 for some reason, must redo math

                
                double total_dt = 0.0;
                uint32_t valid_periods = 0;

                // loop over new periods
                for (uint32_t i = 0; i < new_count; i++)
                {
                    uint64_t newer = hall_data.hall_timestamps[(current_index - i + 63) % 64];
                    uint64_t older = hall_data.hall_timestamps[(current_index - i + 62) % 64];

                    double dt = (double)(newer - older); // difference in timer ticks
                    total_dt += dt;
                    valid_periods++;
                }

                double avg_dt = total_dt / valid_periods;

                rpm = (60ULL * TIMER_FREQ_HZ) / (avg_dt * PULSES_PER_REV);

                /*
                if(rpm == 0.0 || rpm != rpm) { // Check for NaN
                    ESP_LOGI(TAG, "Multiple period mode: new_count: %u, valid_periods: %u, total_dt: %llu ticks, avg_dt: %llu ticks, rpm: %llu", new_count, valid_periods, total_dt, (valid_periods > 0) ? (total_dt / valid_periods) : 0, rpm);
                }
                */

    break;

        }
    }

        return rpm;
}