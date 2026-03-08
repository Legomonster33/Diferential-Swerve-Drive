#include "calculate_rpm.h"
#include "esp_log.h"

static const char *TAG = "calculate_rpm:";

float calculate_rpm(hall_data_t hall_data,float last_rpm) {
    float rpm = 0.0;


    uint64_t dt_since_last_pulse = 0;

    if(hall_data.gptimer_last_update_timestamp > hall_data.gptimer_last_isr_timestamp){
        dt_since_last_pulse = hall_data.gptimer_last_update_timestamp - hall_data.gptimer_last_isr_timestamp;
    }
    
    if (dt_since_last_pulse > MOTOR_STALL_TICKS) {
        rpm = 0.0;
        //ESP_LOGI(TAG, "0 rpm stall detected dt %llu, hall_data.gptimer_last_update_timestamp: %llu, hall_data.gptimer_last_isr_timestamp: %llu", dt_since_last_pulse, hall_data.gptimer_last_update_timestamp, hall_data.gptimer_last_isr_timestamp);
        return rpm;
    }



    uint32_t current_index  = hall_data.hall_timestamps_index;
    uint32_t older_index;

    uint32_t newer_index = current_index;

    uint32_t pulses_since_last_update = hall_data.total_trigger_count - hall_data.last_total_trigger_count;

    uint32_t newer = hall_data.hall_timestamps[newer_index];
    uint32_t older;

    uint64_t dt = 0;

    int mode_ran = 0;


    if (pulses_since_last_update < 3) {


            older_index = (current_index + 63) % 64; // index of the timestamp before last written

            older = hall_data.hall_timestamps[older_index];

            dt = (newer > older) ? (newer - older) : ((UINT32_MAX - older) + newer);

            mode_ran = 0;


        }
        else {
            
            older_index = (current_index + 64 - pulses_since_last_update) % 64; // index of the timestamp before last written

            older = hall_data.hall_timestamps[older_index];

            
            uint64_t total_dt = (newer > older) ? (newer - older) : ((UINT32_MAX - older) + newer);

            dt = total_dt / pulses_since_last_update;

            mode_ran = 1;
                
            
        
    }

    if (dt == 0) {
                //ESP_LOGI(TAG, "dt is 0, returning last rpm %f", last_rpm);
                return last_rpm; // avoid division by zero, return last known rpm
            }
            rpm =((60ULL * TIMER_FREQ_HZ)/(dt * PULSES_PER_REV));


    /*
    if (rpm != rpm || rpm > 10000 || rpm < -10000 || (rpm < 500 && rpm > -500)) {
        ESP_LOGI(TAG, "strange rpm %f", rpm);
        ESP_LOGI(TAG, "hall_data.hall_timestamps_index: %u", hall_data.hall_timestamps_index);
        ESP_LOGI(TAG, "hall_data.gptimer_last_update_timestamp: %llu", hall_data.gptimer_last_update_timestamp);
        ESP_LOGI(TAG, "hall_data.gptimer_last_isr_timestamp: %llu", hall_data.gptimer_last_isr_timestamp);
        ESP_LOGI(TAG, "hall_data.total_trigger_count: %u", hall_data.total_trigger_count);
        ESP_LOGI(TAG, "hall_data.last_total_trigger_count: %u", hall_data.last_total_trigger_count);
        ESP_LOGI(TAG, "mode_ran: %d", mode_ran);
        ESP_LOGI(TAG, "newer index %u" , newer_index);
        ESP_LOGI(TAG, "older index %u" , older_index);
        ESP_LOGI(TAG, "newer timestamp: %u" , newer);
        ESP_LOGI(TAG, "older timestamp: %u" , older);
        for (int i = 0; i < 64; i++) {
            ESP_LOGI(TAG, "hall_timestamps[%d]: %u", i, hall_data.hall_timestamps[i]);
        }
    }
    */
    return rpm;
}