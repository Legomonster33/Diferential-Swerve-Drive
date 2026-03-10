#include "calculate_rpm.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "calculate_rpm:";

float calculate_rpm(hall_data_t hall_data, motor_data_t *motor_data) {
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


    float pulses_per_second = (fabsf(motor_data->target_rpm) * PULSES_PER_REV) / 60.0f;

    uint32_t pulses_to_average = pulses_per_second * TARGET_WINDOW_SEC;

    if (pulses_to_average < 1)
    pulses_to_average = 1;

    if (pulses_to_average > 64)
    pulses_to_average = 64;

    uint32_t current_index  = hall_data.hall_timestamps_index;
    
    uint32_t newer_index = current_index;
    uint32_t newer = hall_data.hall_timestamps[newer_index];
    
    uint32_t older_index = (current_index + HALL_BUFFER_SIZE - pulses_to_average) % HALL_BUFFER_SIZE; // index of the timestamp to average from
    uint32_t older = hall_data.hall_timestamps[older_index];

    uint64_t total_dt = (newer > older) ? (newer - older) : ((UINT32_MAX - older) + newer);

    uint64_t dt = total_dt / pulses_to_average;

                
            

    if (dt < MIN_VALID_DT) {
                //ESP_LOGI(TAG, "dt is 0, returning last rpm %f", last_rpm);
                return motor_data->rpm; // avoid division by zero, return last known rpm
            }
            rpm =((60ULL * TIMER_FREQ_HZ)/(dt * PULSES_PER_REV));


    
    if (rpm != rpm || rpm > 11000 || rpm < -11000) {
        ESP_LOGI(TAG, "strange rpm %f", rpm);
        ESP_LOGI(TAG, "hall_data.hall_timestamps_index: %u", hall_data.hall_timestamps_index);
        ESP_LOGI(TAG, "hall_data.gptimer_last_update_timestamp: %llu", hall_data.gptimer_last_update_timestamp);
        ESP_LOGI(TAG, "hall_data.gptimer_last_isr_timestamp: %llu", hall_data.gptimer_last_isr_timestamp);
        ESP_LOGI(TAG, "hall_data.total_trigger_count: %u", hall_data.total_trigger_count);
        ESP_LOGI(TAG, "hall_data.last_total_trigger_count: %u", hall_data.last_total_trigger_count);
        ESP_LOGI(TAG, "newer index %u" , newer_index);
        ESP_LOGI(TAG, "older index %u" , older_index);
        ESP_LOGI(TAG, "newer timestamp: %u" , newer);
        ESP_LOGI(TAG, "older timestamp: %u" , older);
        for (int i = 0; i < HALL_BUFFER_SIZE; i++) {
            ESP_LOGI(TAG, "hall_timestamps[%d]: %u", i, hall_data.hall_timestamps[i]);
        }
    }
    
    return rpm;
}