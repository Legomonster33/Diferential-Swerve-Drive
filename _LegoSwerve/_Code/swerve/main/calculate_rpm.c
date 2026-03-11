#include "calculate_rpm.h"
#include "esp_log.h"
#include "math.h"
#include <stdio.h>

static const char *TAG = "calculate_rpm:";


// Comparator function for qsort on uint32_t
static int cmp_u32(const void *a, const void *b)
{
    uint32_t ua = *(const uint32_t *)a;
    uint32_t ub = *(const uint32_t *)b;

    if (ua < ub) return -1;
    if (ua > ub) return 1;
    return 0;
}

float calculate_rpm(hall_data_t hall_data, motor_data_t *motor_data) {
    float rpm;


    if (hall_data.ticks_since_last_trigger > MOTOR_STALL_TICKS) {
        rpm = 0.0;
        //ESP_LOGI(TAG, "0 rpm stall detected dt %llu, hall_data.gptimer_last_update_timestamp: %llu, hall_data.gptimer_last_isr_timestamp: %llu", dt_since_last_pulse, hall_data.gptimer_last_update_timestamp, hall_data.gptimer_last_isr_timestamp);
        return rpm;
    }

    float pulses_per_second = (fabsf(motor_data->target_rpm) * PULSES_PER_REV) / 60.0f;

    uint32_t pulses_to_median = pulses_per_second * TARGET_WINDOW_SEC;

    if (pulses_to_median < 1)
    pulses_to_median = 8;

    if (pulses_to_median > 128)
    pulses_to_median = 128;

    //pulses_to_median = 1;

    uint32_t current_index = hall_data.hall_timestamps_index;

    uint32_t dt_median_array[pulses_to_median];


    uint32_t oldest_index = (current_index + HALL_BUFFER_SIZE - pulses_to_median - 1) % HALL_BUFFER_SIZE;

    for (int i = 0; i < pulses_to_median; i++) {

        uint32_t older = hall_data.hall_timestamps[(oldest_index + i) % HALL_BUFFER_SIZE];
        uint32_t newer = hall_data.hall_timestamps[(oldest_index + i + 1) % HALL_BUFFER_SIZE];

        uint32_t delta = newer - older;

        dt_median_array[i] = delta;
    }

    qsort(dt_median_array, pulses_to_median, sizeof(uint32_t), cmp_u32);

    int trim = pulses_to_median / 4;

    uint64_t sum = 0;
    int count = 0;

    for (int i = trim; i < pulses_to_median - trim; i++) {
        sum += dt_median_array[i];
        count++;
    }


    float dt = (float)sum / (float)count;


    if (dt == 0){
        return motor_data->rpm;
    }
    rpm =((60ULL * TIMER_FREQ_HZ)/(dt * PULSES_PER_REV));

    if (isnan(rpm) || rpm > 15000 || rpm < -15000) {
        ESP_LOGI(TAG, "strange rpm %f", rpm);
        ESP_LOGI(TAG, "hall_data.hall_timestamps_index: %u", hall_data.hall_timestamps_index);
        ESP_LOGI(TAG, "hall_data.ticks_since_last_trigger: %u", hall_data.ticks_since_last_trigger);
        ESP_LOGI(TAG, "hall_data.total_trigger_count: %u", hall_data.total_trigger_count);
        ESP_LOGI(TAG, "hall_data.last_total_trigger_count: %u", hall_data.last_total_trigger_count);
        ESP_LOGI(TAG, "count: %d", count);
        ESP_LOGI(TAG, "dt: %f", dt);
        ESP_LOGI(TAG, "pulses_to_median: %u", pulses_to_median);
        ESP_LOGI(TAG, "trim: %d", trim);
        for (int i = 0; i < HALL_BUFFER_SIZE; i++) {
            ESP_LOGI(TAG, "hall_timestamps[%d]: %u", i, hall_data.hall_timestamps[i]);
        }
        for (int i = 0; i < pulses_to_median; i++) {
            ESP_LOGI(TAG, "dt_median_array[%d]: %u", i, dt_median_array[i]);
        }   
    }
    
    //printf("/*%.1f,%.1lf,%lu*/\r\n", rpm, dt, pulses_to_median);

    return rpm;
}