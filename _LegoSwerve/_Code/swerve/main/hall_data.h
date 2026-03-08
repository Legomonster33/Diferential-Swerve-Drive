#ifndef HALL_DATA_H
#define HALL_DATA_H

#include <stdint.h>



typedef struct {
    uint32_t hall_timestamps_index;
    uint32_t hall_timestamps_last_index;
    uint32_t total_trigger_count;
    uint32_t last_total_trigger_count;
    uint64_t gptimer_last_isr_timestamp;
    uint64_t gptimer_last_update_timestamp;
    uint32_t hall_timestamps[64];
} hall_data_t;

#endif