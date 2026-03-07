#ifndef HALL_DATA_H
#define HALL_DATA_H

#include <stdint.h>



typedef struct {
    uint32_t hall_timestamps[64];
    uint32_t hall_timestamps_index;
    uint32_t hall_timestamps_last_index;
    uint32_t total_trigger_count;
    uint32_t last_total_trigger_count;
    uint32_t difference_total_triggers_last_update;
    uint64_t gptimer_last_isr_timestamp;
    uint64_t gptimer_last_update_timestamp;
    enum {
        MODE_SINGLE_PERIOD,
        MODE_MULTIPLE_PERIOD,
    }mode;
} hall_data_t;

#endif