#ifndef INIT_GPTIMER_H
#define INIT_GPTIMER_H

#include "driver/gptimer.h"

gptimer_handle_t init_gptimer_200hz(bool *isr_flag);

#endif // INIT_GPTIMER_H