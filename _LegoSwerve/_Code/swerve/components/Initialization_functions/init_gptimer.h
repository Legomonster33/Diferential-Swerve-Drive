#ifndef INIT_GPTIMER_H
#define INIT_GPTIMER_H

#include "driver/gptimer.h"

/**
 * @brief Initialize a GPTimer for 200Hz operation.
 *
 * @param isr_flag Pointer to a flag that will be set in the ISR.
 * @return gptimer_handle_t Handle to the initialized GPTimer.
 */
gptimer_handle_t init_gptimer_200hz(bool *isr_flag);

#endif // INIT_GPTIMER_H