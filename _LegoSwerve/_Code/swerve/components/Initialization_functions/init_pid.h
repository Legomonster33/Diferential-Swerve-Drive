#ifndef INIT_PID_H
#define INIT_PID_H

#include "pid_ctrl.h"
#include "motor_config.h"

/**
 * @brief Initialize a PID control block.
 *
 * @param pid_handle Pointer to the PID control block handle to be initialized.
 * @param motor_config Pointer to the motor configuration structure.
 */
void init_pid(pid_ctrl_block_handle_t *pid_handle, const MotorConfig *motor_config);

#endif // INIT_PID_H
