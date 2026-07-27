#ifndef __CHASSIS_TASKS_H
#define __CHASSIS_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>


/*
 * Optional text logging function used during communication initialization.
 */
typedef void (*ChassisTasks_LogFunction_t)(const char *text);


/*
 * Chassis task object creation result.
 */
typedef enum
{
    CHASSIS_TASKS_STATUS_OK = 0,
    CHASSIS_TASKS_STATUS_INVALID_ARGUMENT,
	CHASSIS_TASKS_STATUS_COMMUNICATION_ERROR,
    CHASSIS_TASKS_STATUS_MOTOR_TASK_ERROR,
    CHASSIS_TASKS_STATUS_DEBUG_TASK_ERROR
} ChassisTasks_Status_t;


/*
 * Create the UART command queue and initialize
 * the communication module.
 *
 * This function must be called after osKernelInitialize().
 */
ChassisTasks_Status_t ChassisTasks_Init(
    UART_HandleTypeDef *command_uart,
    ChassisTasks_LogFunction_t log_function
);


/*
 * Create the periodic motor control task.
 */
ChassisTasks_Status_t ChassisTasks_CreateMotorTask(void);


/*
 * Create the motion completion debug task.
 */
ChassisTasks_Status_t ChassisTasks_CreateDebugTask(void);


#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_TASKS_H */
