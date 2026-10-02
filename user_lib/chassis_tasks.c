#include "chassis_tasks.h"

#include "cmsis_os2.h"

#include <stddef.h>
#include <stdio.h>

#include "communication.h"
#include "debug_console.h"
#include "encoder.h"
#include "motion_control.h"
#include "chassis_config.h"
#include "mecanum_kinematics.h"
#include "power_monitor.h"


#define CHASSIS_DEBUG_PRINT_FLAG      (1UL << 0)


/*
 * CMSIS-RTOS V2 objects owned by this module.
 */
static osThreadId_t chassis_motor_task_handle = NULL;
static osThreadId_t chassis_debug_task_handle = NULL;




/*
 * CMSIS-RTOS V2 stack_size uses bytes, not FreeRTOS words.
 */
static const osThreadAttr_t chassis_motor_task_attributes =
{
    .name = "run_motor",
    .stack_size = CHASSIS_MOTOR_TASK_STACK_SIZE,
    .priority = osPriorityAboveNormal
};

static const osThreadAttr_t chassis_debug_task_attributes =
{
    .name = "uart_debug",
    .stack_size = CHASSIS_DEBUG_TASK_STACK_SIZE,
    .priority = osPriorityLow
};


static void ChassisTasks_MotorTask(void *argument);
static void ChassisTasks_DebugTask(void *argument);
static void ChassisTasks_ReportCompletion(void);

/*
 * Indicates whether communication and task
 * dependencies have been initialized.
 */
static uint8_t chassis_tasks_initialized = 0U;

/**
  * @brief Create the command queue and initialize communication.
  */
/**
 * @brief Initialize chassis task dependencies.
 */
ChassisTasks_Status_t ChassisTasks_Init( UART_HandleTypeDef *command_uart, ChassisTasks_LogFunction_t log_function )
{
    Communication_InitStatus_t communication_status;

    if (command_uart == NULL)
    {
        return CHASSIS_TASKS_STATUS_INVALID_ARGUMENT;
    }

    chassis_motor_task_handle = NULL;
    chassis_debug_task_handle = NULL;
    chassis_tasks_initialized = 0U;

    communication_status = Communication_Init( command_uart, log_function );

    if (communication_status != COMMUNICATION_INIT_OK)
    {
        return CHASSIS_TASKS_STATUS_COMMUNICATION_ERROR;
    }

    chassis_tasks_initialized = 1U;

    return CHASSIS_TASKS_STATUS_OK;
}


/**
  * @brief Create the periodic motor control task.
  */
ChassisTasks_Status_t ChassisTasks_CreateMotorTask(void)
{
	if (chassis_tasks_initialized == 0U)
	{
	    return CHASSIS_TASKS_STATUS_COMMUNICATION_ERROR;
	}

    if (chassis_motor_task_handle != NULL)
    {
        return CHASSIS_TASKS_STATUS_OK;
    }

    chassis_motor_task_handle = osThreadNew( ChassisTasks_MotorTask, NULL, &chassis_motor_task_attributes );

    if (chassis_motor_task_handle == NULL)
    {
        return CHASSIS_TASKS_STATUS_MOTOR_TASK_ERROR;
    }

    return CHASSIS_TASKS_STATUS_OK;
}


/**
  * @brief Create the motion completion debug task.
  */
ChassisTasks_Status_t ChassisTasks_CreateDebugTask(void)
{
	if (chassis_tasks_initialized == 0U)
	{
	    return CHASSIS_TASKS_STATUS_COMMUNICATION_ERROR;
	}
    if (chassis_debug_task_handle != NULL)
    {
        return CHASSIS_TASKS_STATUS_OK;
    }

    chassis_debug_task_handle = osThreadNew( ChassisTasks_DebugTask, NULL, &chassis_debug_task_attributes );

    if (chassis_debug_task_handle == NULL)
    {
        return CHASSIS_TASKS_STATUS_DEBUG_TASK_ERROR;
    }

    return CHASSIS_TASKS_STATUS_OK;
}


/**
 * @brief Report that the current movement has completed.
 */
static void ChassisTasks_ReportCompletion(void)
{
    char response[32];
    float battery_voltage;

    /*
     * Motor stopping is handled by MotionControl_Update().
     * UART access is handled by the communication module.
     */
    if (PowerMonitor_ReadVoltage( &battery_voltage ) == POWER_MONITOR_STATUS_OK)
    {
        (void)snprintf( response, sizeof(response), "1,%.2f\r\n", (double)battery_voltage );

        (void)Communication_SendText( response, 100U );
    }
    else
    {
        (void)Communication_SendText( "1,ERR\r\n", 100U );
    }

    if (chassis_debug_task_handle != NULL)
    {
        (void)osThreadFlagsSet( chassis_debug_task_handle, CHASSIS_DEBUG_PRINT_FLAG );
    }
}


/**
  * @brief Process commands and update motion control every 10 ms.
  */
static void ChassisTasks_MotorTask(void *argument)
{
    UART_Command_t command;
    uint32_t next_wake_tick;
    HAL_StatusTypeDef receive_status;

    (void)argument;

    receive_status = Communication_StartReceive();

    if (receive_status == HAL_OK)
    {
        DebugConsole_WriteText( "UART4 RX interrupt started\r\n" );
    }
    else if (receive_status == HAL_BUSY)
    {
        DebugConsole_WriteText( "UART4 RX interrupt busy\r\n" );
    }
    else
    {
        DebugConsole_WriteText( "UART4 RX interrupt error\r\n" );
        Error_Handler();
    }

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        Communication_CommandStatus_t command_status;
        MotionControl_UpdateResult_t motion_result;

        /*
         * Read one parsed command without blocking the control loop.
         */
        command_status = Communication_GetCommand( &command, 0U );

        if (command_status == COMMUNICATION_COMMAND_READY)
        {
            MotionControl_StartResult_t start_result;

            start_result = MotionControl_Start( command.direction, command.distance, CHASSIS_DEFAULT_SPEED_RPM );

            if (start_result == MOTION_CONTROL_START_INVALID_DIRECTION)
            {
                DebugConsole_WriteText( "Invalid direction\r\n" );
            }
            else if (start_result == MOTION_CONTROL_START_WHEEL_ERROR)
            {
                DebugConsole_WriteText( "Wheel direction error\r\n" );
            }
        }
        else if (command_status == COMMUNICATION_COMMAND_PARSE_ERROR)
        {
            DebugConsole_WriteText( "UART4 parse error\r\n" );
        }
        else if (command_status == COMMUNICATION_COMMAND_QUEUE_ERROR)
        {
            DebugConsole_WriteText( "UART4 queue error\r\n" );
        }

        /*
         * Update encoder data, completion detection, and wheel PID control.
         */
        motion_result = MotionControl_Update( CHASSIS_CONTROL_DT_S );

        if (motion_result == MOTION_CONTROL_UPDATE_COMPLETED)
        {
            ChassisTasks_ReportCompletion();
        }

        /*
         * Keep the control period close to the tested 10 ms interval.
         */
        next_wake_tick += CHASSIS_CONTROL_PERIOD_MS;

        if (osDelayUntil(next_wake_tick) != osOK)
        {
            next_wake_tick = osKernelGetTickCount();
        }
    }
}


/**
  * @brief Print final encoder distances after a movement completes.
  */
static void ChassisTasks_DebugTask(void *argument)
{
	char transmit_message[ CHASSIS_DEBUG_MESSAGE_SIZE ];
	float wheel_distance_cm[ MECANUM_WHEEL_COUNT ];
    MotionControl_Info_t motion_info;

    (void)argument;

    for (;;)
    {
        uint32_t flags;
        int message_length;

        flags = osThreadFlagsWait( CHASSIS_DEBUG_PRINT_FLAG, osFlagsWaitAny, osWaitForever );

        if ((flags & CHASSIS_DEBUG_PRINT_FLAG) != 0U)
        {
        	for (uint32_t i = 0U; i < MECANUM_WHEEL_COUNT; i++)
            {
                wheel_distance_cm[i] = Encoder_Get_Distance_cm( (uint8_t)(i + 1U) );
            }

            MotionControl_GetInfo(&motion_info);

            message_length = snprintf(
                transmit_message,
                sizeof(transmit_message),
                "M1:%.2f,M2:%.2f,M3:%.2f,M4:%.2f,"
                "dir:%d,target:%.2f\r\n",
                (double)wheel_distance_cm[0],
                (double)wheel_distance_cm[1],
                (double)wheel_distance_cm[2],
                (double)wheel_distance_cm[3],
                motion_info.direction,
                (double)motion_info.target_value
            );

            if (message_length > 0)
            {
                size_t transmit_length = (size_t)message_length;

                if (transmit_length >= sizeof(transmit_message))
                {
                    transmit_length = sizeof(transmit_message) - 1U;
                }

                (void)DebugConsole_Write( (const uint8_t *)transmit_message, (uint16_t)transmit_length, 500U );
            }
        }
    }
}
