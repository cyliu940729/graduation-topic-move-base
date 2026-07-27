#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os2.h"


/*
 * Maximum UART command frame length.
 *
 * Current frame format:
 *
 *     (direction,distance)
 *
 * Examples:
 *
 *     (1,30)
 *     (5,90)
 *     (7,0)
 */
#define COMMUNICATION_RX_BUFFER_SIZE    32U


/*
 * Parsed UART command data.
 */
typedef struct
{
    int direction;
    float distance;
} UART_Command_t;


/*
 * Result returned by Communication_GetCommand().
 */
typedef enum
{
    COMMUNICATION_COMMAND_NONE = 0,
    COMMUNICATION_COMMAND_READY,
    COMMUNICATION_COMMAND_PARSE_ERROR,
    COMMUNICATION_COMMAND_QUEUE_ERROR
} Communication_CommandStatus_t;

/*
 * Communication initialization result.
 */
typedef enum
{
    COMMUNICATION_INIT_OK = 0,
    COMMUNICATION_INIT_INVALID_ARGUMENT,
    COMMUNICATION_INIT_QUEUE_ERROR
} Communication_InitStatus_t;

/*
 * Optional logging function used by the communication module.
 *
 * The function receives a null-terminated text string.
 */
typedef void (*Communication_LogFunction_t)(const char *text);


/*
 * Initialize UART communication and create
 * the internal receive message queue.
 *
 * This function must be called after
 * osKernelInitialize().
 */
Communication_InitStatus_t Communication_Init(
    UART_HandleTypeDef *uart,
    Communication_LogFunction_t log_function
);


/*
 * Start UART interrupt reception.
 *
 * The UART receives one byte at a time.
 */
HAL_StatusTypeDef Communication_StartReceive(void);

/*
 * Send a null-terminated text response through
 * the command UART.
 */
HAL_StatusTypeDef Communication_SendText(
    const char *text,
    uint32_t timeout
);

/*
 * Read and parse one command from the internal frame queue.
 *
 * timeout:
 *     CMSIS-RTOS queue wait timeout in kernel ticks.
 *
 * Return values:
 *
 *     COMMUNICATION_COMMAND_NONE
 *         No command is currently available.
 *
 *     COMMUNICATION_COMMAND_READY
 *         A valid command was received and written to command.
 *
 *     COMMUNICATION_COMMAND_PARSE_ERROR
 *         A complete frame was received but its format was invalid.
 *
 *     COMMUNICATION_COMMAND_QUEUE_ERROR
 *         The module is not initialized or the queue operation failed.
 */
Communication_CommandStatus_t Communication_GetCommand(
    UART_Command_t *command,
    uint32_t timeout
);



#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H */
