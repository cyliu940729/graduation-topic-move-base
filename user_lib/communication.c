#include "communication.h"
#include "chassis_config.h"

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


/*
 * UART handle used by the communication module.
 */
static UART_HandleTypeDef *communication_uart = NULL;


/*
 * Queue used to transfer complete frames
 * from interrupt context to task context.
 */
static osMessageQueueId_t communication_command_queue = NULL;


/*
 * Optional text logging function.
 */
static Communication_LogFunction_t communication_log_function = NULL;


/*
 * Single-byte UART receive buffer.
 */
static uint8_t communication_rx_char = 0U;


/*
 * Buffer used to assemble a complete UART frame.
 */
static char communication_rx_buffer[COMMUNICATION_RX_BUFFER_SIZE];


/*
 * Current write position in communication_rx_buffer.
 */
static volatile uint8_t communication_rx_index = 0U;


/*
 * Frame reception state.
 *
 * 0:
 *     Waiting for '('.
 *
 * 1:
 *     Receiving a frame and waiting for ')'.
 */
static volatile uint8_t communication_receiving = 0U;


/*
 * Parse a complete frame into a command structure.
 */
static uint8_t Communication_ParseFrame(
    const char *frame,
    UART_Command_t *command
)
{
    if ((frame == NULL) || (command == NULL))
    {
        return 0U;
    }

    /*
     * Keep the current UART command format unchanged.
     */
    if (sscanf(frame,
               "(%d,%f)",
               &command->direction,
               &command->distance) == 2)
    {
        return 1U;
    }

    return 0U;
}


/**
 * @brief Initialize the communication module.
 */
Communication_InitStatus_t Communication_Init(
    UART_HandleTypeDef *uart,
    Communication_LogFunction_t log_function
)
{
    if (uart == NULL)
    {
        return COMMUNICATION_INIT_INVALID_ARGUMENT;
    }

    communication_uart = uart;
    communication_log_function = log_function;

    communication_rx_char = 0U;
    communication_rx_index = 0U;
    communication_receiving = 0U;
    communication_rx_buffer[0] = '\0';

    /*
     * The receive queue is an internal implementation
     * detail of the communication module.
     */
    communication_command_queue = osMessageQueueNew(
        CHASSIS_COMMAND_QUEUE_SIZE,
        COMMUNICATION_RX_BUFFER_SIZE,
        NULL
    );

    if (communication_command_queue == NULL)
    {
        communication_uart = NULL;
        communication_log_function = NULL;

        return COMMUNICATION_INIT_QUEUE_ERROR;
    }

    return COMMUNICATION_INIT_OK;
}


/**
 * @brief Start UART interrupt reception.
 */
HAL_StatusTypeDef Communication_StartReceive(void)
{
    if (communication_uart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(
        communication_uart,
        &communication_rx_char,
        1U
    );
}

/**
 * @brief Send a text response through the command UART.
 */
HAL_StatusTypeDef Communication_SendText(
    const char *text,
    uint32_t timeout
)
{
    size_t length;

    if ((communication_uart == NULL) ||
        (text == NULL))
    {
        return HAL_ERROR;
    }

    length = strlen(text);

    if (length == 0U)
    {
        return HAL_OK;
    }

    if (length > UINT16_MAX)
    {
        length = UINT16_MAX;
    }

    return HAL_UART_Transmit(
        communication_uart,
        (uint8_t *)text,
        (uint16_t)length,
        timeout
    );
}

/**
 * @brief Read and parse one command from the frame queue.
 */
Communication_CommandStatus_t Communication_GetCommand(
    UART_Command_t *command,
    uint32_t timeout
)
{
    char frame[COMMUNICATION_RX_BUFFER_SIZE];
    osStatus_t queue_status;

    if ((command == NULL) ||
        (communication_command_queue == NULL))
    {
        return COMMUNICATION_COMMAND_QUEUE_ERROR;
    }

    queue_status = osMessageQueueGet(
        communication_command_queue,
        frame,
        NULL,
        timeout
    );

    /*
     * No frame is currently available.
     */
    if ((queue_status == osErrorResource) ||
        (queue_status == osErrorTimeout))
    {
        return COMMUNICATION_COMMAND_NONE;
    }

    /*
     * An unexpected queue error occurred.
     */
    if (queue_status != osOK)
    {
        return COMMUNICATION_COMMAND_QUEUE_ERROR;
    }

    /*
     * Force null termination even if the queued frame
     * was malformed or did not contain a valid terminator.
     */
    frame[COMMUNICATION_RX_BUFFER_SIZE - 1U] = '\0';

    /*
     * Preserve the existing UART receive debug output.
     */
    if (communication_log_function != NULL)
    {
        char log_message[64];
        int log_length;

        log_length = snprintf(
            log_message,
            sizeof(log_message),
            "UART4 RX: %s\r\n",
            frame
        );

        if (log_length > 0)
        {
            communication_log_function(log_message);
        }
    }

    if (Communication_ParseFrame(frame, command) == 1U)
    {
        return COMMUNICATION_COMMAND_READY;
    }

    return COMMUNICATION_COMMAND_PARSE_ERROR;
}


/**
 * @brief UART receive complete callback.
 *
 * This callback receives one byte at a time and assembles
 * a complete frame between '(' and ')'.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((communication_uart != NULL) &&
        (huart == communication_uart))
    {
        char received_char = (char)communication_rx_char;

        /*
         * Start a new frame when '(' is received.
         *
         * A new '(' discards any incomplete previous frame.
         */
        if (received_char == '(')
        {
            communication_rx_index = 0U;
            communication_receiving = 1U;

            communication_rx_buffer[
                communication_rx_index++
            ] = received_char;
        }
        else if (communication_receiving == 1U)
        {
            /*
             * Reserve the last buffer position for '\0'.
             */
            if (communication_rx_index <
                (COMMUNICATION_RX_BUFFER_SIZE - 1U))
            {
                communication_rx_buffer[
                    communication_rx_index++
                ] = received_char;
            }

            /*
             * Complete the frame when ')' is received.
             */
            if (received_char == ')')
            {
                communication_rx_buffer[
                    communication_rx_index
                ] = '\0';

                communication_receiving = 0U;

                /*
                 * Timeout must be zero when called
                 * from interrupt context.
                 */
                if (communication_command_queue != NULL)
                {
                    (void)osMessageQueuePut(
                        communication_command_queue,
                        communication_rx_buffer,
                        0U,
                        0U
                    );
                }

                communication_rx_index = 0U;
            }
        }

        /*
         * Restart reception for the next byte.
         */
        (void)HAL_UART_Receive_IT(
            communication_uart,
            &communication_rx_char,
            1U
        );
    }
}


/**
 * @brief UART error callback.
 *
 * Clear the overrun condition, discard the incomplete
 * frame, and restart reception.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((communication_uart != NULL) &&
        (huart == communication_uart))
    {
        __HAL_UART_CLEAR_OREFLAG(huart);

        communication_rx_index = 0U;
        communication_receiving = 0U;
        communication_rx_buffer[0] = '\0';

        (void)HAL_UART_Receive_IT(
            communication_uart,
            &communication_rx_char,
            1U
        );
    }
}
