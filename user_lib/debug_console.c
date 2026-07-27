#include "debug_console.h"

#include "cmsis_os2.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>


#define DEBUG_CONSOLE_MUTEX_TIMEOUT_MS    200U
#define DEBUG_CONSOLE_TEXT_TIMEOUT_MS     500U


/*
 * UART handle used for debug output.
 */
static UART_HandleTypeDef *debug_console_uart = NULL;


/*
 * Mutex used to protect UART transmission
 * after the scheduler starts.
 */
static osMutexId_t debug_console_mutex = NULL;


/**
 * @brief Initialize the debug console module.
 */
void DebugConsole_Init(
    UART_HandleTypeDef *uart
)
{
    debug_console_uart = uart;
    debug_console_mutex = NULL;
}


/**
 * @brief Create the UART transmission mutex.
 */
uint8_t DebugConsole_CreateMutex(void)
{
    if (debug_console_uart == NULL)
    {
        return 0U;
    }

    /*
     * Do not create the mutex twice.
     */
    if (debug_console_mutex != NULL)
    {
        return 1U;
    }

    debug_console_mutex = osMutexNew(NULL);

    if (debug_console_mutex == NULL)
    {
        return 0U;
    }

    return 1U;
}


/**
 * @brief Send data through the debug UART.
 */
HAL_StatusTypeDef DebugConsole_Write(
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout
)
{
    HAL_StatusTypeDef uart_status;
    uint8_t mutex_locked = 0U;

    if ((debug_console_uart == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return HAL_ERROR;
    }

    /*
     * Use the mutex only after the scheduler starts.
     *
     * During boot, the kernel is not running yet,
     * so UART transmission is performed directly.
     */
    if ((debug_console_mutex != NULL) &&
        (osKernelGetState() == osKernelRunning))
    {
        if (osMutexAcquire(
                debug_console_mutex,
                DEBUG_CONSOLE_MUTEX_TIMEOUT_MS
            ) != osOK)
        {
            return HAL_BUSY;
        }

        mutex_locked = 1U;
    }

    uart_status = HAL_UART_Transmit(
        debug_console_uart,
        (uint8_t *)data,
        length,
        timeout
    );

    if (mutex_locked == 1U)
    {
        (void)osMutexRelease(
            debug_console_mutex
        );
    }

    return uart_status;
}


/**
 * @brief Send a null-terminated text string.
 */
void DebugConsole_WriteText(
    const char *text
)
{
    size_t length;

    if (text == NULL)
    {
        return;
    }

    length = strlen(text);

    if (length == 0U)
    {
        return;
    }

    if (length > UINT16_MAX)
    {
        length = UINT16_MAX;
    }

    (void)DebugConsole_Write(
        (const uint8_t *)text,
        (uint16_t)length,
        DEBUG_CONSOLE_TEXT_TIMEOUT_MS
    );
}
