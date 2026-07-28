#ifndef __DEBUG_CONSOLE_H
#define __DEBUG_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>


/*
 * Initialize the debug console module.
 *
 * This function may be called before the RTOS kernel
 * is initialized.
 */
void DebugConsole_Init( UART_HandleTypeDef *uart );


/*
 * Create the UART transmission mutex.
 *
 * This function must be called after osKernelInitialize().
 *
 * Return value:
 *
 *     1: Mutex created successfully.
 *     0: Mutex creation failed.
 */
uint8_t DebugConsole_CreateMutex(void);


/*
 * Send binary data through the debug UART.
 *
 * Before the scheduler starts, data is transmitted
 * without using the mutex.
 *
 * After the scheduler starts, the UART mutex protects
 * the transmission operation.
 */
HAL_StatusTypeDef DebugConsole_Write( const uint8_t *data, uint16_t length, uint32_t timeout );


/*
 * Send a null-terminated text string.
 */
void DebugConsole_WriteText( const char *text );


#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_CONSOLE_H */
