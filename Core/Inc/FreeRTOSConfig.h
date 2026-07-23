/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application-specific definitions
 *----------------------------------------------------------*/

/*
 * 不要在這裡 include：
 *
 * #include "FreeRTOS.h"
 * #include "task.h"
 *
 * 因為 FreeRTOS.h 本身會 include FreeRTOSConfig.h，
 * 放在這裡可能產生循環包含。
 */

/* USER CODE BEGIN Includes */
/* Additional include files may be added here. */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, not assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)

#include <stdint.h>

extern uint32_t SystemCoreClock;
void xPortSysTickHandler(void);

#endif

#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f1xx.h"
#endif

/*-----------------------------------------------------------
 * Scheduler configuration
 *----------------------------------------------------------*/

/* 使用搶占式排程。 */
#define configUSE_PREEMPTION                     1

/* 支援靜態配置，例如 xTaskCreateStatic。 */
#define configSUPPORT_STATIC_ALLOCATION          1

/* 支援動態配置，例如 xTaskCreate。 */
#define configSUPPORT_DYNAMIC_ALLOCATION         1

/* 不使用 Idle Hook。 */
#define configUSE_IDLE_HOOK                      0

/* 不使用 Tick Hook。 */
#define configUSE_TICK_HOOK                      0

/* CPU 時脈。 */
#define configCPU_CLOCK_HZ                       (SystemCoreClock)

/* RTOS Tick 為 1000 Hz，即每 1 ms 一個 Tick。 */
#define configTICK_RATE_HZ                       ((TickType_t)1000)

/*
 * CMSIS-RTOS V2 使用較完整的 priority mapping，
 * 因此 CubeMX 通常設定成 56。
 */
#define configMAX_PRIORITIES                     56

/*
 * 最小 Task Stack 深度。
 *
 * Cortex-M3 的 StackType_t 是 4 Bytes：
 * 128 Words = 512 Bytes。
 */
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)

/*
 * FreeRTOS 動態記憶體池。
 *
 * 此空間會提供給：
 * - Task Stack
 * - Task Control Block
 * - Queue
 * - Mutex
 * - Semaphore
 * - Software Timer
 */
#define configTOTAL_HEAP_SIZE                    ((size_t)(16U * 1024U))

/* Task 名稱最大長度，包含字串結尾 '\0'。 */
#define configMAX_TASK_NAME_LEN                  16

/* 啟用 Trace 功能。 */
#define configUSE_TRACE_FACILITY                 1

/* 使用 32-bit TickType_t。 */
#define configUSE_16_BIT_TICKS                   0

/* 啟用 Mutex。 */
#define configUSE_MUTEXES                        1

/* Queue Registry 最大登記數量。 */
#define configQUEUE_REGISTRY_SIZE                8

/* 啟用 Recursive Mutex。 */
#define configUSE_RECURSIVE_MUTEXES              1

/* 啟用 Counting Semaphore。 */
#define configUSE_COUNTING_SEMAPHORES            1

/*
 * Cortex-M3 不使用最佳化的 Task Priority Selection。
 * 設為 0 沒問題。
 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0

/*-----------------------------------------------------------
 * Co-routine configuration
 *----------------------------------------------------------*/

#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          2

/*-----------------------------------------------------------
 * Software timer configuration
 *----------------------------------------------------------*/

#define configUSE_TIMERS                         1

/*
 * Timer Service Task 優先級。
 * 數字越大，FreeRTOS 原生優先級越高。
 */
#define configTIMER_TASK_PRIORITY                2

/* Timer command queue 長度。 */
#define configTIMER_QUEUE_LENGTH                 10

/*
 * Timer Task Stack：
 * 256 Words × 4 Bytes = 1024 Bytes。
 */
#define configTIMER_TASK_STACK_DEPTH             256

/*-----------------------------------------------------------
 * Optional API functions
 *----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_xQueueGetMutexHolder             1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_eTaskGetState                    1

/*-----------------------------------------------------------
 * FreeRTOS heap implementation
 *----------------------------------------------------------*/

/*
 * 使用 heap_4.c。
 *
 * heap_4 支援：
 * - pvPortMalloc()
 * - vPortFree()
 * - 合併相鄰的空閒記憶體區塊
 */
#define USE_FreeRTOS_HEAP_4

/*-----------------------------------------------------------
 * Cortex-M interrupt priority configuration
 *----------------------------------------------------------*/

#ifdef __NVIC_PRIO_BITS

/*
 * CMSIS 已經由 MCU 標頭定義 NVIC priority bits。
 */
#define configPRIO_BITS                          __NVIC_PRIO_BITS

#else

/*
 * STM32F103 使用 4-bit interrupt priority。
 */
#define configPRIO_BITS                          4

#endif

/*
 * Cortex-M 中斷優先級：
 *
 * 數字越小，硬體中斷優先級越高。
 * 數字越大，硬體中斷優先級越低。
 */

/* Kernel 使用最低硬體中斷優先級。 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15

/*
 * 可以呼叫 FreeRTOS FromISR API 的最高中斷優先級。
 *
 * 設為 5 表示：
 * - priority 0～4：不能呼叫 FreeRTOS API
 * - priority 5～15：可以呼叫 FromISR API
 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

/*
 * 轉換成 Cortex-M 暫存器實際使用的優先級格式。
 */
#define configKERNEL_INTERRUPT_PRIORITY                          \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY <<                  \
    (8U - configPRIO_BITS))

/*
 * 注意：configMAX_SYSCALL_INTERRUPT_PRIORITY 不得為 0。
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY                     \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY <<             \
    (8U - configPRIO_BITS))

/*-----------------------------------------------------------
 * Assert configuration
 *----------------------------------------------------------*/

/*
 * Assert 發生時：
 * 1. 關閉中斷
 * 2. 停在無限迴圈
 *
 * 除錯時可在 for (;;) 內設中斷點。
 */
#define configASSERT(x)                                          \
    do                                                           \
    {                                                            \
        if ((x) == 0)                                            \
        {                                                        \
            taskDISABLE_INTERRUPTS();                            \
            for (;;)                                             \
            {                                                    \
            }                                                    \
        }                                                        \
    } while (0)

/*-----------------------------------------------------------
 * Cortex-M exception handler mapping
 *----------------------------------------------------------*/

#define vPortSVCHandler                      SVC_Handler
#define xPortPendSVHandler                   PendSV_Handler

/*
 * SysTick_Handler 由專案自行實作。
 *
 * 必須確認 stm32f1xx_it.c 裡的 SysTick_Handler
 * 有正確呼叫 xPortSysTickHandler()。
 */
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION  0

/* USER CODE BEGIN Defines */
/* Additional definitions may be added here. */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
