/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - CMSIS-RTOS V2
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "motor.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    int direction;
    float distance;
} UART_Command_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART4_RX_BUFFER_SIZE      32U
#define UART_COMMAND_QUEUE_SIZE    8U
#define MOTOR_TASK_PERIOD_MS      10U
#define MOTOR_CONTROL_DT_S         0.01f
#define DRIVE_SPEED_RPM           30.0f
#define DEBUG_PRINT_FLAG          (1UL << 0)
#define UART5_TX_MUTEX_TIMEOUT_MS  200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* CMSIS-RTOS V2 objects */
static osThreadId_t motor_run_handle = NULL;
static osThreadId_t uart_debug_handle = NULL;
static osMessageQueueId_t uart_command_queue = NULL;
static osMutexId_t uart5_tx_mutex = NULL;

/*
 * CMSIS-RTOS V2 stack_size uses bytes, not FreeRTOS words.
 * UART debug task gets a larger stack because snprintf with float is expensive.
 */
static const osThreadAttr_t motor_run_attributes =
{
    .name = "run_motor",
    .stack_size = 2048U,
    .priority = osPriorityAboveNormal
};

static const osThreadAttr_t uart_debug_attributes =
{
    .name = "uart_debug",
    .stack_size = 1536U,
    .priority = osPriorityLow
};

/* UART4 interrupt receive state */
static uint8_t uart4_rx_char = 0U;
static char uart4_rx_buf[UART4_RX_BUFFER_SIZE];
static volatile uint8_t uart4_rx_idx = 0U;
static volatile uint8_t uart4_receiving = 0U;

/* Current vehicle command/state */
static volatile uint8_t motor_run_flag = 0U;
static volatile uint8_t motor_hw_ready = 0U;
static volatile int dir = 0;
static volatile float dist = 0.0f;

static float cm = 0.0f;
static float car_angle = 0.0f;

/* Signed target RPM for M1~M4. Updated when a new UART command arrives. */
static float motor_target_rpm[4] =
{
    0.0f, 0.0f, 0.0f, 0.0f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void Run_Motor_Task(void *argument);
static void UartDebugTask(void *argument);

static void Motor_Start_Command(const UART_Command_t *command);
static void Motor_Stop_And_Report(void);
static void Motor_Clear_Targets(void);
static float Get_Average_Abs_Wheel_Distance_cm(void);
static HAL_StatusTypeDef Uart5_Send(const uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout);
static void Uart5_SendText(const char *text);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef Uart5_Send(const uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout)
{
    HAL_StatusTypeDef status;
    uint8_t mutex_locked = 0U;

    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    /*
     * Scheduler 啟動後，所有 Task 共用 UART5 mutex。
     * UART ISR 不再呼叫 UART5，因此不會發生 ISR/Task 同時傳送。
     */
    if ((uart5_tx_mutex != NULL) &&
        (osKernelGetState() == osKernelRunning))
    {
        if (osMutexAcquire(uart5_tx_mutex,
                           UART5_TX_MUTEX_TIMEOUT_MS) != osOK)
        {
            return HAL_BUSY;
        }

        mutex_locked = 1U;
    }

    status = HAL_UART_Transmit(&huart5,
                               (uint8_t *)data,
                               length,
                               timeout);

    if (mutex_locked == 1U)
    {
        (void)osMutexRelease(uart5_tx_mutex);
    }

    return status;
}

static void Uart5_SendText(const char *text)
{
    if (text != NULL)
    {
        size_t length = strlen(text);

        if (length > UINT16_MAX)
        {
            length = UINT16_MAX;
        }

        (void)Uart5_Send((const uint8_t *)text,
                         (uint16_t)length,
                         500U);
    }
}

static void Boot_Debug(const char *text)
{
    /* Boot 階段 scheduler 尚未啟動，直接以阻塞方式輸出。 */
    if (text != NULL)
    {
        (void)HAL_UART_Transmit(&huart5,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                500U);
    }
}

static void Motor_Clear_Targets(void)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        motor_target_rpm[i] = 0.0f;
    }
}

static float Get_Average_Abs_Wheel_Distance_cm(void)
{
    float distance_sum = 0.0f;

    for (uint8_t i = 1U; i <= 4U; i++)
    {
        distance_sum += fabsf(Encoder_Get_Distance_cm(i));
    }

    return distance_sum / 4.0f;
}
/**
  * @brief Load a new vehicle command and prepare four target RPM values.
  * @note  PID calculation is not done here. Run_Motor_Task executes PID every
  *        10 ms after Encoder_Update_All().
  */
static void Motor_Start_Command(const UART_Command_t *command)
{
    if (command == NULL)
    {
        return;
    }

    Motor_Stop_All();
    Motor_Clear_Targets();
    motor_run_flag = 0U;

    dir = command->direction;
    dist = fabsf(command->distance);

    if ((dir < 1) || (dir > 7))
    {
        Uart5_SendText("Invalid direction\r\n");
        return;
    }

    /* Direction 7 is an immediate stop command. */
    if (dir == 7)
    {
        return;
    }

    /* Reset distance, RPM filter, and encoder counters for this movement. */
    Encoder_Init();

    switch (dir)
    {
        case 1: /* Forward */
            motor_target_rpm[0] =  DRIVE_SPEED_RPM;
            motor_target_rpm[1] =  DRIVE_SPEED_RPM;
            motor_target_rpm[2] =  DRIVE_SPEED_RPM;
            motor_target_rpm[3] =  DRIVE_SPEED_RPM;
            break;

        case 2: /* Backward */
            motor_target_rpm[0] = -DRIVE_SPEED_RPM;
            motor_target_rpm[1] = -DRIVE_SPEED_RPM;
            motor_target_rpm[2] = -DRIVE_SPEED_RPM;
            motor_target_rpm[3] = -DRIVE_SPEED_RPM;
            break;

        case 4:
            motor_target_rpm[0] = -DRIVE_SPEED_RPM;
            motor_target_rpm[1] =  DRIVE_SPEED_RPM;
            motor_target_rpm[2] = -DRIVE_SPEED_RPM;
            motor_target_rpm[3] =  DRIVE_SPEED_RPM;
            break;

        case 3:
            motor_target_rpm[0] =  DRIVE_SPEED_RPM;
            motor_target_rpm[1] = -DRIVE_SPEED_RPM;
            motor_target_rpm[2] =  DRIVE_SPEED_RPM;
            motor_target_rpm[3] = -DRIVE_SPEED_RPM;
            break;

        case 6: /* Rotate */
            motor_target_rpm[0] = -DRIVE_SPEED_RPM;
            motor_target_rpm[1] =  DRIVE_SPEED_RPM;
            motor_target_rpm[2] =  DRIVE_SPEED_RPM;
            motor_target_rpm[3] = -DRIVE_SPEED_RPM;
            break;

        case 5: /* Rotate opposite direction */
            motor_target_rpm[0] =  DRIVE_SPEED_RPM;
            motor_target_rpm[1] = -DRIVE_SPEED_RPM;
            motor_target_rpm[2] = -DRIVE_SPEED_RPM;
            motor_target_rpm[3] =  DRIVE_SPEED_RPM;
            break;

        default:
            Motor_Stop_All();
            Motor_Clear_Targets();
            return;
    }

    motor_run_flag = 1U;
}

/**
  * @brief Stop all motors, report completion on UART4 and wake debug task.
  */
static void Motor_Stop_And_Report(void)
{
    static const uint8_t done_msg[] = "1\r\n";

    Motor_Stop_All();
    Motor_Clear_Targets();
    motor_run_flag = 0U;

    (void)HAL_UART_Transmit(&huart4,
                            (uint8_t *)done_msg,
                            sizeof(done_msg) - 1U,
                            100U);

    if (uart_debug_handle != NULL)
    {
        (void)osThreadFlagsSet(uart_debug_handle, DEBUG_PRINT_FLAG);
    }
}

/**
  * @brief Motor control task. Encoder and PID are both updated every 10 ms.
  */
static void Run_Motor_Task(void *argument)
{
    UART_Command_t command;
    char frame[UART4_RX_BUFFER_SIZE];
    uint32_t next_wake_tick;

    (void)argument;

    HAL_StatusTypeDef uart4_rx_status;

    uart4_rx_status = HAL_UART_Receive_IT(&huart4,
                                          &uart4_rx_char,
                                          1U);

    if (uart4_rx_status == HAL_OK)
    {
        static const uint8_t rx_ok_msg[] =
            "UART4 RX interrupt started\r\n";

        (void)Uart5_Send(rx_ok_msg,
                         sizeof(rx_ok_msg) - 1U,
                         500U);
    }
    else if (uart4_rx_status == HAL_BUSY)
    {
        static const uint8_t rx_busy_msg[] =
            "UART4 RX interrupt busy\r\n";

        (void)Uart5_Send(rx_busy_msg,
                         sizeof(rx_busy_msg) - 1U,
                         500U);
    }
    else
    {
        static const uint8_t rx_error_msg[] =
            "UART4 RX interrupt error\r\n";

        (void)Uart5_Send(rx_error_msg,
                         sizeof(rx_error_msg) - 1U,
                         500U);

        Error_Handler();
    }

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        if (osMessageQueueGet(uart_command_queue,
                              frame,
                              NULL,
                              0U) == osOK)
        {
            char rx_message[64];
            int rx_length;

            rx_length = snprintf(rx_message,
                                 sizeof(rx_message),
                                 "UART4 RX: %s\r\n",
                                 frame);

            if (rx_length > 0)
            {
                uint16_t send_length =
                    (rx_length < (int)sizeof(rx_message))
                    ? (uint16_t)rx_length
                    : (uint16_t)(sizeof(rx_message) - 1U);

                (void)Uart5_Send((const uint8_t *)rx_message,
                                 send_length,
                                 500U);
            }

            if (sscanf(frame,
                       "(%d,%f)",
                       &command.direction,
                       &command.distance) == 2)
            {
                Motor_Start_Command(&command);
            }
            else
            {
                Uart5_SendText("UART4 parse error\r\n");
            }
        }

        /* This is the only place that updates encoder state. */
        Encoder_Update_All();

        if (motor_run_flag == 1U)
        {
            uint8_t movement_finished = 0U;

            /* Average all four wheels instead of using only M1. */
            cm = Get_Average_Abs_Wheel_Distance_cm();
            car_angle = CarAngle_From_WheelDistance(cm);

            if ((dir >= 1) && (dir <= 4))
            {
                if (cm >= dist)
                {
                    movement_finished = 1U;
                }
            }
            else if ((dir == 5) || (dir == 6))
            {
                if (car_angle >= dist)
                {
                    movement_finished = 1U;
                }
            }

            if (movement_finished == 1U)
            {
                Motor_Stop_And_Report();
            }
            else
            {
                /* Closed-loop control must run continuously, not only once
                 * when the command is received. */
                for (uint8_t i = 0U; i < 4U; i++)
                {
                    (void)Motor_Speed_PID((uint8_t)(i + 1U),
                                          motor_target_rpm[i],
                                          MOTOR_CONTROL_DT_S);
                }
            }
        }

        /* Keep the encoder/PID sample period close to the tested 10 ms. */
        next_wake_tick += MOTOR_TASK_PERIOD_MS;

        if (osDelayUntil(next_wake_tick) != osOK)
        {
            next_wake_tick = osKernelGetTickCount();
        }
    }
}

/**
  * @brief Print the final four encoder distances after an action finishes.
  */
static void UartDebugTask(void *argument)
{
    char tx_msg[160];
    float all_cm[4];

    (void)argument;

    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(DEBUG_PRINT_FLAG,
                                           osFlagsWaitAny,
                                           osWaitForever);

        if ((flags & DEBUG_PRINT_FLAG) != 0U)
        {
            for (uint32_t i = 0U; i < 4U; i++)
            {
                all_cm[i] = Encoder_Get_Distance_cm((int)i + 1);
            }

            int length = snprintf(tx_msg,
                                  sizeof(tx_msg),
                                  "M1:%.2f,M2:%.2f,M3:%.2f,M4:%.2f,dir:%d,target:%.2f\r\n",
                                  all_cm[0],
                                  all_cm[1],
                                  all_cm[2],
                                  all_cm[3],
                                  dir,
                                  (double)dist);

            if (length > 0)
            {
                size_t tx_length = (size_t)length;

                if (tx_length >= sizeof(tx_msg))
                {
                    tx_length = sizeof(tx_msg) - 1U;
                }

                (void)Uart5_Send((const uint8_t *)tx_msg,
                                 (uint16_t)tx_length,
                                 500U);
            }
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration------------------------------------------------------*/

    HAL_Init();

    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_UART4_Init();
    MX_TIM1_Init();
    MX_UART5_Init();
    MX_I2C2_Init();
    MX_TIM8_Init();

    /*
     * UART4 ISR calls CMSIS-RTOS V2 API, therefore use an RTOS-safe
     * interrupt priority. This should match FreeRTOSConfig.h.
     */
    HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    /* USER CODE BEGIN 2 */
    Encoder_Init();
    Motor_Init();
    motor_hw_ready = 1U;
    Motor_Stop_All();

    {
        static const uint8_t init_msg[] = "Init OK\r\n";

        (void)HAL_UART_Transmit(&huart5,
                                (uint8_t *)init_msg,
                                sizeof(init_msg) - 1U,
                                500U);
    }



    /* USER CODE END 2 */

    /* Initialize CMSIS-RTOS V2 kernel */
    Boot_Debug("RTOS 1: kernel init\r\n");

    if (osKernelInitialize() != osOK)
    {
        Boot_Debug("ERROR: osKernelInitialize\r\n");
        Error_Handler();
    }

    Boot_Debug("RTOS 2: kernel init OK\r\n");

    /*
     * 暫時不要建立 CubeMX 的 defaultTask。
     * 如果未來要使用 freertos.c 內的物件，再打開。
     */
    // MX_FREERTOS_Init();

    Boot_Debug("RTOS 3: create queue\r\n");

    uart_command_queue = osMessageQueueNew(
        UART_COMMAND_QUEUE_SIZE,
        UART4_RX_BUFFER_SIZE,
        NULL
    );

    if (uart_command_queue == NULL)
    {
        Boot_Debug("ERROR: queue create failed\r\n");
        Error_Handler();
    }

    Boot_Debug("RTOS 4: queue OK\r\n");

    Boot_Debug("RTOS 5: create UART5 mutex\r\n");

    uart5_tx_mutex = osMutexNew(NULL);

    if (uart5_tx_mutex == NULL)
    {
        Boot_Debug("ERROR: UART5 mutex create failed\r\n");
        Error_Handler();
    }

    Boot_Debug("RTOS 6: UART5 mutex OK\r\n");

    motor_run_handle = osThreadNew(
        Run_Motor_Task,
        NULL,
        &motor_run_attributes
    );

    if (motor_run_handle == NULL)
    {
        Boot_Debug("ERROR: motor task create failed\r\n");
        Error_Handler();
    }

    Boot_Debug("RTOS 7: motor task OK\r\n");

    uart_debug_handle = osThreadNew(
        UartDebugTask,
        NULL,
        &uart_debug_attributes
    );

    if (uart_debug_handle == NULL)
    {
        Boot_Debug("ERROR: debug task create failed\r\n");
        Error_Handler();
    }

    Boot_Debug("RTOS 8: debug task OK\r\n");
    Boot_Debug("RTOS 9: scheduler start\r\n");

    if (osKernelStart() != osOK)
    {
        Boot_Debug("ERROR: scheduler start failed\r\n");
        Error_Handler();
    }

    /* osKernelStart() does not return when successful. */
    Error_Handler();

    while (1)
    {
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
  * @brief UART receive complete callback.
  * @note  This ISR only assembles a frame and puts it into a CMSIS-RTOS V2
  *        message queue. Parsing is deliberately left to task context.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        char c = (char)uart4_rx_char;

        if (c == '(')
        {
            uart4_rx_idx = 0U;
            uart4_receiving = 1U;
            uart4_rx_buf[uart4_rx_idx++] = c;
        }
        else if (uart4_receiving == 1U)
        {
            if (uart4_rx_idx < (UART4_RX_BUFFER_SIZE - 1U))
            {
                uart4_rx_buf[uart4_rx_idx++] = c;
            }

            if (c == ')')
            {
                uart4_rx_buf[uart4_rx_idx] = '\0';
                uart4_receiving = 0U;

                /*
                 * timeout must be 0 when called from an interrupt.
                 * The queue copies UART4_RX_BUFFER_SIZE bytes immediately.
                 */
                if (uart_command_queue != NULL)
                {
                    (void)osMessageQueuePut(uart_command_queue,
                                            uart4_rx_buf,
                                            0U,
                                            0U);
                }

                uart4_rx_idx = 0U;
            }
        }

        if (HAL_UART_Receive_IT(&huart4, &uart4_rx_char, 1U) != HAL_OK)
        {
            /*
             * Do not call Error_Handler() inside the ISR because it disables
             * interrupts permanently. The next task-level diagnostic can
             * handle a persistent UART fault.
             */
        }
    }
}

/**
  * @brief UART error callback.
  * @note  If UART4 encounters framing/noise/overrun errors, restart reception.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        /*
         * Reading SR/DR through the HAL clear macro releases an overrun
         * condition on STM32F1, then reception is armed again.
         */
        __HAL_UART_CLEAR_OREFLAG(huart);

        (void)HAL_UART_Receive_IT(&huart4,
                                  &uart4_rx_char,
                                  1U);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non-blocking mode.
  * @note   TIM6 is used as the HAL time base.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();

    if (motor_hw_ready == 1U)
    {
        Motor_Stop_All();
    }

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Reports the name of the source file and source line number.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif /* USE_FULL_ASSERT */
