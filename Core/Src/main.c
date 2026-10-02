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
#include "motor.h"
#include "encoder.h"
#include "wheel_control.h"
#include "motion_control.h"
#include "debug_console.h"
#include "chassis_tasks.h"
#include "power_monitor.h"
#include "imu.h"
#include "ultrasonic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/*
 * Indicates whether the motor hardware has been initialized.
 * Error_Handler() uses this flag to determine whether the motors
 * can be stopped safely.
 */
static volatile uint8_t motor_hw_ready = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    ChassisTasks_Status_t chassis_status;

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
     * Initialize the debug console after UART5 is ready.
     */
    DebugConsole_Init(&huart5);

    /*
     * UART4 ISR calls CMSIS-RTOS V2 APIs, so it must use an
     * RTOS-safe interrupt priority that matches FreeRTOSConfig.h.
     */
    HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

    /* USER CODE BEGIN 2 */

    Encoder_Init();
    Motor_Init();
    WheelControl_Init();
    MotionControl_Init();

    if (Ultrasonic_Init() != ULTRASONIC_STATUS_OK)
    {
        DebugConsole_WriteText(
            "ERROR: ultrasonic init failed\r\n"
        );

        Error_Handler();
    }

    DebugConsole_WriteText(
        "Ultrasonic init OK\r\n"
    );

    if (IMU_Init(&hi2c2) != IMU_STATUS_OK)
    {
        DebugConsole_WriteText( "ERROR: IMU DMP init failed\r\n" );

        Error_Handler();
    }

    DebugConsole_WriteText( "IMU DMP init OK\r\n" );

    if (PowerMonitor_Init() != POWER_MONITOR_STATUS_OK)
    {
        DebugConsole_WriteText(
            "ERROR: power monitor init failed\r\n"
        );

        Error_Handler();
    }

    motor_hw_ready = 1U;

    DebugConsole_WriteText("Init OK\r\n");

    /* USER CODE END 2 */

    /*
     * Initialize the CMSIS-RTOS V2 kernel.
     */
    DebugConsole_WriteText("RTOS 1: kernel init\r\n");

    if (osKernelInitialize() != osOK)
    {
        DebugConsole_WriteText("ERROR: osKernelInitialize\r\n");
        Error_Handler();
    }

    DebugConsole_WriteText("RTOS 2: kernel init OK\r\n");

    /*
     * CubeMX defaultTask is intentionally not created here.
     * Re-enable MX_FREERTOS_Init() only when objects generated
     * in freertos.c are required.
     */
    /* MX_FREERTOS_Init(); */

    DebugConsole_WriteText("RTOS 3: communication init\r\n");

    chassis_status = ChassisTasks_Init(&huart4,DebugConsole_WriteText);

    if (chassis_status != CHASSIS_TASKS_STATUS_OK)
    {
        DebugConsole_WriteText("ERROR: communication init failed\r\n");
        Error_Handler();
    }

    DebugConsole_WriteText("RTOS 4: communication OK\r\n");

    DebugConsole_WriteText("RTOS 5: create debug console mutex\r\n");

    if (DebugConsole_CreateMutex() == 0U)
    {
        DebugConsole_WriteText("ERROR: debug console mutex create failed\r\n");
        Error_Handler();
    }

    DebugConsole_WriteText("RTOS 6: debug console mutex OK\r\n");

    chassis_status =ChassisTasks_CreateMotorTask();

    if (chassis_status != CHASSIS_TASKS_STATUS_OK)
    {
        DebugConsole_WriteText("ERROR: motor task create failed\r\n");
        Error_Handler();
    }

    DebugConsole_WriteText("RTOS 7: motor task OK\r\n");

    chassis_status = ChassisTasks_CreateImuTask();

    if (chassis_status != CHASSIS_TASKS_STATUS_OK)
    {
        DebugConsole_WriteText(
            "ERROR: IMU task create failed\r\n"
        );

        Error_Handler();
    }

    DebugConsole_WriteText(
        "RTOS: IMU task OK\r\n"
    );

    chassis_status = ChassisTasks_CreateUltrasonicTask();

    if (chassis_status != CHASSIS_TASKS_STATUS_OK)
    {
        DebugConsole_WriteText(
            "ERROR: ultrasonic task create failed\r\n"
        );

        Error_Handler();
    }

    DebugConsole_WriteText(
        "RTOS: ultrasonic task OK\r\n"
    );

    chassis_status = ChassisTasks_CreateDebugTask();

    if (chassis_status != CHASSIS_TASKS_STATUS_OK)
    {
        DebugConsole_WriteText(
            "ERROR: debug task create failed\r\n"
        );

        Error_Handler();
    }

    DebugConsole_WriteText(
        "RTOS 8: debug task OK\r\n"
    );
    DebugConsole_WriteText("RTOS 9: scheduler start\r\n");

    if (osKernelStart() != osOK)
    {
        DebugConsole_WriteText("ERROR: scheduler start failed\r\n");
        Error_Handler();
    }

    /*
     * osKernelStart() does not return when successful.
     */
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2) != HAL_OK)
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
