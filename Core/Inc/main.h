/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define E1_B_Pin GPIO_PIN_0
#define E1_B_GPIO_Port GPIOA
#define E1_A_Pin GPIO_PIN_1
#define E1_A_GPIO_Port GPIOA
#define B_V_Pin GPIO_PIN_4
#define B_V_GPIO_Port GPIOA
#define E2_A_Pin GPIO_PIN_6
#define E2_A_GPIO_Port GPIOA
#define E2_B_Pin GPIO_PIN_7
#define E2_B_GPIO_Port GPIOA
#define M1_I2_Pin GPIO_PIN_14
#define M1_I2_GPIO_Port GPIOF
#define M1_I1_Pin GPIO_PIN_15
#define M1_I1_GPIO_Port GPIOF
#define M2_I1_Pin GPIO_PIN_0
#define M2_I1_GPIO_Port GPIOG
#define M2_I2_Pin GPIO_PIN_1
#define M2_I2_GPIO_Port GPIOG
#define M3_I2_Pin GPIO_PIN_7
#define M3_I2_GPIO_Port GPIOE
#define M3_I1_Pin GPIO_PIN_8
#define M3_I1_GPIO_Port GPIOE
#define M1_S_Pin GPIO_PIN_9
#define M1_S_GPIO_Port GPIOE
#define M4_I1_Pin GPIO_PIN_10
#define M4_I1_GPIO_Port GPIOE
#define M2_S_Pin GPIO_PIN_11
#define M2_S_GPIO_Port GPIOE
#define M4_I2_Pin GPIO_PIN_12
#define M4_I2_GPIO_Port GPIOE
#define M3_S_Pin GPIO_PIN_13
#define M3_S_GPIO_Port GPIOE
#define M4_S_Pin GPIO_PIN_14
#define M4_S_GPIO_Port GPIOE
#define MPU_SCL_Pin GPIO_PIN_10
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_11
#define MPU_SDA_GPIO_Port GPIOB
#define E3_B_Pin GPIO_PIN_12
#define E3_B_GPIO_Port GPIOD
#define E3_A_Pin GPIO_PIN_13
#define E3_A_GPIO_Port GPIOD
#define EC1_Pin GPIO_PIN_6
#define EC1_GPIO_Port GPIOC
#define EC2_Pin GPIO_PIN_7
#define EC2_GPIO_Port GPIOC
#define EC3_Pin GPIO_PIN_8
#define EC3_GPIO_Port GPIOC
#define EC4_Pin GPIO_PIN_9
#define EC4_GPIO_Port GPIOC
#define TR4_Pin GPIO_PIN_8
#define TR4_GPIO_Port GPIOA
#define TE3_Pin GPIO_PIN_9
#define TE3_GPIO_Port GPIOA
#define TR2_Pin GPIO_PIN_10
#define TR2_GPIO_Port GPIOA
#define TR1_Pin GPIO_PIN_11
#define TR1_GPIO_Port GPIOA
#define E4_B_Pin GPIO_PIN_15
#define E4_B_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_10
#define ESP_RX_GPIO_Port GPIOC
#define ESP_TX_Pin GPIO_PIN_11
#define ESP_TX_GPIO_Port GPIOC
#define E4_A_Pin GPIO_PIN_3
#define E4_A_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_6
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_7
#define LCD_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
