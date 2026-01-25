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
#include "stm32h7xx_hal.h"

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
#define M1_ENC_Z_Pin GPIO_PIN_4
#define M1_ENC_Z_GPIO_Port GPIOE
#define M1_ENC_Z_EXTI_IRQn EXTI4_IRQn
#define M1_ENC_A_Pin GPIO_PIN_5
#define M1_ENC_A_GPIO_Port GPIOE
#define M1_ENC_B_Pin GPIO_PIN_6
#define M1_ENC_B_GPIO_Port GPIOE
#define M0_TEMP_Pin GPIO_PIN_0
#define M0_TEMP_GPIO_Port GPIOC
#define M1_TEMP_Pin GPIO_PIN_1
#define M1_TEMP_GPIO_Port GPIOC
#define M1_SO2_Pin GPIO_PIN_2
#define M1_SO2_GPIO_Port GPIOA
#define M0_SO2_Pin GPIO_PIN_3
#define M0_SO2_GPIO_Port GPIOA
#define M1_SO3_Pin GPIO_PIN_4
#define M1_SO3_GPIO_Port GPIOA
#define M0_SO1_Pin GPIO_PIN_6
#define M0_SO1_GPIO_Port GPIOA
#define M1_SO1_Pin GPIO_PIN_4
#define M1_SO1_GPIO_Port GPIOC
#define M0_SO3_Pin GPIO_PIN_5
#define M0_SO3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_7
#define KEY1_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOB
#define M2_ENC_A_Pin GPIO_PIN_4
#define M2_ENC_A_GPIO_Port GPIOB
#define M2_ENC_B_Pin GPIO_PIN_5
#define M2_ENC_B_GPIO_Port GPIOB
#define M2_ENC_Z_Pin GPIO_PIN_6
#define M2_ENC_Z_GPIO_Port GPIOB
#define M2_ENC_Z_EXTI_IRQn EXTI9_5_IRQn
#define KEY3_Pin GPIO_PIN_0
#define KEY3_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_1
#define KEY4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
