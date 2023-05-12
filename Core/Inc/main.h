/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define LED_STT_Pin GPIO_PIN_13
#define LED_STT_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_14
#define BTN2_GPIO_Port GPIOC
#define VT6_Pin GPIO_PIN_15
#define VT6_GPIO_Port GPIOC
#define VT3_Pin GPIO_PIN_0
#define VT3_GPIO_Port GPIOA
#define VT5_Pin GPIO_PIN_1
#define VT5_GPIO_Port GPIOA
#define VT2_Pin GPIO_PIN_2
#define VT2_GPIO_Port GPIOA
#define VT4_Pin GPIO_PIN_3
#define VT4_GPIO_Port GPIOA
#define RC522_NSS_Pin GPIO_PIN_0
#define RC522_NSS_GPIO_Port GPIOB
#define VT1_Pin GPIO_PIN_1
#define VT1_GPIO_Port GPIOB
#define CB_OUT_Pin GPIO_PIN_10
#define CB_OUT_GPIO_Port GPIOB
#define CB_IN_Pin GPIO_PIN_11
#define CB_IN_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_12
#define SERVO2_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_13
#define SERVO1_GPIO_Port GPIOB
#define DTE_Pin GPIO_PIN_8
#define DTE_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_11
#define SPEAKER_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
