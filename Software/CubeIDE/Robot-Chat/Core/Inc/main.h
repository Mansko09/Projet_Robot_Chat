/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

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
#define Enc2A_Pin GPIO_PIN_0
#define Enc2A_GPIO_Port GPIOA
#define Enc2B_Pin GPIO_PIN_1
#define Enc2B_GPIO_Port GPIOA
#define LED_ROUGE_Pin GPIO_PIN_4
#define LED_ROUGE_GPIO_Port GPIOA
#define LED_VERTE_Pin GPIO_PIN_5
#define LED_VERTE_GPIO_Port GPIOA
#define M1_FWD_Pin GPIO_PIN_8
#define M1_FWD_GPIO_Port GPIOA
#define M1_REV_Pin GPIO_PIN_9
#define M1_REV_GPIO_Port GPIOA
#define M2_FWD_Pin GPIO_PIN_10
#define M2_FWD_GPIO_Port GPIOA
#define M2_REV_Pin GPIO_PIN_11
#define M2_REV_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
