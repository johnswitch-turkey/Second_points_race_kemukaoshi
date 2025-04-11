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
#include "stm32f4xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define EncoderA_1_Pin GPIO_PIN_0
#define EncoderA_1_GPIO_Port GPIOA
#define EncoderA_2_Pin GPIO_PIN_1
#define EncoderA_2_GPIO_Port GPIOA
#define Beep_Pin GPIO_PIN_4
#define Beep_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_6
#define SERVO_GPIO_Port GPIOA
#define Bin2_Pin GPIO_PIN_12
#define Bin2_GPIO_Port GPIOB
#define Bin1_Pin GPIO_PIN_13
#define Bin1_GPIO_Port GPIOB
#define Ain1_Pin GPIO_PIN_14
#define Ain1_GPIO_Port GPIOB
#define Ain2_Pin GPIO_PIN_15
#define Ain2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_10
#define PWM2_GPIO_Port GPIOA
#define EncoderB_1_Pin GPIO_PIN_6
#define EncoderB_1_GPIO_Port GPIOB
#define EncoderB_2_Pin GPIO_PIN_7
#define EncoderB_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
