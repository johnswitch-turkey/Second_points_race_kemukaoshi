#ifndef __KEY_H 
#define	__KEY_H

#include "stm32f4xx_hal.h"


// 设置灯与蜂鸣器开关
#define BEEP_OFF    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BEEP_ON     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BEEP_TOGGLE HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4)
#define LED_OFF     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_ON      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_TOGGLE  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

#endif
