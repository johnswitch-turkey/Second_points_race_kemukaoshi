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


// 按键引脚
#define KEY1_PIN GPIO_PIN_0
#define KEY1_PORT GPIOA
#define KEY2_PIN GPIO_PIN_4
#define KEY2_PORT GPIOA
#define KEY3_PIN GPIO_PIN_12
#define KEY3_PORT GPIOB

//定义按键状态机的各个状态
typedef enum
{
	KEY_CHECK,
	KEY_COMFIRM,
	KEY_RELEASE
}KEY_STATE;

//检测按键被按下的标记
uint8_t KEY1flag;
uint8_t KEY2flag;
uint8_t KEY3flag;

//按键状态，对应KEY_STATE
uint8_t KEY1state;
uint8_t KEY2state;
uint8_t KEY3state;

//定义几种模式
#define BACK_PACKING 			1
#define SIDE_PACKING 			2
#define BACK_SIDE_PACKING 3
uint8_t 	ModeChoose;    //模式选择

void key_state(KEY_STATE *key,GPIO_TypeDef *port,uint16_t pin,uint8_t *keyflag);
void modechoose(uint8_t* ModeChoose);
#endif
