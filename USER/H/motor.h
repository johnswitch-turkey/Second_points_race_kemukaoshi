#ifndef  _MOTOR_H
#define  _MOTOR_H

#include "stm32f4xx_hal.h"

#define Ain1_Pin    GPIO_PIN_14
#define Ain1_Port   GPIOB
#define Ain2_Pin    GPIO_PIN_15
#define Ain2_Port   GPIOB

#define Bin1_Pin    GPIO_PIN_13
#define Bin1_Port   GPIOB
#define Bin2_Pin    GPIO_PIN_12
#define Bin2_Port   GPIOB

void Limit(int *motoA, int *motoB, float *servo);
int abs(int p);
void Load(int moto2, int moto1, uint16_t Target_Position);
void set_motor_enable(void);
void set_motor_disable(void);

#endif
