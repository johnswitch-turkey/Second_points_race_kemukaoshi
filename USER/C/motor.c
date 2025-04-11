/**
  *************************************************************************************************************************
  * @file    motor.c
  * @author  amkl
  * @version V1.0
  * @date    2022-09-22
  * @brief   xx模块.c文件配置
  *************************************************************************************************************************
  * @attention
  *
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "motor.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;


/* 定义 -----------------------------------------------------------------------------------------------------------------*/


/*电机初始化函数*/
//void Motor_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    
//    // 使能GPIO时钟
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    
//    // 初始化GPIO为推挽输出
//    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//}

/*限幅函数*/
//void Limit(int *motoA, int *motoB, float *servo)
//{
//    if(*motoA >= 6500) *motoA = 6500;
//    if(*motoA <= -6500) *motoA = -6500;
//    
//    if(*motoB >= 6500) *motoB = 6500;
//    if(*motoB <= -6500) *motoB = -6500;

//    if(*servo >= 1900) *servo = 1900;
//    if(*servo <= 1800) *servo = 1800;    
//}

/*绝对值函数*/
int abs(int p)
{
    int q;
    q = p > 0 ? p : (-p);
    return q;
}

/*赋值函数*/
/*入口参数：PID运算完成后的最终PWM值*/
void Load(int moto2, int moto1, uint16_t Target_Position)
{
    // 1.研究正负号，对应正反转
    if(moto1 > 0) {
        HAL_GPIO_WritePin(Ain1_Port, Ain1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Ain2_Port, Ain2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Ain1_Port, Ain1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Ain2_Port, Ain2_Pin, GPIO_PIN_SET);
    }
    
    if(moto2 > 0) {
        HAL_GPIO_WritePin(Bin1_Port, Bin1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Bin2_Port, Bin2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Bin1_Port, Bin1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Bin2_Port, Bin2_Pin, GPIO_PIN_SET);
    }    
    
    // 2.装载PWM值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(moto1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(moto2));
    
    // 3.装载舵机PWM值
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Target_Position);
}

/**
 * 函数名:set_motor_enable
 * 描述:使能电机
 * 输入:无
 * 输出:无
 */
void set_motor_enable(void)  
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 使能TIM1通道1 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // 使能TIM1通道4 PWM
}

/**
 * 函数名:set_motor_disable
 * 描述:失能电机
 * 输入:无
 * 输出:无
 */
void set_motor_disable(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // 关闭TIM1通道1 PWM
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);  // 关闭TIM1通道4 PWM
}

/*****************************************************END OF FILE*********************************************************/
