#include "encoder.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;



int Read_Speed(uint8_t TIMx)
{
    int value = 0;
    
    switch(TIMx)
    {
        case 2:
            value = (int)__HAL_TIM_GET_COUNTER(&htim2);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            break;
            
        case 4:
            value = (int)__HAL_TIM_GET_COUNTER(&htim4);
            __HAL_TIM_SET_COUNTER(&htim4, 0);
            break;
            
        default:
            value = 0;
            break;
    }
    
    return value;
}