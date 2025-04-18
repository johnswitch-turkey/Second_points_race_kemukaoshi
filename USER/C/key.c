#include "key.h"


uint8_t KEY1flag=0;
uint8_t KEY2flag=0;
uint8_t KEY3flag=0;

uint8_t KEY1state=0;
uint8_t KEY2state=0;
uint8_t KEY3state=0;

//利用状态机实现单按键消抖
void key_state(KEY_STATE *key,GPIO_TypeDef *port,uint16_t pin,uint8_t *keyflag)
{
  switch (*key)
  {
    case KEY_CHECK:
      if (HAL_GPIO_ReadPin(port,pin)== 0)
      {
        *key = KEY_COMFIRM;
      }
      break;
    case KEY_COMFIRM:
    {
      if (HAL_GPIO_ReadPin(port,pin)== 0)
      {
        *key = KEY_RELEASE;
        *keyflag = 1;
      }
      else
      {
        *key = KEY_CHECK;
      }
      break;
    }
    case KEY_RELEASE:
      if (HAL_GPIO_ReadPin(port,pin)== 1)
      {
        *key = KEY_CHECK;
      }
    break;
  }
}

//依次检测各个按键是否被按下，从而确定状态
void modechoose(uint8_t* ModeChoose)
{
	key_state(&KEY1state,KEY1_PORT, KEY1_PIN,&KEY1flag);
	key_state(&KEY2state,KEY1_PORT, KEY1_PIN,&KEY2flag);
	key_state(&KEY3state,KEY1_PORT, KEY1_PIN,&KEY3flag);
	
	if (KEY1flag==1)
		*ModeChoose=BACK_PACKING;
	else if (KEY2flag==1)
		*ModeChoose=SIDE_PACKING;
	else if (KEY3flag==1)
		*ModeChoose=BACK_SIDE_PACKING;
	
}
