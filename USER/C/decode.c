#include "usart.h"
#include "decode.h"

int flag=0;
int value=0;

int get_value(){

	flag = USART1_RX_STA ;		
	
	//如果接受到的数据只有1位则开始处理
	if(flag==1){
		//清空接收状态
		USART1_RX_STA = 0;
		value = str_num-48;
	}
	//异常处理,在函数调用后要判断返回值是否是INVADE
	else{
		return INVADE;
			
	}
	return value;
}