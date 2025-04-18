#include "uart.h"
#include "decode.h"

int value=0;

int rsspi_rx(){	
	if (start_receive==1){
	//如果接受到的数据只有1位则开始处理
		if(USART2_RX_STA>=1){
			//取最新的一个数据位进行处理
			value = USART2_RX_BUF[USART2_RX_STA-1]-48;
					//清空接收状态
			USART2_RX_STA = 0;
		}
		//异常处理,在函数调用后要判断返回值是否是INVADE
		else{
			return INVADE;		
		}
		return value;
	}
	else
		return 0;
}