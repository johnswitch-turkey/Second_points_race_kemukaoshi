#include "uart.h"
#include "decode.h"

int value=0;

int rsspi_rx(){	
	if (start_receive==1){
	//������ܵ�������ֻ��1λ��ʼ����
		if(USART2_RX_STA>=1){
			//ȡ���µ�һ������λ���д���
			value = USART2_RX_BUF[USART2_RX_STA-1]-48;
					//��ս���״̬
			USART2_RX_STA = 0;
		}
		//�쳣����,�ں������ú�Ҫ�жϷ���ֵ�Ƿ���INVADE
		else{
			return INVADE;		
		}
		return value;
	}
	else
		return 0;
}  	