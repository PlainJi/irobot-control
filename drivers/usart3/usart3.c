#include <string.h>
#include "main.h"
#include "usart3.h"
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
 u8 mode_data[13]="123456789012";
 u8 six_data_1[4]={6,5,4,0};
 u8 six_data_2[4]={4,5,6,0};

void uart3_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000/2)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
  mantissa<<=4;
	mantissa+=fraction; 
	//AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
	RCC->APB2ENR|=1<<3;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOB->CRH&=0XFFFF00FF; 
	GPIOB->CRH|=0X00008B00;//IO״̬����
	GPIOB->ODR|=1<<10;	  
	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART3->BRR=mantissa; // ����������	 
	USART3->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART3->CR1|=1<<8;    //PE�ж�ʹ��
	USART3->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(1,3,USART3_IRQChannel,2);//��2��������ȼ� 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//���յ�����
	{	  
	  	static	int uart_receive=0;//����������ر���
		static u8 p_w = 0;
		uart_receive=USART3->DR; 
		if (uart_receive == '$')
			p_w = 0;
		if (p_w == sizeof(mode_data))
			p_w = 0;
		mode_data[p_w++] = uart_receive;
		
		if (uart_receive == '#' && mode_data[0] == '$') {
			p_w = 0;
			if(!strncmp((const char*)(mode_data+1), "C5", 2))
				Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;						//ɲ��
			else if(!strncmp((const char*)(mode_data+1), "C2", 2))
				Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;						//ǰ
			else if(!strncmp((const char*)(mode_data+1), "C8", 2))
				Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;						//��
			else if(!strncmp((const char*)(mode_data+1), "C4", 2))
				Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;						//��
			else if(!strncmp((const char*)(mode_data+1), "C6", 2))
				Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;						//��
			else if(!strncmp((const char*)(mode_data+1), "CL", 2))
					Flag_Show ^= 1;																						//����OLED��ʾ
			else if(!strncmp((const char*)(mode_data+1), "CS", 2))
					Flag_Stop=!Flag_Stop;																			//ֹͣ�������
			else if (mode_data[1] == 'S') {
				static float value = 0.0;
				value = (float)((mode_data[4]-'0')*100000 + (mode_data[5]-'0')*10000 + 
								(mode_data[6]-'0')*1000 + (mode_data[7]-'0')*100 + 
								(mode_data[8]-'0')*10 + (mode_data[9]-'0'))/1000.0;
				if (!strncmp((const char*)(mode_data+2), "BP", 2))					// Kp of balance
					balance_kp = value;
				else if (!strncmp((const char*)(mode_data+2), "BD", 2))				// Kd of balance
					balance_kd = value;
				else if (!strncmp((const char*)(mode_data+2), "VP", 2))				// Kp of velocity
					velocity_kp = value;
				else if (!strncmp((const char*)(mode_data+2), "VI", 2))				// Ki of velocity
					velocity_ki = value;
				else if (!strncmp((const char*)(mode_data+2), "VV", 2))				// velocity
					vol = (int)value;
				else if (!strncmp((const char*)(mode_data+2), "TP", 2))				// Kp of turn
					turn_kp = value;
				else if (!strncmp((const char*)(mode_data+2), "TD", 2))				// Kd of turn
					turn_kd = value;
				else if (!strncmp((const char*)(mode_data+2), "TV", 2))				// turn velocity
					turn_vol = (int)value;
			}
		}
	}  											 
} 

