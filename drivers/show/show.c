#include "show.h"
#include "MiniBalance.h"
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
static u32 Count;
void oled_show(void)
{
	static float angle = 0.0;
	Count=0;
	OLED_Display_On();  //��ʾ����
		//=============��ʾ�˲���=======================//	
	//OLED_ShowString(00,0,"WAY-");
	OLED_ShowNumber(0,0, Way_Angle,1,12);
	if(Way_Angle==1)	OLED_ShowString(6,0,"-DMP");
	else if(Way_Angle==2)	OLED_ShowString(6,0,"-Kalman");
	else if(Way_Angle==3)	OLED_ShowString(6,0,"-Hubu");
	//=============��ʾ��ѹ=======================//          
	OLED_ShowNumber(54,0,Voltage/100,2,12);
	OLED_ShowString(66,0,".");
	OLED_ShowNumber(72,0,Voltage%100/10,1,12);
	OLED_ShowString(78,0,"V");
	//=============��ʾ�Ƕ�=======================//
	angle = Angle_Balance;
	if(Angle_Balance<0) {
		angle += 360;
	}
	OLED_ShowNumber(98,0,angle,3,12);
	OLED_ShowString(116,0,".");
	OLED_ShowNumber(122,0,((u32)(angle*10))%10,1,12);
	//=============ƽ��PD=======================//
	OLED_ShowString(0,12,"ba");
	OLED_ShowFloat(16, 12, balance_kp);
	OLED_ShowFloat(72, 12, balance_kd);
	//=============�ٶ�PI=======================//
	OLED_ShowString(0,24,"vo");
	OLED_ShowFloat(16, 24, velocity_kp);
	OLED_ShowFloat(72, 24, velocity_ki);
	//=============ת��PD=======================//
	OLED_ShowString(0,36,"tn");
	OLED_ShowFloat(16, 36, turn_kp);
	OLED_ShowFloat(72, 36, turn_kd);

	//=============velocity======================//
	OLED_ShowString(0, 52,"vv");
	OLED_ShowNumber(16, 52, vol, 5, 12);
	//=============turn velocity======================//
	OLED_ShowString(64, 52,"tv");
	OLED_ShowNumber(80, 52, turn_vol, 2, 12);

	//============UART3 BUF=======================//
	//mode_data[sizeof(mode_data)-1] = '\0';
	//OLED_ShowString(0,52,mode_data);

	//=============��ʾ�¶�=======================//	
	//   OLED_ShowString(00,10,"Wendu");
	//   OLED_ShowNumber(45,10,Temperature/10,2,12);
	//   OLED_ShowNumber(68,10,Temperature%10,1,12);
	//   OLED_ShowString(58,10,".");
	//   OLED_ShowString(80,10,"`C");
		//=============��ʾ������1=======================//	
	// 	                      OLED_ShowString(00,20,"Enco1");
	// 	if( Encoder_Left<0)		OLED_ShowString(45,20,"-"),
	// 	                      OLED_ShowNumber(65,20,-Encoder_Left,5,12);
	// 	else                 	OLED_ShowString(45,20,"+"),
	// 	                      OLED_ShowNumber(65,20, Encoder_Left,5,12);
  	// //=============��ʾ������2=======================//		
	// 	                      OLED_ShowString(00,30,"Enco2");
	// 	if(Encoder_Right<0)		OLED_ShowString(45,30,"-"),
	// 	                      OLED_ShowNumber(65,30,-Encoder_Right,5,12);
	// 	else               		OLED_ShowString(45,30,"+"),
	// 	                      OLED_ShowNumber(65,30,Encoder_Right,5,12);	
		
		
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	
	}
/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DataScope(void)
{   
	DataScope_Get_Channel_Data( (float)Desire1, 1 );
	DataScope_Get_Channel_Data( (float)Encoder_Left, 2 );
	DataScope_Get_Channel_Data( (float)Moto1, 3 ); 
	DataScope_Get_Channel_Data( (float)Voltage , 4 );
	DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
	DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
	DataScope_Get_Channel_Data(0, 7 );
	DataScope_Get_Channel_Data( 0, 8 ); 
	DataScope_Get_Channel_Data(0, 9 );  
	DataScope_Get_Channel_Data( 0 , 10);
	Send_Count = DataScope_Data_Generate(4);
	for( i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}
	delay_ms(50); //20HZ
}
