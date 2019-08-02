#include "main.h"
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� ���е�6050ʹ��DMPʱ����Ҫ������ͣҡ��С��10S���ң��ȴ������ȶ���
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=1;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Desire1=150, Desire2=0;
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
float balance_kp=60.0, balance_kd=0.18;
float velocity_kp=7.0, velocity_ki=2, velocity_kd = 0;
float turn_kp=2.0, turn_kd=0.083;
int vol = 500;
int turn_vol = 1;

/**************************************************************************
�������ܣ������� ��ʼ��ϵͳ������
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int main(void)
{
	SystemInit();                   //=====ϵͳ��ʼ��
	delay_init(72);                 //=====��ʱ����
	usart1_init();                  //=====����1��ʼ�� �����ʣ�115200
	uart3_init(72,9600);            //=====����3��ʼ�� �����ʣ�9600
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	led_init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
	Adc_Init();	                    //=====��ʼ��ADCģ��
	MiniBalance_PWM_Init(3599,0);   //=====��ʼ��PWM 20KHZ ��Ƶ���Է�ֹ�����Ƶʱ�ļ����
	OLED_Init();	                  //=====��ʼ��OLED ģ��SPI 
	Encoder_Init();                 //=====��ʼ��������1
	Encoder_Init2();	              //=====��ʼ��������2
	delay_ms(200);                  //=====��ʱ�ȴ��ȶ�		
	IIC_Init();                     //=====ģ��IIC��ʼ��
	MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //=====DMP��ʼ��
	Timer1_Init(49,7199);           //=====5MS��һ���жϷ����� �жϷ�������minibalance.c����
	
	while(1)
	{
		DataScope();
		delay_ms(50);
	}	
}

