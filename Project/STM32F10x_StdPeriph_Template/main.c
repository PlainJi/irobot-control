#include "main.h"

int MotoL=0, MotoR=0;				//���PWM����
int Encoder_Left,Encoder_Right;		//���ұ��������������
float DesireL= 0, DesireR = 0;		//�������������
char Stop = 0;						//����/ֹͣ
int Temperature;					//��ʾ�¶�
int Voltage;						//��ص�ѹ������صı���
float velocity_kp=7, velocity_ki=2, velocity_kd = 0;
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
/***********************************************/
float DesireVelocity = 0.2;			//�����ٶ� m/s
float DesireAngVelo = 3.14*20/180;	//�����Ľ��ٶ� rad/s

int main(void)
{
	SystemInit();                   //=====ϵͳ��ʼ��
	delay_init(72);                 //=====��ʱ����
	usart1_init();                  //=====����1��ʼ�� �����ʣ�115200����λ��ͨ��
	uart3_init(72,9600);            //=====����3��ʼ�� �����ʣ�9600������
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	led_init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
	Adc_Init();	                    //=====��ʼ��ADCģ��
	MiniBalance_PWM_Init(3599,0);   //=====��ʼ��PWM 20KHZ ��Ƶ���Է�ֹ�����Ƶʱ�ļ����
	OLED_Init();	                //=====��ʼ��OLED ģ��SPI 
	Encoder_Init();                 //=====��ʼ��������1
	Encoder_Init2();	            //=====��ʼ��������2
	delay_ms(200);                  //=====��ʱ�ȴ��ȶ�		
	IIC_Init();                     //=====ģ��IIC��ʼ�� MPU6050
	MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //=====DMP��ʼ��
	Timer1_Init(49,7199);           //=====5MS��һ���жϷ����� �жϷ�������minibalance.c����
	
	while(1)
	{
		DataScope();
		delay_ms(20);
	}	
}
