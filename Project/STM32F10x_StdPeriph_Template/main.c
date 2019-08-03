#include "main.h"

int MotoL=0, MotoR=0;				//电机PWM变量
int Encoder_Left,Encoder_Right;		//左右编码器的脉冲计数
float DesireL= 0, DesireR = 0;		//期望的脉冲计数
char Stop = 0;						//启动/停止
int Temperature;					//显示温度
int Voltage;						//电池电压采样相关的变量
float velocity_kp=7, velocity_ki=2, velocity_kd = 0;
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
/***********************************************/
float DesireVelocity = 0.2;			//期望速度 m/s
float DesireAngVelo = 3.14*20/180;	//期望的角速度 rad/s

int main(void)
{
	SystemInit();                   //=====系统初始化
	delay_init(72);                 //=====延时函数
	usart1_init();                  //=====串口1初始化 波特率：115200，上位机通信
	uart3_init(72,9600);            //=====串口3初始化 波特率：9600，蓝牙
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	led_init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
	Adc_Init();	                    //=====初始化ADC模块
	MiniBalance_PWM_Init(3599,0);   //=====初始化PWM 20KHZ 高频可以防止电机低频时的尖叫声
	OLED_Init();	                //=====初始化OLED 模拟SPI 
	Encoder_Init();                 //=====初始化编码器1
	Encoder_Init2();	            //=====初始化编码器2
	delay_ms(200);                  //=====延时等待稳定		
	IIC_Init();                     //=====模拟IIC初始化 MPU6050
	MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //=====DMP初始化
	Timer1_Init(49,7199);           //=====5MS进一次中断服务函数 中断服务函数在minibalance.c里面
	
	while(1)
	{
		DataScope();
		delay_ms(20);
	}	
}
