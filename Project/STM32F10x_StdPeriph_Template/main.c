#include "main.h"
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 （有的6050使用DMP时，需要开机后不停摇晃小车10S左右，等待数据稳定）
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=1;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Desire1=150, Desire2=0;
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
float balance_kp=60.0, balance_kd=0.18;
float velocity_kp=7.0, velocity_ki=2, velocity_kd = 0;
float turn_kp=2.0, turn_kd=0.083;
int vol = 500;
int turn_vol = 1;

/**************************************************************************
函数功能：主函数 初始化系统和外设
作    者：平衡小车之家
**************************************************************************/
int main(void)
{
	SystemInit();                   //=====系统初始化
	delay_init(72);                 //=====延时函数
	usart1_init();                  //=====串口1初始化 波特率：115200
	uart3_init(72,9600);            //=====串口3初始化 波特率：9600
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	led_init();                     //=====LED初始化
	KEY_Init();                     //=====按键初始化
	Adc_Init();	                    //=====初始化ADC模块
	MiniBalance_PWM_Init(3599,0);   //=====初始化PWM 20KHZ 高频可以防止电机低频时的尖叫声
	OLED_Init();	                  //=====初始化OLED 模拟SPI 
	Encoder_Init();                 //=====初始化编码器1
	Encoder_Init2();	              //=====初始化编码器2
	delay_ms(200);                  //=====延时等待稳定		
	IIC_Init();                     //=====模拟IIC初始化
	MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //=====DMP初始化
	Timer1_Init(49,7199);           //=====5MS进一次中断服务函数 中断服务函数在minibalance.c里面
	
	while(1)
	{
		DataScope();
		delay_ms(50);
	}	
}

