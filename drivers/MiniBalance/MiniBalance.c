#include "MiniBalance.h"
#include "math.h"
#include "led.h"
#include "mpu6050.h"
#define PI 3.14159265
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
函数功能：5MS定时中断函数 5MS控制周期
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void) {
  if (TIM1->SR & 0X0001)  // 5ms定时中断
  {
    TIM1->SR &= ~(1 << 0);  //===清除定时器1中断标志位
    readEncoder();          //===读取编码器的值
    Led_Flash(20);          //===LED闪烁
    Get_battery_volt();     //===获取电池电压
    // key(100); //===扫描按键状态 
	// Get_Angle(Way_Angle); //===更新姿态

	Moto1 = pid_velocity();
    Xianfu_Pwm();
    //if (!Turn_Off(0, Voltage)) {
    Set_Pwm(Moto1, 0);
    //}
  }
}

int pid_velocity(void) {
  static int Integral = 0;
  static int LastError = 0;
	int velocity = 0;

  int ErrorL = ((int)Desire1 - Encoder_Left);
  Integral += ErrorL;
  velocity = velocity_kp * ErrorL + velocity_ki * Integral  + velocity_kd * (ErrorL - LastError);

  LastError = ErrorL;
  if (Integral > 360000) Integral = 360000;
  if (Integral < -360000) Integral = -360000;

  return velocity;
}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-0;              //===求出平衡的角度中值 和机械相关 -0意味着身重中心在0度附近 如果身重中心在5度附近 那就应该减去5
	 balance=Bias*balance_kp + Gyro*balance_kd;//===计算平衡控制的电机PWM  PD控制   35是P系数 0.125是D系数 
	 //balance=35*Bias+Gyro*0.125;//===计算平衡控制的电机PWM  PD控制   35是P系数 0.125是D系数 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修改Movement的值，比如，改成-600和600就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	  static int Velocity,Encoder_Least,Encoder,Movement;
	  static int Encoder_Integral;
	  //=============遥控前进后退部分=======================//
		if(1==Flag_Qian)	Movement=-vol;	             //===如果前进标志位置1 位移为负
		else if(1==Flag_Hou)	  Movement=vol;          //===如果后退标志位置1 位移为正
	  else  Movement=0;	
   //=============速度PI控制器======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;  //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此次为零） 
		Encoder *= 0.8;		                             //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                 //===一阶低通滤波器    
	  if(Turn_Off(Angle_Balance,Voltage)==0)         //为了防止积分影响用户体验，只有电机开启的时候才开始积分
		{	
  	Encoder_Integral +=Encoder;                     //===积分出位移 积分时间：5ms
		Encoder_Integral=Encoder_Integral-Movement;     //===接收遥控器数据，控制前进后退
		}
		if(Encoder_Integral>360000)  	Encoder_Integral=360000;          //===积分限幅
		if(Encoder_Integral<-360000)	Encoder_Integral=-360000;         //===积分限幅	
		Velocity=Encoder*velocity_kp+Encoder_Integral*velocity_ki;                        //===速度PI控制器	速度P系数是4  I系数是1/140
		//Velocity=Encoder*4+Encoder_Integral/140;                        //===速度PI控制器	速度P系数是4  I系数是1/140
		if(Turn_Off(Angle_Balance,Voltage)==1)   Encoder_Integral=0;    //===电机关闭后清除积分
	  return Velocity;
}

/**************************************************************************
函数功能：转向PD控制
入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
  	static int Turn_Target,Turn,Encoder_temp,Turn_Convert=3,Turn_Count;
	  int Turn_Bias,Turn_Amplitude=1500/Way_Angle+800;     //===Way_Angle为滤波方法，当是1时，即由DMP获取姿态，Turn_Amplitude取大，卡尔曼和互补是，取小，因为这两种滤波算法效果稍差。
	  static long Turn_Bias_Integral;
	  //=============遥控左右旋转部分=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
			Turn_Convert=2000/Encoder_temp;
			if(Turn_Convert<3)Turn_Convert=3;
			if(Turn_Convert>10)Turn_Convert=10;
		}
	  else
		{
			Turn_Convert=3;
			Turn_Count=0;
			Encoder_temp=0;
		}
		Turn_Convert = turn_vol;
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert; //左转
		else if(1==Flag_Right)	     Turn_Target-=Turn_Convert; //右转
		else Turn_Target=0;                                     //停止
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
  	//=============转向PD控制器=======================//
		 Turn_Bias=Encoder_Left-Encoder_Right;         //===计算转向速度偏差  
		if(Turn_Off(Angle_Balance,Voltage)==0)         //为了防止积分影响用户体验，只有电机开启的时候才开始积分
		{	
		Turn_Bias_Integral+=Turn_Bias;                //转向速度偏差积分得到转向偏差（请认真理解这句话）
		Turn_Bias_Integral-=Turn_Target;              //获取遥控器数据
		}
		if(Turn_Bias_Integral>1800)  	Turn_Bias_Integral=1800;          //===积分限幅
		if(Turn_Bias_Integral<-1800)	Turn_Bias_Integral=-1800;         //===积分限幅	
	  Turn=Turn_Bias_Integral*turn_kp+gyro*turn_kd;   //===结合Z轴陀螺仪进行PD控制
	  //Turn=Turn_Bias_Integral*2+gyro/12;   //===结合Z轴陀螺仪进行PD控制
	  return Turn;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
			if(moto1<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto1);
		  if(moto2<0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);	
}
/**************************************************************************
函数功能：读取编码器的数据并进行数据类型转换
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void readEncoder(void)
{
	  u16 Encoder_L,Encoder_R;       //===左右编码器的脉冲计数
		Encoder_R = TIM4 -> CNT;       //===获取正交解码1数据	
		TIM4 -> CNT=0;                 //===计数器清零  
	  Encoder_L= TIM2 -> CNT;        //===获取正交解码2数据	
	  TIM2 -> CNT=0;	               //===计数器清零
		if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  
	  //=这个处理的原因是：编码器到0后会跳到65000向下计数，这样处理方便我们在控制程序中使用
	  if(Encoder_R>32768)  Encoder_Right=Encoder_R-65000; else  Encoder_Right=Encoder_R;
	 	Encoder_Left=-Encoder_Left;//这里取反是因为，平衡小车的两个电机是旋转了180度安装的，为了保证前进后退时候的编码器数据符号一致
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=3500;    //===PWM满幅是3600 限制在3500
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
作    者：平衡小车之家
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<-40||angle>40||1==Flag_Stop||Voltage<1110)//===电压低于11.1V 关闭电机
			{	                                                 //===倾角大于40度关闭电机
				temp=1;                                            //===Flag_Stop置1关闭电机
				AIN1=0;                                            //===可自行增加主板温度过高时关闭电机
				AIN2=0;
				BIN1=0;
				BIN2=0;
      }
			else
				temp=0;
      return temp;			
}
	
/**************************************************************************
函数功能：获取角度
入口参数：获取角度的算法 1：无  2：卡尔曼 3：互补滤波
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	    if(way==1)                                      //DMP没有涉及到严格的时序问题，在主函数读取
			{	
			}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度记
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度记
		  if(Gyro_Y>32768)  Gyro_Y-=65536;     //数据类型转换  也可通过short强制类型转换
			if(Gyro_Z>32768)  Gyro_Z-=65536;     //数据类型转换
	  	if(Accel_X>32768) Accel_X-=65536;    //数据类型转换
		  if(Accel_Z>32768) Accel_Z-=65536;    //数据类型转换
			Gyro_Balance=-Gyro_Y;                                  //更新平衡角速度
	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //计算与地面的夹角	
		  Gyro_Y=Gyro_Y/16.4;                                    //陀螺仪量程转换	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Y,-Gyro_Y);//卡尔曼滤波	
			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);    //互补滤波
	    Angle_Balance=angle;                                   //更新平衡倾角
			Gyro_Turn=Gyro_Z;                                      //更新转向角速度
	  	}
}
