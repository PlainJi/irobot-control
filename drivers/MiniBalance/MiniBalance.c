#include "MiniBalance.h"
#include "led.h"
#include <math.h>
#include "mpu6050.h"

#define PI 3.14159265
Odometry Odom;

void TIM1_UP_TIM16_IRQHandler(void) {
  if (TIM1->SR & 0X0001)  // 5ms定时中断
  {
    TIM1->SR &= ~(1 << 0);  //===清除定时器1中断标志位

    readEncoder();
    report_flag=1;
    pid_velocity_weizhi();
    //pid_velocity_zengliang();
    Set_Pwm();
    Led_Flash(200/led_freq);
  }
}

void pid_velocity_weizhi(void) {
  static int IntegralL = 0, IntegralR = 0;
  static int LastErrorL = 0, LastErrorR = 0;
  int ErrorL = 0, ErrorR = 0;
  int desire_left  = DesireL;
	int desire_right = DesireR;
  if (abs(desire_left)<=70 && abs(desire_right)<=70) {
    led_freq = 1;
  }
 	if (desire_left >  70) {
    desire_left  =  70;
    led_freq=10;
  } else if (desire_left < -70) {
    desire_left  = -70;
    led_freq=10;
  }
	if (desire_right>  70) {
    desire_right =  70;
    led_freq=10;
  } else if (desire_right< -70) {
    desire_right = -70;
    led_freq=10;
  }

  ErrorL = ((int)desire_left - Encoder_Left);
  ErrorR = ((int)desire_right - Encoder_Right);
  if (Stop) {
    MotoL = 0, MotoR = 0;
    IntegralL = 0, IntegralR = 0;
  } else {
    IntegralL += ErrorL;
    IntegralR += ErrorR;
    MotoL = velocity_kp * ErrorL + velocity_ki * IntegralL +
            velocity_kd * (ErrorL - LastErrorL);
    MotoR = velocity_kp * ErrorR + velocity_ki * IntegralR +
            velocity_kd * (ErrorR - LastErrorR);
  }

  LastErrorL = ErrorL;
  LastErrorR = ErrorR;
  if (IntegralL > 360000) IntegralL = 360000;
  if (IntegralL < -360000) IntegralL = -360000;
  if (IntegralR > 360000) IntegralR = 360000;
  if (IntegralR < -360000) IntegralR = -360000;
}

void pid_velocity_zengliang(void) {
  static int PreErrorL = 0, PreErrorR = 0;
  static int LastErrorL = 0, LastErrorR = 0;
  int ErrorL = 0, ErrorR = 0;
  int desire_left  = DesireL;
	int desire_right = DesireR;
	if (abs(desire_left)<=70 && abs(desire_right)<=70) {
    led_freq = 1;
  }
 	if (desire_left >  70) {
    desire_left  =  70;
    led_freq=10;
  } else if (desire_left < -70) {
    desire_left  = -70;
    led_freq=10;
  }
	if (desire_right>  70) {
    desire_right =  70;
    led_freq=10;
  } else if (desire_right< -70) {
    desire_right = -70;
    led_freq=10;
  }

  ErrorL = ((int)desire_left - Encoder_Left);
  ErrorR = ((int)desire_right - Encoder_Right);
  if (Stop) {
    MotoL = 0, MotoR = 0;
  } else {
    MotoL += velocity_kp * (ErrorL - LastErrorL) + velocity_ki * ErrorL +
             velocity_kd * (ErrorL - 2 * LastErrorL + PreErrorL);
    MotoR += velocity_kp * (ErrorR - LastErrorR) + velocity_ki * ErrorR +
             velocity_kd * (ErrorR - 2 * LastErrorR + PreErrorR);
  }

  PreErrorL = LastErrorL;
  PreErrorR = LastErrorR;
  LastErrorL = ErrorL;
  LastErrorR = ErrorR;
}

void Set_Pwm(void) {
  // PWM满幅是3600 限制在3500
  int Amplitude = 3500;
  if (MotoL < -Amplitude) MotoL = -Amplitude;
  if (MotoL > Amplitude) MotoL = Amplitude;
  if (MotoR < -Amplitude) MotoR = -Amplitude;
  if (MotoR > Amplitude) MotoR = Amplitude;

  if (Stop) {
    AIN1 = 0, AIN2 = 0;
    BIN1 = 0, BIN2 = 0;
  } else {
    if (MotoL < 0)
      AIN2 = 1, AIN1 = 0;
    else
      AIN2 = 0, AIN1 = 1;
    PWMA = abs(MotoL);

    if (MotoR < 0)
      BIN1 = 0, BIN2 = 1;
    else
      BIN1 = 1, BIN2 = 0;
    PWMB = abs(MotoR);
  }
}

void readEncoder(void) {
  u16 Encoder_L, Encoder_R;
  Encoder_R = TIM4->CNT;
  TIM4->CNT = 0;
  Encoder_L = TIM2->CNT;
  TIM2->CNT = 0;

  //这个处理的原因是：编码器到0后会跳到65000向下计数，这样处理方便我们在控制程序中使用
  if (Encoder_L > 32768)
    Encoder_Left = -(Encoder_L - 65000);
  else
    Encoder_Left = -Encoder_L;
  if (Encoder_R > 32768)
    Encoder_Right = Encoder_R - 65000;
  else
    Encoder_Right = Encoder_R;

  //这里取反是因为，平衡小车的两个电机是旋转了180度安装的，为了保证前进后退时候的编码器数据符号一致
  //Encoder_Left = -Encoder_Left;
}

// void Get_Angle(u8 way)
// {
// 	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
// 	    if(way==1) //DMP没有涉及到严格的时序问题，在主函数读取
// 			{
// 			}
//       else
//       {
// 			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);
// //读取Y轴陀螺仪
// 			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);
// //读取Z轴陀螺仪
// 		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L);
// //读取X轴加速度记
// 	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L);
// //读取Z轴加速度记 		  if(Gyro_Y>32768)  Gyro_Y-=65536;
// //数据类型转换 也可通过short强制类型转换 			if(Gyro_Z>32768)
// Gyro_Z-=65536;     //数据类型转换 	  	if(Accel_X>32768)
// Accel_X-=65536;
// //数据类型转换 		  if(Accel_Z>32768) Accel_Z-=65536;
// //数据类型转换 			Gyro_Balance=-Gyro_Y; //更新平衡角速度
// 	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI; //计算与地面的夹角
// 		  Gyro_Y=Gyro_Y/16.4; //陀螺仪量程转换
//       if(Way_Angle==2)
//       Kalman_Filter(Accel_Y,-Gyro_Y);//卡尔曼滤波
// 			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);
// //互补滤波 	    Angle_Balance=angle; //更新平衡倾角
// Gyro_Turn=Gyro_Z;
// //更新转向角速度
// 	  	}
// }

void Report(void) {
  static u8 cnt = 0;
  static s16 Encoder_Left_Report = 0;
  static s16 Encoder_Right_Report = 0;

  Encoder_Left_Report += Encoder_Left;
  Encoder_Right_Report += Encoder_Right;
  cnt++;
  if (!(cnt%EncoderReportCnt)) {
    USART1_ReportEncoder(Encoder_Left_Report, Encoder_Right_Report);
    Encoder_Left_Report = 0;
    Encoder_Right_Report = 0;
  }
  if (!(cnt%BatteryReportCnt)) {
    Get_battery_volt();
    USART1_ReportBattery(Voltage);
    cnt = 0;
  }
}
