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
    // key(100);
    // Get_Angle(Way_Angle);

    readEncoder();
    CalOdom();
    Led_Flash(20);
    Get_battery_volt();
    pid_velocity_weizhi();
    //pid_velocity_zengliang();
    Set_Pwm(MotoL, MotoR);
  }
}

void pid_velocity_weizhi(void) {
  static int IntegralL = 0, IntegralR = 0;
  static int LastErrorL = 0, LastErrorR = 0;
  int ErrorL = 0, ErrorR = 0;
  float DiffDis = 0;

  // 速度
  DesireL = DesireVelocity * SampleTime * Unit;
  DesireR = DesireVelocity * SampleTime * Unit;
  // 角速度 rad/s
  // DesireAngVelo 平分给两个轮，每个转动的弧度 theta = DesireAngVelo / 2.0
  // 采样时间内转动的弧度 theta1 = theta * SampleTime
  // sin(theta1) = d / (wheelbase/2)
  // 每个轮需要运动的距离 d = sin(theta) * (wheelbase/2) = theta * wheelbase / 2
  // 编码器的计数值 DiffDis = d * Unit

  // 本次要转动的角度 theta = DesireAngVelo * SampleTime
  // Theta = dis / base   dis = theta * base
  // 每个轮子移动距离 d = dis/2 = theta * base / 2
  DiffDis = DesireAngVelo * SampleTime * Wheelbase / 2.0 * Unit;
  DesireL -= DiffDis;
  DesireR += DiffDis;
  ErrorL = ((int)DesireL - Encoder_Left);
  ErrorR = ((int)DesireR - Encoder_Right);
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
  float DiffDis = 0;

  // 速度
  DesireL = DesireVelocity * SampleTime * Unit;
  DesireR = DesireVelocity * SampleTime * Unit;
  // 角速度 rad/s
  // DesireAngVelo 平分给两个轮，每个转动的弧度 theta = DesireAngVelo / 2.0
  // 采样时间内转动的弧度 theta1 = theta * SampleTime
  // sin(theta1) = d / (wheelbase/2)
  // 每个轮需要运动的距离 d = sin(theta) * (wheelbase/2) = theta * wheelbase / 2
  // 编码器的计数值 DiffDis = d * Unit
  DiffDis = DesireAngVelo * SampleTime * Wheelbase / 2.0 * Unit;
  DesireL -= DiffDis;
  DesireR += DiffDis;
  ErrorL = ((int)DesireL - Encoder_Left);
  ErrorR = ((int)DesireR - Encoder_Right);
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

void Set_Pwm(int moto1, int moto2) {
  Xianfu_Pwm();
  if (Stop) {
    AIN1 = 0, AIN2 = 0;
    BIN1 = 0, BIN2 = 0;
  } else {
    if (moto1 < 0)
      AIN2 = 1, AIN1 = 0;
    else
      AIN2 = 0, AIN1 = 1;
    PWMA = abs(moto1);

    if (moto2 < 0)
      BIN1 = 0, BIN2 = 1;
    else
      BIN1 = 1, BIN2 = 0;
    PWMB = abs(moto2);
  }
}

void readEncoder(void) {
  u16 Encoder_L, Encoder_R;  //===左右编码器的脉冲计数
  Encoder_R = TIM4->CNT;     //===获取正交解码1数据
  TIM4->CNT = 0;             //===计数器清零
  Encoder_L = TIM2->CNT;     //===获取正交解码2数据
  TIM2->CNT = 0;             //===计数器清零
  if (Encoder_L > 32768)
    Encoder_Left = Encoder_L - 65000;
  else
    Encoder_Left = Encoder_L;
  //=这个处理的原因是：编码器到0后会跳到65000向下计数，这样处理方便我们在控制程序中使用
  if (Encoder_R > 32768)
    Encoder_Right = Encoder_R - 65000;
  else
    Encoder_Right = Encoder_R;
  //这里取反是因为，平衡小车的两个电机是旋转了180度安装的，为了保证前进后退时候的编码器数据符号一致
  Encoder_Left = -Encoder_Left;
}

void Xianfu_Pwm(void) {
  int Amplitude = 3500;  //===PWM满幅是3600 限制在3500
  if (MotoL < -Amplitude) MotoL = -Amplitude;
  if (MotoL > Amplitude) MotoL = Amplitude;
  if (MotoR < -Amplitude) MotoR = -Amplitude;
  if (MotoR > Amplitude) MotoR = Amplitude;
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

void CalOdom(void) {
  int EncoderLeft =0, EncoderRight = 0;
  float DisLeft=0, DisRight=0, Distance=0, DistanceDiff=0, Theta=0, r=0;

  if (MotoL < 0) EncoderLeft = -Encoder_Left;
  else EncoderLeft = Encoder_Left;
  if (MotoR < 0) EncoderRight = -Encoder_Right;
  else EncoderRight = Encoder_Right;
  DisLeft = (float)EncoderLeft / Unit;            //左轮行驶的距离，m
  DisRight = (float)EncoderRight / Unit;          //右轮行驶的距离，m
  DistanceDiff = DisRight - DisLeft;              //两轮行驶的距离差，m
  Distance = (DisLeft + DisRight) / 2.0;          //两轮平均行驶距离，m
  Odom.velocity_linear = Distance / SampleTime;   //小车线速度，m/s
  Theta = DistanceDiff / Wheelbase;               //小车转向角，rad  当θ很小时，θ ≈ sin(θ)
  Odom.velocity_anglar = Theta / SampleTime;      //角速度，rad/s
  Odom.oriention += Theta;                        //朝向角，rad
  if (Odom.oriention > PI) Odom.oriention -= 2*PI;
  if (Odom.oriention < -PI) Odom.oriention += 2*PI;
  r = Distance / Theta;                           //转弯半径
  Odom.x += r * Theta;                            //小车x坐标
  Odom.y += r * (1-cos(Theta));                   //小车y坐标
}

