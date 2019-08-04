#include "MiniBalance.h"
#include "led.h"
#include <math.h>
#include "mpu6050.h"

#define PI 3.14159265
Odometry Odom;

void TIM1_UP_TIM16_IRQHandler(void) {
  if (TIM1->SR & 0X0001)  // 5ms��ʱ�ж�
  {
    TIM1->SR &= ~(1 << 0);  //===�����ʱ��1�жϱ�־λ
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

  // �ٶ�
  DesireL = DesireVelocity * SampleTime * Unit;
  DesireR = DesireVelocity * SampleTime * Unit;
  // ���ٶ� rad/s
  // DesireAngVelo ƽ�ָ������֣�ÿ��ת���Ļ��� theta = DesireAngVelo / 2.0
  // ����ʱ����ת���Ļ��� theta1 = theta * SampleTime
  // sin(theta1) = d / (wheelbase/2)
  // ÿ������Ҫ�˶��ľ��� d = sin(theta) * (wheelbase/2) = theta * wheelbase / 2
  // �������ļ���ֵ DiffDis = d * Unit

  // ����Ҫת���ĽǶ� theta = DesireAngVelo * SampleTime
  // Theta = dis / base   dis = theta * base
  // ÿ�������ƶ����� d = dis/2 = theta * base / 2
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

  // �ٶ�
  DesireL = DesireVelocity * SampleTime * Unit;
  DesireR = DesireVelocity * SampleTime * Unit;
  // ���ٶ� rad/s
  // DesireAngVelo ƽ�ָ������֣�ÿ��ת���Ļ��� theta = DesireAngVelo / 2.0
  // ����ʱ����ת���Ļ��� theta1 = theta * SampleTime
  // sin(theta1) = d / (wheelbase/2)
  // ÿ������Ҫ�˶��ľ��� d = sin(theta) * (wheelbase/2) = theta * wheelbase / 2
  // �������ļ���ֵ DiffDis = d * Unit
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
  u16 Encoder_L, Encoder_R;  //===���ұ��������������
  Encoder_R = TIM4->CNT;     //===��ȡ��������1����
  TIM4->CNT = 0;             //===����������
  Encoder_L = TIM2->CNT;     //===��ȡ��������2����
  TIM2->CNT = 0;             //===����������
  if (Encoder_L > 32768)
    Encoder_Left = Encoder_L - 65000;
  else
    Encoder_Left = Encoder_L;
  //=��������ԭ���ǣ���������0�������65000���¼��������������������ڿ��Ƴ�����ʹ��
  if (Encoder_R > 32768)
    Encoder_Right = Encoder_R - 65000;
  else
    Encoder_Right = Encoder_R;
  //����ȡ������Ϊ��ƽ��С���������������ת��180�Ȱ�װ�ģ�Ϊ�˱�֤ǰ������ʱ��ı��������ݷ���һ��
  Encoder_Left = -Encoder_Left;
}

void Xianfu_Pwm(void) {
  int Amplitude = 3500;  //===PWM������3600 ������3500
  if (MotoL < -Amplitude) MotoL = -Amplitude;
  if (MotoL > Amplitude) MotoL = Amplitude;
  if (MotoR < -Amplitude) MotoR = -Amplitude;
  if (MotoR > Amplitude) MotoR = Amplitude;
}

// void Get_Angle(u8 way)
// {
// 	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
// 	    if(way==1) //DMPû���漰���ϸ��ʱ�����⣬����������ȡ
// 			{
// 			}
//       else
//       {
// 			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);
// //��ȡY��������
// 			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);
// //��ȡZ��������
// 		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L);
// //��ȡX����ٶȼ�
// 	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L);
// //��ȡZ����ٶȼ� 		  if(Gyro_Y>32768)  Gyro_Y-=65536;
// //��������ת�� Ҳ��ͨ��shortǿ������ת�� 			if(Gyro_Z>32768)
// Gyro_Z-=65536;     //��������ת�� 	  	if(Accel_X>32768)
// Accel_X-=65536;
// //��������ת�� 		  if(Accel_Z>32768) Accel_Z-=65536;
// //��������ת�� 			Gyro_Balance=-Gyro_Y; //����ƽ����ٶ�
// 	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI; //���������ļн�
// 		  Gyro_Y=Gyro_Y/16.4; //����������ת��
//       if(Way_Angle==2)
//       Kalman_Filter(Accel_Y,-Gyro_Y);//�������˲�
// 			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);
// //�����˲� 	    Angle_Balance=angle; //����ƽ�����
// Gyro_Turn=Gyro_Z;
// //����ת����ٶ�
// 	  	}
// }

void CalOdom(void) {
  int EncoderLeft =0, EncoderRight = 0;
  float DisLeft=0, DisRight=0, Distance=0, DistanceDiff=0, Theta=0, r=0;

  if (MotoL < 0) EncoderLeft = -Encoder_Left;
  else EncoderLeft = Encoder_Left;
  if (MotoR < 0) EncoderRight = -Encoder_Right;
  else EncoderRight = Encoder_Right;
  DisLeft = (float)EncoderLeft / Unit;            //������ʻ�ľ��룬m
  DisRight = (float)EncoderRight / Unit;          //������ʻ�ľ��룬m
  DistanceDiff = DisRight - DisLeft;              //������ʻ�ľ���m
  Distance = (DisLeft + DisRight) / 2.0;          //����ƽ����ʻ���룬m
  Odom.velocity_linear = Distance / SampleTime;   //С�����ٶȣ�m/s
  Theta = DistanceDiff / Wheelbase;               //С��ת��ǣ�rad  ���Ⱥ�Сʱ���� �� sin(��)
  Odom.velocity_anglar = Theta / SampleTime;      //���ٶȣ�rad/s
  Odom.oriention += Theta;                        //����ǣ�rad
  if (Odom.oriention > PI) Odom.oriention -= 2*PI;
  if (Odom.oriention < -PI) Odom.oriention += 2*PI;
  r = Distance / Theta;                           //ת��뾶
  Odom.x += r * Theta;                            //С��x����
  Odom.y += r * (1-cos(Theta));                   //С��y����
}

