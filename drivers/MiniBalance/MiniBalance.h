#ifndef __MINIBALANCE_H
#define __MINIBALANCE_H
#include "main.h"
#include "filter.h"
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void);  
int balance(float angle,float gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
void Set_Pwm(int moto1,int moto2);

void readEncoder(void);

void Xianfu_Pwm(void);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
#endif
