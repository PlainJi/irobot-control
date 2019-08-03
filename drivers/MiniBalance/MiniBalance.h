#ifndef __MINIBALANCE_H
#define __MINIBALANCE_H
#include "main.h"
#include "filter.h"

typedef struct _odom {
  float velocity_linear;
  float velocity_anglar;
  float oriention;
  float x;
  float y;
}Odometry;

extern Odometry Odom;
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void);  
void pid_velocity_weizhi(void);
void pid_velocity_zengliang(void);
void Set_Pwm(int moto1,int moto2);
void readEncoder(void);
void Xianfu_Pwm(void);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
void CalOdom(void);
#endif
