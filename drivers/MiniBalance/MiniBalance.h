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
void Set_Pwm(void);
void readEncoder(void);
extern void Report(void);
#endif
