#ifndef __ADC_H
#define __ADC_H	

#include "stm32f10x.h"

u16 Get_Adc(u8 ch);
void Adc_Init(void);
void Get_battery_volt(void); 
 
#endif 
