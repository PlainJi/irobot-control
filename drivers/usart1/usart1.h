#ifndef __USRAT1_H
#define __USRAT1_H 

#include "main.h"

void usart1_init(void);
u8 usart1_receive(void);
void usart1_send(u8 data);
void USART1_IRQHandler(void);
extern void USART1_ReportEncoder(s16 l, s16 r);
extern void USART1_ReportBattery(int voltage);
#endif

