#ifndef __USRAT1_H
#define __USRAT1_H 

#include "main.h"

void usart1_init(void);
u8 usart1_receive(void);
void usart1_send(u8 data);
void USART1_IRQHandler(void);

void command1(void);
void command2(void);
void command3(void);
#endif

