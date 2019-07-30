#ifndef __USRAT1_H
#define __USRAT1_H 

#include "main.h"
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
void usart1_init(void);
u8 usart1_receive(void);
void usart1_send(u8 data);
void USART1_IRQHandler(void);

void command1(void);
void command2(void);
void command3(void);
#endif

