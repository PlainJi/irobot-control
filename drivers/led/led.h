#ifndef __LED_H
#define __LED_H 
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"
//LED端口定义
#define LED1 PBout(8)  //低电平亮灯 高电平熄灭 详情见原理图
void led_init(void);
void Led_Flash(u16 time);
#endif

