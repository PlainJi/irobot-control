#ifndef __LED_H
#define __LED_H

#include "main.h"
// LED�˿ڶ���
#define LED1 PBout(8)  //�͵�ƽ���� �ߵ�ƽϨ�� �����ԭ��ͼ
void led_init(void);
void Led_Flash(u16 time);
#endif
