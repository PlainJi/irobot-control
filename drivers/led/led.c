
#include "main.h"

void led_init(void) {
  RCC->APB2ENR |= 1 << 3;  //ʹ��PORTBʱ��
  GPIOB->CRH &= 0XFFFFFFF0;
  GPIOB->CRH |= 0X00000003;  // PB8 ������� 50MHZ
}

void Led_Flash(u16 time) {
  static int temp;
  if (temp > time) {
    temp = 0;
  }
  if (++temp == time) {
    LED1 = ~LED1;
    temp = 0;
  }
}
