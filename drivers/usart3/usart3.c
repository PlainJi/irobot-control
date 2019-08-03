#include "usart3.h"
#include <string.h>
#include "main.h"

void uart3_init(u32 pclk2, u32 bound) {
  float temp;
  u16 mantissa;
  u16 fraction;
  temp = (float)(pclk2 * 1000000 / 2) / (bound * 16);  //得到USARTDIV
  mantissa = temp;                                     //得到整数部分
  fraction = (temp - mantissa) * 16;                   //得到小数部分
  mantissa <<= 4;
  mantissa += fraction;
  // AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
  RCC->APB2ENR |= 1 << 3;   //使能PORTA口时钟
  RCC->APB1ENR |= 1 << 18;  //使能串口时钟
  GPIOB->CRH &= 0XFFFF00FF;
  GPIOB->CRH |= 0X00008B00;  // IO状态设置
  GPIOB->ODR |= 1 << 10;
  RCC->APB1RSTR |= 1 << 18;     //复位串口1
  RCC->APB1RSTR &= ~(1 << 18);  //停止复位
                                //波特率设置
  USART3->BRR = mantissa;       // 波特率设置
  USART3->CR1 |= 0X200C;        // 1位停止,无校验位.
  //使能接收中断
  USART3->CR1 |= 1 << 8;                     // PE中断使能
  USART3->CR1 |= 1 << 5;                     //接收缓冲区非空中断使能
  MY_NVIC_Init(1, 3, USART3_IRQChannel, 2);  //组2，最低优先级
}

void USART3_IRQHandler(void) {
  static u8 mode_data[13] = "$SVP0000000#";
  static u8 p_w = 0;  //当前字节存储位置
  char uart_receive = 0;

  if (USART3->SR & (1 << 5))  //接收到数据
  {
    uart_receive = USART3->DR;
    if (uart_receive == '$') p_w = 0;
    if (p_w == sizeof(mode_data)) p_w = 0;
    mode_data[p_w++] = uart_receive;

    if (uart_receive == '#' && mode_data[0] == '$') {
      if (!strncmp((const char*)(mode_data + 1), "C5", 2))
        Stop = 1;  //刹车
      else if (!strncmp((const char*)(mode_data + 1), "C2", 2))
        Stop = 0;
      //   else if (!strncmp((const char*)(mode_data + 1), "C2", 2))
      //     Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;  //前
      //   else if (!strncmp((const char*)(mode_data + 1), "C8", 2))
      //     Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;  //后
      //   else if (!strncmp((const char*)(mode_data + 1), "C4", 2))
      //     Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;  //左
      //   else if (!strncmp((const char*)(mode_data + 1), "C6", 2))
      //     Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;  //右
      //   else if (!strncmp((const char*)(mode_data + 1), "CL", 2))
      //     Flag_Show ^= 1;  //开关OLED显示
      //   else if (!strncmp((const char*)(mode_data + 1), "CS", 2))
      //     Flag_Stop = !Flag_Stop;  //停止输出控制
      else if (mode_data[1] == 'S') {
        static float value = 0.0;
        value = (float)((mode_data[5] - '0') * 100000 + (mode_data[6] - '0') * 10000 + (mode_data[7] - '0') * 1000 +
                        (mode_data[8] - '0') * 100 + (mode_data[9] - '0') * 10 + (mode_data[10] - '0')) / 1000.0;
        if (mode_data[4] == '-') {
          value = -value;
        }
        if (!strncmp((const char*)(mode_data + 2), "VP", 2))
          velocity_kp = value;
        else if (!strncmp((const char*)(mode_data + 2), "VI", 2)) 
          velocity_ki = value;
        else if (!strncmp((const char*)(mode_data + 2), "VD", 2))
          velocity_kd = value;
        else if (!strncmp((const char*)(mode_data + 2), "DV", 2))
          DesireVelocity = value;
        else if (!strncmp((const char*)(mode_data + 2), "DA", 2))
          DesireAngVelo = value;
      }
    }
  }
}
