#include "main.h"

char uart1_recv_buf[17] = "$+12345,+12345\n";
char uart1_send_buf[17] = "#+12345,+12345\n";

int fputc(int ch, FILE *f) {
  USART1->DR = (u8)ch;
  while ((USART1->SR & 0X40) == 0);
  return ch;
}

void usart1_init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //ʹ�� UART1 ģ���ʱ��  ʹ�� UART1��Ӧ�����Ŷ˿�PA��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  //����UART1 �ķ�������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //����PA9 Ϊ�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //ˢ��Ƶ��50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //����UART1 �Ľ�������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //����PA10Ϊ��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // UART1������:
  USART_InitStructure.USART_BaudRate = 460800;                 //������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8λ����
  USART_InitStructure.USART_StopBits = USART_StopBits_1;       //һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;          //����żЧ��
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;  //��ʹ��Ӳ��������
  USART_InitStructure.USART_Mode =
      USART_Mode_Rx | USART_Mode_Tx;  //ʹ�ܷ��ͺͽ��չ���
  //Ӧ�����õ�UART1
  USART_Init(USART1, &USART_InitStructure);
  USART_ClearFlag(USART1, USART_FLAG_TC);         //�����1���ֽڶ�ʧ
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //�򿪽����ж�
                                                  //����UART1
  USART_Cmd(USART1, ENABLE);
  //��usart1�ж�
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

u8 usart1_receive(void) {
  while ((USART1->SR & 0x20) == 0);
  return USART1->DR;
}

void usart1_send(u8 data) {
  USART1->DR = data;
  while ((USART1->SR & 0x40) == 0);
}

void USART1_IRQHandler(void) {
  static u8 p_w = 0;
  char uart_receive = 0;
  //if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
  if (USART1->SR & (1 << 5)) {
    uart_receive = USART1->DR;
    if (uart_receive == '$') p_w = 0;
    if (p_w == sizeof(uart1_recv_buf)) p_w = 0;
    uart1_recv_buf[p_w++] = uart_receive;

    if (uart_receive == '\n' && uart1_recv_buf[0] == '$') {
      DesireL = (uart1_recv_buf[2]-'0')*10000 + (uart1_recv_buf[3]-'0')*1000 +
        (uart1_recv_buf[4]-'0')*100 + (uart1_recv_buf[5]-'0')*10 + (uart1_recv_buf[6]-'0');
      if (uart1_recv_buf[1] == '-') {
        DesireL = -DesireL;
      }

      DesireR = (uart1_recv_buf[9]-'0')*10000 + (uart1_recv_buf[10]-'0')*1000 +
        (uart1_recv_buf[11]-'0')*100 + (uart1_recv_buf[12]-'0')*10 + (uart1_recv_buf[13]-'0');
      if (uart1_recv_buf[8] == '-') {
        DesireR = -DesireR;
      }
    }
  }
}

void USART1_ReportEncoder(s16 l, s16 r) {
  u8 cnt = 0;
  u8 send_cnt = 15;

  sprintf(uart1_send_buf, "#%+06d,%+06d\n", l, r);
  while (cnt < send_cnt) {
    USART1->DR = uart1_send_buf[cnt++];
    while ((USART1->SR & 0x40) == 0);
  }
}

void USART1_ReportBattery(int voltage) {
  u8 cnt = 0;
  u8 send_cnt = 7;

  sprintf(uart1_send_buf, "#%+06d\n", voltage);
  while (cnt < send_cnt) {
    USART1->DR = uart1_send_buf[cnt++];
    while ((USART1->SR & 0x40) == 0);
  }
}

