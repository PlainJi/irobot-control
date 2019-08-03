#include "show.h"
#include "MiniBalance.h"

void DataScope(void) {
  char SendCount = 0, i = 0;
  // DataScope_Get_Channel_Data((float)Voltage, 1);
  // DataScope_Get_Channel_Data((float)DesireL, 2);
  // DataScope_Get_Channel_Data((float)Encoder_Left, 3);
  // DataScope_Get_Channel_Data((float)MotoL, 4);
  // DataScope_Get_Channel_Data((float)(DesireR), 5);
  // DataScope_Get_Channel_Data((float)(Encoder_Right), 6);
  // DataScope_Get_Channel_Data((float)(MotoR), 7);
  // DataScope_Get_Channel_Data(DesireVelocity, 8);
  // DataScope_Get_Channel_Data(DesireAngVelo, 9);
  // DataScope_Get_Channel_Data(velocity_kp, 10);

  DataScope_Get_Channel_Data(Voltage, 1);
  DataScope_Get_Channel_Data(DesireVelocity, 2);
  DataScope_Get_Channel_Data(DesireAngVelo, 3);
  DataScope_Get_Channel_Data(Odom.velocity_linear, 4);
  DataScope_Get_Channel_Data(Odom.velocity_anglar, 5);
  DataScope_Get_Channel_Data(0, 6);
  DataScope_Get_Channel_Data(0, 7);
  DataScope_Get_Channel_Data(0, 8);
  DataScope_Get_Channel_Data(0, 9);
  DataScope_Get_Channel_Data(0, 10);

  SendCount = DataScope_Data_Generate(5);
  for (i = 0; i < SendCount; i++) {
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = DataScope_OutPut_Buffer[i];
  }
}
