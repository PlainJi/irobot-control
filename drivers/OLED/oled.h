#ifndef __OLED_H
#define __OLED_H			  	 
#include "main.h"
#include "stdlib.h"	    
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 0
		    						  
//-----------------OLED端口定义----------------  					   

#define OLED_CS_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_6)
#define OLED_CS_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_6)

#define OLED_RST_Clr() PCout(13)=0
#define OLED_RST_Set() PCout(13)=1

#define OLED_RS_Clr() PBout(9)=0
#define OLED_RS_Set() PBout(9)=1

#define OLED_WR_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_14)
#define OLED_WR_Set() GPIO_SetBits(GPIOG,GPIO_Pin_14)

#define OLED_RD_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_13)
#define OLED_RD_Set() GPIO_SetBits(GPIOG,GPIO_Pin_13)




//PC0~7,作为数据线
#define DATAOUT(x) GPIO_Write(GPIOC,x);//输出  
//使用4线串行接口时使用 

#define OLED_SCLK_Clr()  PCout(15)=0
#define OLED_SCLK_Set()  PCout(15)=1

#define OLED_SDIN_Clr()  PCout(14)=0
#define OLED_SDIN_Set()  PCout(14)=1

#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
void JTAG_Set(u8 mode);							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	 
void OLED_ShowFloat(u8 x,u8 y,float num);
#endif  
	 



