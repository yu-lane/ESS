#ifndef __CAN_H
#define	__CAN_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"


static void CAN_GPIO_Config(void);
static void CAN_NVIC_Config(void);
static void CAN_Mode_Config(void);
static void CAN_Filter_Config(u32 Receive_ID,u32 ID_Mask);
void CAN_Config(void);
void CAN_Send_Std_Msg(int can_addr,unsigned char *data,char datanum);
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);

#endif
