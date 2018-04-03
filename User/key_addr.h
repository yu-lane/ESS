#ifndef __KEY_ADDR_H
#define	__KEY_ADDR_H

#include "stm32f10x.h"

#define KEY_PORT GPIOB
#define KEY_PIN1 GPIO_Pin_5
#define KEY_PIN2 GPIO_Pin_6
#define KEY_PIN3 GPIO_Pin_7
#define KEY_PIN4 GPIO_Pin_8
#define KEY_PIN5 GPIO_Pin_9
#define KEY_PIN6 GPIO_Pin_10


void Key_Init(void );
int Get_Key_Addr(void);
void Addr_Process(unsigned int addr,unsigned int *CAN_ID,unsigned int wireless_setting[2]);

#endif
