#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

#define LED_PIN0 GPIO_Pin_0
#define LED_PIN1 GPIO_Pin_1
#define LED_PORT GPIOB

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_7)
/*
#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOF,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOF,GPIO_Pin_7)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOF,GPIO_Pin_8);\
					else		\
					GPIO_ResetBits(GPIOF,GPIO_Pin_8)
*/

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� 
#define LED1_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_8)
#define LED1_OFF		digitalHi(GPIOA,GPIO_Pin_8)
#define LED1_ON			digitalLo(GPIOA,GPIO_Pin_8)
*//**/
#define LED2_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_0)
#define LED2_OFF		digitalHi(GPIOB,GPIO_Pin_0)
#define LED2_ON			digitalLo(GPIOB,GPIO_Pin_0)

#define LED3_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_1)
#define LED3_OFF		digitalHi(GPIOB,GPIO_Pin_1)
#define LED3_ON			digitalLo(GPIOB,GPIO_Pin_1)

void LED_GPIO_Config(void);
void LED_Delay(__IO uint32_t nCount);

#endif /* __LED_H */
