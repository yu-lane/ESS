#include "key_addr.h"

void Key_Init()
{
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����GPIOB������ʱ��*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 

		/*ѡ��Ҫ���Ƶ�GPIOB����*/															   
		GPIO_InitStructure.GPIO_Pin = KEY_PIN1|KEY_PIN2|KEY_PIN3|KEY_PIN4|KEY_PIN5|KEY_PIN6;	//

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOB*/
		GPIO_Init(KEY_PORT, &GPIO_InitStructure);	
	
}

int Get_Key_Addr(void)
{
	char k1,k2,k3,k4,k5,k6;
	int k;
		
	k1=GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN1);
	k2=GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN2);
	k3=GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN3);
	k4=GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN4);
	k5=GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN5);
	k6=GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN6);
	
	k = 0x3f-(k1+2*k2+4*k3+8*k4+16*k5+32*k6);
	
	return k;
}

void Addr_Process(unsigned int addr,unsigned int *CAN_ID,unsigned int wireless_setting[2])
{
	*CAN_ID = (addr & 0x000f)+0x700;
	wireless_setting[0] = (addr&0x0030)>>4;
	wireless_setting[1] = (addr & 0x000f);
	
}

