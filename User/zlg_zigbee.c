#include "zlg_zigbee.h"

//need some zlg zigbee module configuration function
extern union var ZLG;
extern unsigned char zlg_zigbee_flag;
void zlg_config()
{
	unsigned char aa,bb,i;
	printf("%c",0xAB);
	printf("%c",0xBC);
	printf("%c",0xCD);
	printf("%c",0xD1);
	printf("%c",0xAA);
	
	while(zlg_zigbee_flag==0);
	
	ZLG.zlgConfData.zlgdev.SerialRate=0x07;
	aa=ZLG.zlgConfData.zlgdev.MyAddr>>8;
	bb=ZLG.zlgConfData.zlgdev.MyAddr&0x00ff;
	
	printf("%c",0xAB);
	printf("%c",0xBC);
	printf("%c",0xCD);
	printf("%c",0xD6);
	printf("%c",bb);
	printf("%c",aa);
	
	for(i=0;i<65;i++)
	{
		printf("%c",ZLG.zlgRawData[i+4]);
	}
	printf("%c",0xAA);
	
}
void zlg_zigbee_reset()
{
	printf("%c",0xAB);
	printf("%c",0xBC);
	printf("%c",0xCD);
	printf("%c",0xD9);
	printf("%c",0x20);
	printf("%c",0x03);
	printf("%c",0x00);
	printf("%c",0x03);
	printf("%c",0xaa);
}



