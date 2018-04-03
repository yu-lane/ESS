#ifndef __ZLG_ZIGBEE_H
#define	__ZLG_ZIGBEE_H

#include "bsp_usart1.h "

struct dev_info{
	unsigned char Dev_Name[16];
	unsigned char Dev_Pwd[16];
	unsigned char Dev_Mode;
	unsigned char Chan;
	unsigned short int PanID;
	unsigned short int MyAddr;
	unsigned char MyIEEE[8];
	unsigned short int DstAddr;
	unsigned char DstIEEE[8];
	unsigned char Reserve;
	unsigned char PowerLevel;
	unsigned char RetryNum;
	unsigned char TranTimeout;
	unsigned char SerialRate;
	unsigned char SerialDataB;
	unsigned char SerialStopB;
	unsigned char SerialParityB;
	unsigned char Reserve1;
	
};
//unsigned char startflag[3]={0XAB,0XBC,0XCD};
struct zlg_data{
	unsigned char startflag[3];//3 bytr
	unsigned char cmd;//1 byte
	struct dev_info zlgdev;//65 byte
	unsigned char endflag;//2 byte
	
};//sum of 71 byte

union var{
	struct zlg_data zlgConfData;
	unsigned char zlgRawData[74];
};

void zlg_config(void);
void zlg_zigbee_reset(void);
#endif
