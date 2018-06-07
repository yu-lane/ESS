#ifndef _GRAPH_SHOW_H_
#define _GRAPH_SHOW_H_

#include "bsp_usart1.h"

void Send_Data_AHRS(unsigned char sensor_amount,unsigned char data_count);
void Send_Data_CVI(unsigned char sensor_amount);
void Send_Data_Foot(unsigned char sensor_amount);
void Send_Data_ASC(unsigned char sensor_amount,unsigned char data_count);
void dataProcess(unsigned char rawdata[11][8], int data[11][4]);
void oldZhuRequest( int data[11][4],unsigned char finaldata[18]);
void oldZhuSend(unsigned char finaldata[18]);


#endif


