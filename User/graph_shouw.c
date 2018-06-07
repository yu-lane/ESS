#include "graph_show.h"


void Send_Data_AHRS(unsigned char sensor_amount,unsigned char data_count)
{
	unsigned char sum,i,j,k;
		printf("%c",0xA5);
		printf("%c",0x5A);
		printf("%c",0x1D);					
		printf("%c",0x80);
		printf("%c",0x0F);
  	sum=0xac;
		for( i=0;i<sensor_amount;i++)
		{
			for( j=0;j<data_count*2;j++)
			{
				sum+=Sensor_data[i][j];
				printf("%c",Sensor_data[i][j]);
			}
		}
		for( k = 0;k<(0x18-sensor_amount*data_count*2);k++)
		{
			printf("%c",0x00);

		}
		printf("%c",sum);
		printf("%c",0xAA);
}

void Send_Data_CVI(unsigned char sensor_amount)
{
	unsigned char i;
	printf("%c",0x88);
	printf("%c",0xFF);
//	for(i=0;i<sensor_amount;i++)
//	{
//		printf("%c",Sensor_data[i][0]);
//		printf("%c",Sensor_data[i][1]);
//	}
//		printf("%c",Sensor_data[0][4]);
//		printf("%c",Sensor_data[0][5]);
	/*body data*/
		for(i=0;i<sensor_amount-2;i++)
		{
			printf("%c",Sensor_data[i][0]);
			printf("%c",Sensor_data[i][1]);
		}
		/*foot data*/
		for(i=0;i<4;i++)
		{
			printf("%c",Sensor_data[5][2*i]);
			printf("%c",Sensor_data[5][2*i+1]);
		}
		/*foot data*/
		for(i=0;i<4;i++)
		{
			printf("%c",Sensor_data[6][2*i]);
			printf("%c",Sensor_data[6][2*i+1]);
		}
		/*acc*/
		printf("%c",Sensor_data[0][4]);
		printf("%c",Sensor_data[0][5]);
		
		printf("%c",Sensor_data[7][4]);
		printf("%c",Sensor_data[7][5]);
		
//		for(i=0;i<6;i++)
//		{
//			printf("%c",Sensor_data[8][i]);
//			
//		}

	
//	for(i=0;i<7-sensor_amount;i++)
//	{
//		printf("%c",0x00);
//		printf("%c",0x00);
//	}
}

void Send_Data_Foot(unsigned char sensor_amount)
{
	unsigned char i;
	printf("%c",0x88);
	printf("%c",0xFF);
	for(i=0;i<sensor_amount;i++)
	{
		printf("%c",Sensor_data[5][2*i]);
		printf("%c",Sensor_data[5][2*i+1]);
	}
	for(i=0;i<sensor_amount;i++)
	{
		printf("%c",Sensor_data[6][2*i]);
		printf("%c",Sensor_data[6][2*i+1]);
	}
}


void Send_Data_ASC(unsigned char sensor_amount,unsigned char data_count)
{
	unsigned char i,j;
	printf("\n");
	for( i=0;i<sensor_amount;i++)
	{
		printf("D%d",i);
		for( j=0;j<data_count*2;j++)
		{
			printf("%x ",Sensor_data[i][j]);
		}
		printf("\t");
	}
}

void dataProcess(unsigned char rawdata[11][8], int data[11][4])
{
	char i,j;
	for(i=0;i<5;i++)
	{
		for(j=0;j<4;j++)
		{
			if(rawdata[i][2*j]>0x7f)
			data[i][j]=rawdata[i][2*j]<<8|rawdata[i][2*j+1]-0x10000;
			else
				data[i][j]=rawdata[i][2*j]<<8|rawdata[i][2*j+1];
		}
	}
	for(i=5;i<7;i++)
	{
		for(j=0;j<4;j++)
		{
				data[i][j]=rawdata[i][2*j]<<8|rawdata[i][2*j+1];
		}
	}
	for(i=7;i<9;i++)
	{
		for(j=0;j<4;j++)
		{
			if(rawdata[i][2*j]>0x7f)
			data[i][j]=rawdata[i][2*j]<<8|rawdata[i][2*j+1]-0x10000;
			else
				data[i][j]=rawdata[i][2*j]<<8|rawdata[i][2*j+1];
		}
	}
	for(i=9;i<11;i++)
	{
		for(j=0;j<4;j++)
		{
				data[i][j]=rawdata[i][2*j]<<8|rawdata[i][2*j+1];
		}
	}
}

int footThresholdMatrix[2][4]={{500,500,500,500},{500,500,500,500}};
int guaiThresholdMatrix[2][2]={{500,500},{500,500}};
void oldZhuRequest( int data[11][4],unsigned char finaldata[18])
{
	char i;
//	unsigned char finaldata[18];
	for(i=0;i<18;i++)
	{
		finaldata[i]=0;
	}
	//身体大腿小腿姿态
	for(i=0;i<5;i++)
	{
		finaldata[i*2]=Sensor_data[i][0];
		finaldata[i*2+1]=Sensor_data[i][1];
	}
	//左拐右拐姿态
	finaldata[10]=Sensor_data[7][0];
	finaldata[11]=Sensor_data[7][1];
	finaldata[12]=Sensor_data[8][0];
	finaldata[13]=Sensor_data[8][1];
	//左脚底
	if(data[5][0]>footThresholdMatrix[0][0])
	{
		finaldata[14]=finaldata[14]|0x01;
	}
	if(data[5][1]>footThresholdMatrix[0][1])
	{
		finaldata[14]=finaldata[14]|0x02;
	}
	if(data[5][2]>footThresholdMatrix[0][2])
	{
		finaldata[14]=finaldata[14]|0x04;
	}
	if(data[5][3]>footThresholdMatrix[0][3])
	{
		finaldata[14]=finaldata[14]|0x08;
	}
	//右脚底
	if(data[6][0]>footThresholdMatrix[1][0])
	{
		finaldata[15]=finaldata[15]|0x01;
	}
	if(data[6][1]>footThresholdMatrix[1][1])
	{
		finaldata[15]=finaldata[15]|0x02;
	}
	if(data[6][2]>footThresholdMatrix[1][2])
	{
		finaldata[15]=finaldata[15]|0x04;
	}
	if(data[6][3]>footThresholdMatrix[1][3])
	{
		finaldata[15]=finaldata[15]|0x08;
	}
	//拐杖压力开关
	if(data[9][0]>guaiThresholdMatrix[0][0])
	{
		finaldata[16]=finaldata[16]|0x01;
	}
	if(data[9][1]>guaiThresholdMatrix[0][1])
	{
		finaldata[16]=finaldata[16]|0x02;
	}
	if(data[10][0]>guaiThresholdMatrix[1][0])
	{
		finaldata[17]=finaldata[17]|0x01;
	}
	if(data[10][1]>guaiThresholdMatrix[1][1])
	{
		finaldata[17]=finaldata[17]|0x02;
	}

	
}
void oldZhuSend(unsigned char finaldata[18])
{
	char i;
		//send
	printf("%c",0x88);
	printf("%c",0xFF);
	for(i=0;i<18;i++)
	{
		printf("%c",finaldata[i]);
	}
}

