/**
  ******************************************************************************
  * @file    main.c
  * @author  LANE
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   测试led
  ******************************************************************************

  ******************************************************************************
  */ 
#include "include.h"

//#define LAB7_AHRS
void MPU9250IMU(void);
void SendDataProcess(void);
void ADC_Data_Process( unsigned short int *Raw_Data,float Ris_Data[4]);

unsigned int systime=0;
unsigned int preview_time=0;
int key_addr_raw=0;//拨码开关获得的原始数据
unsigned int can_addr,radio_info[2];//CAN总线的地址和ZIGBEE的无线电配置参数即通道号和PANID
 CanTxMsg TxMessage;//CAN总线发送的数据
 CanRxMsg RxMessage;//CAN总线接收的数据
unsigned char sendBufAtt[8]={0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80};//最后结算出来的姿态数据
unsigned char sendBufVol[8];//ADC电压采集数据
union var ZLG;//周立功ZIGBEE的配置接收缓冲区
union combine MPU9250D;//MPU9250获取的原始9轴数据
union combine MPU_Static_Err;
extern __IO uint16_t ADC_ConvertedValue[ADC_NUMOFCHANNEL];//原始ADC转换得到的数据
unsigned char zlg_zigbee_flag=0;

struct Axis_Euler Angle_m;
struct Axis_Euler Gyro_m;
struct Axis_Euler Angle;
float Resistance[4];
//临时使用的一些参数
char runflag =5;

int main(void)
{
	LED_GPIO_Config();//LED初始化
	Key_Init();//拨码开关初始化
	USART1_Config(115200);//串口初始化
	NVIC_Configuration();//串口中断配置
	CAN_Config();//CAN总先初始化
	key_addr_raw = Get_Key_Addr();//读取拨码开关设置的地址
	Addr_Process(key_addr_raw,&can_addr,radio_info);//把地址分别处理给CAN和ZIGBEE
	
	MPU9250_GPIO_Config();//与MPU9250相关的IO初始化
	MPU9250_Init();//MPU9250的设置

	IMU_init();

	ADCx_Init();//ADC初始化，DMA传输
	
//	zlg_config();
//	zlg_zigbee_reset();
//	USART1_Config(115200);
	
	LED_Delay(0xffff);

	MPU_Gyro_Static_Err(MPU_Static_Err.ShortDataArr.ShortGyro);
	TIM2_Configuration();//定时器初始化
	TIM2_NVIC_Configuration();//定时器中断设置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);//开启定时器，一般在最后开启	

	while(1)
	{
//		if(systime-preview_time>=10)
//		{
//			preview_time=systime;
			
			IMU_getYawPitchRoll();
//			printf("IMU: X:%.2f\tY:%.2f\t",	Angle.Pitch,   Angle.Roll);
			MPU9250IMU();
//			printf("Angle: X:%.2f\tY:%.2f\tX:%.2f\tY:%.2f\r\n",	Angle.Pitch,   Angle.Roll,  Angle_m.Pitch, Angle_m.Roll 	);

			ADC_Data_Process( ADC_ConvertedValue,Resistance);
			SendDataProcess();
//			printf("Angle: X:%.2f\tY:%.2f\tZ:%.2f\r\n",	Angle.Pitch,   Angle.Roll,   Angle.Yaw	);
			LED3_TOGGLE;
			//LED_Delay(0xffff);
//		}

	}
}




/**************************************************************/
/*
 *CAN BUS INTERRUPT FUNCTION
 *
 *
 **************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{

  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  // 比较是否是发送的数据和ID 
  if((RxMessage.StdId==can_addr) && (RxMessage.IDE==CAN_ID_STD) )
  {//发送姿态信息
		//	设置要通过CAN发送的信息	
			CAN_Send_Std_Msg(can_addr-0x100,sendBufAtt,6);
  }
	else if((RxMessage.StdId==can_addr+0x010) && (RxMessage.IDE==CAN_ID_STD) )
	{//发送ADC读取的值
		CAN_Send_Std_Msg(can_addr+0x010-0x100,sendBufVol,8);
	}
	else if((RxMessage.StdId==can_addr+0x020) && (RxMessage.IDE==CAN_ID_STD) )
	{
		LED2_TOGGLE;
	}

	
}

/******************************************************************************
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  ****************************************************************************/
void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		
		systime++;
		//按理说不需要下面的处理，数值溢出会自动回零
//		if(systime==0xffffffff)
//		{
//			systime=0;
//		}
//		else
//		{
//			systime++;
//		}
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);  		 
	}		 	
}

/**/
//char Usart_flag,GET_COUNT;
u8 Uart1_get_count=0;
unsigned char Uart1_buf[80];
void USART1_IRQHandler(void)
{
  unsigned char i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    
		Uart1_buf[Uart1_get_count]  = USART_ReceiveData(USART1);
		Uart1_get_count++;
		if(Uart1_get_count<4)
		{
			if(Uart1_buf[0]!=0xAB){Uart1_get_count=0;}
			if(Uart1_buf[1]==0xAB){Uart1_get_count=1;}
			if(Uart1_buf[1]!=0xBC&&Uart1_get_count==2){Uart1_get_count=0;}
			if((Uart1_buf[2]!=0xCD)&&(Uart1_get_count==3)){Uart1_get_count=0;}
		}
		if(Uart1_get_count>=74)
		{
			for(i=0;i<74;i++)
			{
				ZLG.zlgRawData[i] = Uart1_buf[i];
				
			}
			zlg_zigbee_flag=1;
		}
	}
}

/******************************************************************************
  * @brief  This function use in IMU
  * @param  None
  * @retval None
  ****************************************************************************/

void MPU9250IMU()
{
//	MPU9250_Get_Data(&MPU9250D);
	Angle_m.Roll=atan2(MPU9250D.DataINT.ACC.y,MPU9250D.DataINT.ACC.z)*57.2957795;//转换为角度单位度
	Angle_m.Pitch=atan2(MPU9250D.DataINT.ACC.x,MPU9250D.DataINT.ACC.z)*57.2957795;
	Angle_m.Yaw=0;  //atan2(MPU9250D.DataINT.MAG.x,MPU9250D.DataINT.MAG.y)*57.2957795;//使用待定
	
	Gyro_m.Roll=(MPU9250D.DataINT.GYRO.x-MPU_Static_Err.DataINT.GYRO.x)/16.40;   //转换为角速度单位度每秒
	Gyro_m.Pitch=-(MPU9250D.DataINT.GYRO.y-MPU_Static_Err.DataINT.GYRO.y)/16.40;
	Gyro_m.Yaw=-(MPU9250D.DataINT.GYRO.z-MPU_Static_Err.DataINT.GYRO.z)/16.40;
	
	Angle.Roll = Kalman_Filter(Angle_m.Roll,Gyro_m.Roll);
	Angle.Pitch = Kalman_Filter1(Angle_m.Pitch,Gyro_m.Pitch);
	Angle.Yaw=atan2(MPU9250D.DataINT.MAG.x,MPU9250D.DataINT.MAG.y)*57.2957795;
	
}
/****************************************************************
*
*
***************************************************************
float Roll_err=0,Pitch_err=0,gyro_err=0;//??????????
float Roll, Pitch;
float Roll_pre,Pitch_pre, gyro_x_pre,gyro_y_pre;
void Kalman_IMU(int accx,int accy,int accz,float gyro_x,float gyro_y)
{
	

	Roll=atan2(accy,accz)*57.2957795;//????????
	Pitch=atan2(accx,accz)*57.2957795;
	gyroGx=(gyro_x-gyro_static_err_x)/16.40;         //???????????
	gyroGy=-(gyro_y-gyro_static_err_y)/16.40;
	
	if(((Roll-Roll_pre>Roll_err)||(Roll_pre-Roll>Roll_err))\
		&&((gyroGx>gyro_err)||(gyroGx<-gyro_err)))
	{
		Roll_pre  = Roll;
		Kalman_Filter(Roll,gyroGx);
	}
	else
	{
		Kalman_Filter(Roll_pre,gyroGx);
	}
	
	if(((Pitch-Pitch_pre>Pitch_err)||(Pitch_pre-Pitch>Pitch_err))\
		&&((gyroGy>gyro_err)||(gyroGy<-gyro_err)))
	{
		Pitch_pre = Pitch;
		Kalman_Filter1(Pitch,gyroGy);
	}
	else
	{
		Kalman_Filter1(Pitch_pre,gyroGy);
	}

}*/
/******************************************************************************
  * @brief  This function use in dataprocess
  * @param  None
  * @retval None
  ****************************************************************************/
void SendDataProcess()
{
	//经过计算后的最终姿态
	sendBufAtt[0] = (short)(Angle.Pitch*100)>>8;
	sendBufAtt[1] = (short)(Angle.Pitch*100)&0x00ff;
	sendBufAtt[2] = (short)(Angle.Roll*100)>>8;
	sendBufAtt[3] = (short)(Angle.Roll*100)&0x00ff;
	sendBufAtt[4] = (short)(Angle.Yaw*100)>>8;
	sendBufAtt[5] = (short)(Angle.Yaw*100)&0x00ff;
	sendBufAtt[6] = 0;
	sendBufAtt[7] = 0;
	
	//未经过计算的ADC数据，数据由硬件低通滤波后ADC采样得到
	sendBufVol[0] = (unsigned short)Resistance[0]>>8;
	sendBufVol[1] = (unsigned short)Resistance[0]&0x00ff;
	sendBufVol[2] = (unsigned short)Resistance[1]>>8;
	sendBufVol[3] = (unsigned short)Resistance[1]&0x00ff;
	sendBufVol[4] = (unsigned short)Resistance[2]>>8;
	sendBufVol[5] = (unsigned short)Resistance[2]&0x00ff;
	sendBufVol[6] = (unsigned short)Resistance[3]>>8;
	sendBufVol[7] = (unsigned short)Resistance[3]&0x00ff;
//	ADC_Data_Process(ADC_ConvertedValue,sendBufVol);
	
}
/******************************************************************************
  * @brief  This function use in ADC value transmit to resistance
  * @param  None
  * @retval None
  ****************************************************************************/
int Sample_Ris = 10;
void ADC_Data_Process( unsigned short int *Raw_Data,float Ris_Data[4])
{
/*	float Vol_Data[4];
	
	Vol_Data[0] = Raw_Data[0]*33000/4095;//33000???????0.1mv????
	Vol_Data[1] = Raw_Data[1]*33000/4095;//ADC?????1,???0.8mv??
	Vol_Data[2] = Raw_Data[2]*33000/4095;//?33000?,??????8,?3300?,?????0.8
	Vol_Data[3] = Raw_Data[3]*33000/4095;//??????
*/	
	Ris_Data[0] = (4095-Raw_Data[0])*Sample_Ris/Raw_Data[0];//(4095-ADCVAL)*Rsample/ADCVAL
	Ris_Data[1] = (4095-Raw_Data[1])*Sample_Ris/Raw_Data[1];//?????
	Ris_Data[2] = (4095-Raw_Data[2])*Sample_Ris/Raw_Data[2];//
	Ris_Data[3] = (4095-Raw_Data[3])*Sample_Ris/Raw_Data[3];//
	

}

