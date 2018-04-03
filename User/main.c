/**
  ******************************************************************************
  * @file    main.c
  * @author  LANE
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ����led
  ******************************************************************************

  ******************************************************************************
  */ 
#include "include.h"

void MPU9250IMU(void);
void SendDataProcess(void);


unsigned int systime=0;
unsigned int preview_time=0;
int key_addr_raw=0;//���뿪�ػ�õ�ԭʼ����
unsigned int can_addr,radio_info[2];//CAN���ߵĵ�ַ��ZIGBEE�����ߵ����ò�����ͨ���ź�PANID
 CanTxMsg TxMessage;//CAN���߷��͵�����
 CanRxMsg RxMessage;//CAN���߽��յ�����
unsigned char sendBufAtt[8]={0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80};//�������������̬����
unsigned char sendBufVol[8];//ADC��ѹ�ɼ�����
union var ZLG;//������ZIGBEE�����ý��ջ�����
union combine MPU9250D;//MPU9250��ȡ��ԭʼ9������
union combine MPU_Static_Err;
extern __IO uint16_t ADC_ConvertedValue[ADC_NUMOFCHANNEL];//ԭʼADCת���õ�������
unsigned char zlg_zigbee_flag=0;

struct Axis_Euler Angle_m;
struct Axis_Euler Gyro_m;
struct Axis_Euler Angle;
float Resistance[4];
//��ʱʹ�õ�һЩ����
char runflag =5;

int main(void)
{
	LED_GPIO_Config();//LED��ʼ��
	Key_Init();//���뿪�س�ʼ��
	USART1_Config(115200);//���ڳ�ʼ��
	NVIC_Configuration();//�����ж�����
	CAN_Config();//CAN���ȳ�ʼ��
	key_addr_raw = Get_Key_Addr();//��ȡ���뿪�����õĵ�ַ
	Addr_Process(key_addr_raw,&can_addr,radio_info);//�ѵ�ַ�ֱ�����CAN��ZIGBEE
	
	MPU9250_GPIO_Config();//��MPU9250��ص�IO��ʼ��
	MPU9250_Init();//MPU9250������
	IMU_init();
	ADCx_Init();//ADC��ʼ����DMA����
	
//	zlg_config();
//	zlg_zigbee_reset();
//	USART1_Config(115200);
	
	LED_Delay(0xffff);

	MPU_Gyro_Static_Err(MPU_Static_Err.ShortDataArr.ShortGyro);
	TIM2_Configuration();//��ʱ����ʼ��
	TIM2_NVIC_Configuration();//��ʱ���ж�����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);//������ʱ����һ���������	

	while(1)
	{
		if(systime-preview_time>=10)
		{
			preview_time=systime;
			//MPU9250IMU();
			IMU_getYawPitchRoll();
			ADC_Data_Process( ADC_ConvertedValue,Resistance);
			SendDataProcess();
			printf("Angle: X:%.2f\tY:%.2f\tZ:%.2f\r\n",	Angle.Pitch,   Angle.Roll,   Angle.Yaw	);
			LED3_TOGGLE;
			//LED_Delay(0xffff);
		}

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
  // �Ƚ��Ƿ��Ƿ��͵����ݺ�ID 
  if((RxMessage.StdId==can_addr) && (RxMessage.IDE==CAN_ID_STD) )
  {//������̬��Ϣ
		//	����Ҫͨ��CAN���͵���Ϣ	
			CAN_Send_Std_Msg(can_addr,sendBufAtt,6);
  }
	else if((RxMessage.StdId==can_addr+0x010) && (RxMessage.IDE==CAN_ID_STD) )
	{//����ADC��ȡ��ֵ
		CAN_Send_Std_Msg(can_addr+0x010,sendBufVol,8);
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
		//����˵����Ҫ����Ĵ�������ֵ������Զ�����
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
	MPU9250_Get_Data(&MPU9250D);
	Angle_m.Roll=atan2(MPU9250D.DataINT.ACC.y,MPU9250D.DataINT.ACC.z)*57.2957795;//ת��Ϊ�Ƕȵ�λ��
	Angle_m.Pitch=atan2(MPU9250D.DataINT.ACC.x,MPU9250D.DataINT.ACC.z)*57.2957795;
	Angle_m.Yaw=0;  //atan2(MPU9250D.DataINT.MAG.x,MPU9250D.DataINT.MAG.y)*57.2957795;//ʹ�ô���
	
	Gyro_m.Roll=(MPU9250D.DataINT.GYRO.x-MPU_Static_Err.DataINT.GYRO.x)/16.40;   //ת��Ϊ���ٶȵ�λ��ÿ��
	Gyro_m.Pitch=-(MPU9250D.DataINT.GYRO.x-MPU_Static_Err.DataINT.GYRO.x)/16.40;
	Gyro_m.Yaw=-(MPU9250D.DataINT.GYRO.z-MPU_Static_Err.DataINT.GYRO.z)/16.40;
	
	Angle.Roll = Kalman_Filter(Angle_m.Roll,Gyro_m.Roll);
	Angle.Pitch = Kalman_Filter1(Angle_m.Pitch,Gyro_m.Pitch);
	Angle.Yaw=atan2(MPU9250D.DataINT.MAG.x,MPU9250D.DataINT.MAG.y)*57.2957795;
	
}

void SendDataProcess()
{
	//����������������̬
	sendBufAtt[0] = (short)(Angle.Pitch*100)>>8;
	sendBufAtt[1] = (short)(Angle.Pitch*100)&0x00ff;
	sendBufAtt[2] = (short)(Angle.Roll*100)>>8;
	sendBufAtt[3] = (short)(Angle.Roll*100)&0x00ff;
	sendBufAtt[4] = (short)(Angle.Yaw*100)>>8;
	sendBufAtt[5] = (short)(Angle.Yaw*100)&0x00ff;
	sendBufAtt[6] = 0;
	sendBufAtt[7] = 0;
	
	//δ���������ADC���ݣ�������Ӳ����ͨ�˲���ADC�����õ�
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


