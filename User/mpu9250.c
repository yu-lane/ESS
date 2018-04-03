#include "mpu9250.h"
#include "bsp_led.h"
//The slave address of the MPU-9250 is b110100X which is 7 bits long.

#define I2C_nCS_PIN		GPIO_Pin_12			/* IICģʽ�����ӵ�VDDIO��GPIO */
#define I2C_AD0_PIN		GPIO_Pin_14			/* ���Ƶ�ַ������λ */



void MPU9250_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	/* ��GPIOʱ�� */
	//ΪMPU9250IIC��SPI ���Ÿ���ģʽ�µ�IICģʽ������
	GPIO_InitStructure.GPIO_Pin = I2C_nCS_PIN | I2C_AD0_PIN;//IICģʽ�µ���Ӧ��;��Ӧ������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  	/* ������� */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, I2C_nCS_PIN);//nCS��IICģʽ��Ӧ�ӵ�VDDIO
	GPIO_ResetBits(GPIOB, I2C_AD0_PIN);//��ַκ�ӵ�0���ӻ���ַ����Ϊ0xd0
	
	i2c_GPIO_Config();       //iic����������
	
}


void Delay(__IO uint32_t nCount)	 //???????
{
	for(; nCount != 0; nCount--);
}

unsigned char ak9860=0x16;
void MPU9250_Init(void)
{
	 
  //��ʼ��֮ǰ��ʱһ��ʱ�䣬����ʱ���ܻ����
  Delay(0x1ffff);
	//�������״̬
	IIC_WriteReg(MPU9250_SLAVE_ADDR,MPU6050_RA_PWR_MGMT_1,0x00);
	//�����ǲ�����1KHZ
	IIC_WriteReg(MPU9250_SLAVE_ADDR,MPU6050_RA_SMPLRT_DIV,0x07);
	//��ͨ�˲������ã���ֹƵ��1KHZ������5K
	IIC_WriteReg(MPU9250_SLAVE_ADDR,MPU6050_RA_CONFIG,0x06);
	//���ü��ٶȴ�����������2Gģʽ�����Լ�
	IIC_WriteReg(MPU9250_SLAVE_ADDR,MPU6050_RA_ACCEL_CONFIG,0x00);
	//���������ǹ�����2000deg/s�����Լ�
	IIC_WriteReg(MPU9250_SLAVE_ADDR,MPU6050_RA_GYRO_CONFIG,0x18);
	//�����ⲿIICΪbypassģʽ����������ֱ�Ӷ�ȡ
	IIC_WriteReg(MPU9250_SLAVE_ADDR,0x37,0x02);
	
	Delay(0x1ffff);
	//������д�شżƵ����ó���
	IIC_WriteReg(AK8963_SLAVE_ADDR,AK8960_RA_CNTL1,ak9860);
	
}


void MPU9250_Get_Data(union combine *MPU9250Data)
{
	unsigned char buf[14],i;
//	IIC_WriteReg(AK8963_SLAVE_ADDR,AK8960_RA_CNTL1,ak9860);
	IIC_ReadData(MPU9250_SLAVE_ADDR,MPU6050_ACC_OUT,buf,14);
	
	for(i=0;i<7;i++)
	{
		MPU9250Data->DataDev.MPU6050[2*i]=buf[2*i+1];
		MPU9250Data->DataDev.MPU6050[2*i+1]=buf[2*i];
	}
	//the ak8960 have 6 byte data but must read the 0x09 reg that device works narmal
	IIC_ReadData(AK8963_SLAVE_ADDR,AK8960_RA_HXL,MPU9250Data->DataDev.AK8960,7);
}
/*
*�ڳ�����������ʱ��ȡ�����ǵľ�̬���
*
*
*
**/
void MPU_Gyro_Static_Err(short int GyroStaticErr[3])
{
	char i,j,k;
	u8 buf[14];
	union combine MPU9250Tmp;
	int gyro_sum_err_x=0,gyro_sum_err_y=0,gyro_sum_err_z=0;
	
	for(i=0;i<10;i++)
	{
		for(j=0;j<10;j++)
		{
			IIC_ReadData(MPU9250_SLAVE_ADDR,MPU6050_ACC_OUT,buf,14);
			for(k=0;k<7;k++)
			{
				MPU9250Tmp.DataDev.MPU6050[2*k]=buf[2*k+1];
				MPU9250Tmp.DataDev.MPU6050[2*k+1]=buf[2*k];
			}

			if(((MPU9250Tmp.DataINT.GYRO.x<50)&&(MPU9250Tmp.DataINT.GYRO.x>-50))\
				&&((MPU9250Tmp.DataINT.GYRO.y<50)&&(MPU9250Tmp.DataINT.GYRO.y>-50))\
				&&((MPU9250Tmp.DataINT.GYRO.z<50)&&(MPU9250Tmp.DataINT.GYRO.z>-50)))
			{
				gyro_sum_err_x+=MPU9250Tmp.DataINT.GYRO.x;
				gyro_sum_err_y+=MPU9250Tmp.DataINT.GYRO.y;
				gyro_sum_err_z+=MPU9250Tmp.DataINT.GYRO.z;
			}
			Delay(0xffff);
		}
		LED3_TOGGLE;
		if(i==9)
		{
			GyroStaticErr[0]=gyro_sum_err_x/100;
			GyroStaticErr[1]=gyro_sum_err_y/100;
			GyroStaticErr[2]=gyro_sum_err_z/100;
		}
	}
	
}












