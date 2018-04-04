/* ahrs.c file
编写者：AHRS的其中一位使用者，
整理:lisn3188
网址：www.chiplab7.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2013-04-25
测试： 本程序已在第七实验室的mini AHRS上完成测试
功能：
	使用扩展卡尔曼融合9轴数据。

本程序是一个客户无私提供的。非第七实验室编写，在得到他本人同意后，在此开放仅做学习用途。
程序参考的文档见工程目录下的PDF文件：[INSGPSAlg.pdf]
------------------------------------
*/

#include "ahrs.h"
//#ifdef LAB7_AHRS
extern union combine MPU9250D;
extern union combine MPU_Static_Err;
extern struct Axis_Euler Angle;

volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0, q1, q2, q3,w1,w2,w3; // 全局四元数
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
float f;
volatile float Ya_offset=0,P_offset=0,R_offset=0;
float P[49]={	0.0001,0,0,0,0,0,0,
							0,0.0001,0,0,0,0,0,
							0,0,0.0001,0,0,0,0,
							0,0,0,0.0001,0,0,0,
							0,0,0,0,0.0002,0,0,
							0,0,0,0,0,0.0002,0,
							0,0,0,0,0,0,0.0002};

  float Q[49]={0.0001,0,0,0,0,0,0,
               0,0.0001,0,0,0,0,0,
							 0,0,0.0001,0,0,0,0,
							 0,0,0,0.0001,0,0,0,
							 0,0,0,0,0.0005,0,0,		 
							 0,0,0,0,0,0.0005,0,	 
							 0,0,0,0,0,0,0.0005} ;  
			    
float R[36]={0.0003,0,0,0,0,0,
             0,0.0003,0,0,0,0,
						 0,0,0.0003,0,0,0,
						 0,0,0,0.0002,0,0,
						 0,0,0,0,0.0002,0,
						 0,0,0,0,0,0.0002} ;	
			   		
float A[49],B[49],E[42],F1[36],X[49],Z[49],Ht[42],Ft[49],K[42],O[49],T[6],F[49],Y[7],P1[49],U1[36],U1t[36],D1[36],X1[36],X2[36];
float H[42]={
			   0,0,0,0,0,0,0,
			   0,0,0,0,0,0,0,
			   0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,													   												  
			   0,0,0,0,0,0,0,
							  };
float I[49]={1,0,0,0,0,0,0,
               0,1,0,0,0,0,0,
			   0,0,1,0,0,0,0,
			   0,0,0,1,0,0,0,
			   0,0,0,0,1,0,0,
			   0,0,0,0,0,1,0,
			   0,0,0,0,0,0,1
			   };

/**************************实现函数********************************************
*函数原型:		void Initial_Timer3(void)
*功　　能:	  初始化Tim2  Tim3 将两个定时器级联，以产生一个32位的定时器来提供系统us 级的计时	
输入参数：无
输出参数：没有	
******************************************************************************
void Initial_Timer3(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 
	// TIM2 configuration 
  // Time Base configuration 基本配置 配置定时器的时基单元
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值         
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
  
  TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Update);
//   Disable the TIM2 Update event 
  TIM_UpdateDisableConfig(TIM2, ENABLE);
  //----------------------TIM2 Configuration as slave for the TIM3 ----------//
  // Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2//
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
  // Use the External Clock as TIM2 Slave Mode //
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
  //Enable the TIM2 Master Slave Mode //
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);	
	// 定时器配置:
//	1.设置定时器最大计数值 50000
//	2.设置时钟分频系数：TIM_CKD_DIV1
//	3. 设置预分频：  1Mhz/50000= 1hz 
//	4.定时器计数模式  向上计数模式	 
  	TIM_TimeBaseStructure.TIM_Period = 0xffff;     
  	TIM_TimeBaseStructure.TIM_Prescaler = 72;	 //1M 的时钟  
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//应用配置到TIM3 
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	// 使能TIM3重载寄存器ARR
  	TIM_ARRPreloadConfig(TIM3, ENABLE);	

	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
	//----------------------TIM3 Configuration as Master for the TIM2 -----------//
  	// Use the TIM3 Update event  as TIM3 Trigger Output(TRGO) //
  	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  	// Enable the TIM3 Master Slave Mode //
  	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

  	//启动定时器
	TIM_Cmd(TIM3, ENABLE); 
  	TIM_Cmd(TIM2, ENABLE);                  
}*/

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
******************************************************************************
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM2->CNT; //读高16位时间
 	temp = temp<<16;
 	temp += TIM3->CNT; //读低16位时间
 	return temp;
}*/

/**************************实现函数********************************************
*函数原型:	   void AHRS_init(void)
*功　　能:	  初始化IMU相关	
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void AHRS_init(void)
{
//	Initial_Timer3();
//	MPU6050_initialize();
//	HMC5883L_SetUp();
//	delay_ms(50);
//	MPU6050_initialize();
//	HMC5883L_SetUp();
//	BMP180_init();

  	//陀螺仪偏差
	w1=0;//0.095f;
	w2=0;//0.078f;
	w3=0;//-0.014f;
	
//  	lastUpdate = micros();//更新时间
//  	now = micros();

    q0=1.0;
    q1=0;
    q2=0;
    q3=0;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
输出参数：没有
*******************************************************************************/

void IMU_getValues(float * values) {  

		//读取加速度和陀螺仪的当前ADC
	MPU9250_Get_Data(&MPU9250D);
//    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
  values[0] = (float) MPU9250D.DataINT.ACC.x;
	values[1] = (float) MPU9250D.DataINT.ACC.y;
	values[2] = (float) MPU9250D.DataINT.ACC.z;
	values[3] = (float) (MPU9250D.DataINT.GYRO.x-MPU_Static_Err.DataINT.GYRO.x)/16.40;
	values[4] = (float) (MPU9250D.DataINT.GYRO.y-MPU_Static_Err.DataINT.GYRO.y)/16.40;
	values[5] = (float) (MPU9250D.DataINT.GYRO.z-MPU_Static_Err.DataINT.GYRO.z)/16.40;
	values[6] = (float) MPU9250D.DataINT.MAG.x;
	values[7] = (float) MPU9250D.DataINT.MAG.y;
	values[8] = (float) MPU9250D.DataINT.MAG.z;

}


/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/



void AHRS_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float g=9.79973;
  float Ha1,Ha2,Ha3,Ha4,Hb1,Hb2,Hb3,Hb4;
  float e1,e2,e3,e4,e5,e6;
  float halfT=0.05;


// 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;      
  //石家庄地区磁场 
  bx = 0.5500;
  bz = 0.8351; 
//  now = micros();  //读取时间
//  if(now<lastUpdate){ //定时器溢出过了。
//  halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);
//  }
//  else	{
//  halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//  }
//  lastUpdate = now;	//更新时间
   norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm*g;
  ay = ay * norm*g;
  az = az * norm*g;

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
  
  gx=gx-w1;gy=gy-w2;gz=gz-w3;


  Ha1=(-q2)*g; Ha2=q3*g;Ha3=-q0*g;Ha4=q1*g;	 
  Hb1=bx*q0-bz*q2;
  Hb2=bx*q1+bz*q3;//
  Hb3=-bx*q2-bz*q0;
  Hb4=-bx*q3+bz*q1;
  

  H[0]= Ha1;H[1]= Ha2;H[2]= Ha3;H[3]= Ha4;
  H[7]= Ha4;H[8]=-Ha3;H[9]= Ha2;H[10]=-Ha1;
  H[14]=-Ha3;H[15]=-Ha4;H[16]= Ha1;H[17]= Ha2;
  
  H[21]= Hb1;H[22]= Hb2;H[23]= Hb3;H[24]= Hb4;      
  H[28]= Hb4;H[29]=-Hb3;H[30]= Hb2;H[31]=-Hb1;
  H[35]=-Hb3;H[36]=-Hb4;H[37]= Hb1;H[38]= Hb2;
  
  

  //状态更新
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    // 四元数归一
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
//F阵赋值
F[0]=1;F[8]=1;F[16]=1;F[24]=1;F[32]=1;F[40]=1;F[48]=1;
F[1]=-gx*halfT;F[2]=-gz*halfT;F[3]=-gz*halfT;	F[4]=0; F[5]=0; F[6]=0;
F[7]=gx*halfT;F[9]=gz*halfT;F[10]=-gy*halfT;F[11]=0; F[12]=0; F[13]=0;
F[14]=gy*halfT;F[15]=-gz*halfT;F[17]=gx*halfT;F[18]=0; F[19]=0;F[20]=0;
F[21]=gz*halfT;F[22]=gy*halfT;F[23]=-gx*halfT;F[25]=0; F[26]=0; F[27]=0;
F[28]=0;F[29]=0;F[30]=0;F[31]=0;F[33]=0;F[34]=0;
F[35]=0;F[36]=0;F[37]=0;F[38]=0;F[39]=0;F[41]=0;
F[42]=0;F[43]=0;F[44]=0;F[45]=0;F[46]=0;F[47]=0;
 //卡尔曼滤波
 MatrixMultiply(F,7,7,P,7,7,A );	//A=F*P
 MatrixTranspose(F,7,7,Ft);	  //F转置  F'
 MatrixMultiply(A,7,7,Ft,7,7,B); // B=F*P*F'
 MatrixAdd( B,Q,P1,7,7 );
 MatrixTranspose(H,6,7,Ht);	  //F转置  F'
 MatrixMultiply(P1,7,7,Ht,7,6,E );   //E=P*H'
 MatrixMultiply(H,6,7,E,7,6,F1 ); //	 F1=H*P*H'	6*6
 MatrixAdd(F1,R,X,6,6 );           //X=F1+R	   6*6
 UD(X,6,U1,D1);	   //X的UD分解
 MatrixTranspose(U1,6,6,U1t);	 //U1的转置
 MatrixMultiply(U1,6,6,D1,6,6,X1); //X1=U1*D1
 MatrixMultiply(X1,6,6,U1t,6,6,X2); //X2=U1*D1*U1t 
 MatrixInverse(X2,6,0);	 //X逆 
 MatrixMultiply(E,7,6,X2,6,6,K ); //增益K   7*6

  vx = 2*(q1q3 - q0q2)*g;
  vy = 2*(q0q1 + q2q3)*g;
  vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;
           
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  e1=ax-vx;e2=ay-vy;e3=az-vz;
  e4=mx-wx;e5=my-wy;e6=mz-wz;
 T[0]=e1;T[1]=e2;T[2]=e3;T[3]=e4;T[4]=e5;T[5]=e6;
 MatrixMultiply(K,7,6,T,6,1,Y );   //Y=K*(Z-Y)	7*1
 q0= q0+Y[0];
 q1= q1+Y[1];
 q2= q2+Y[2];
 q3= q3+Y[3];
 w1= w1+Y[4];
 w2= w2+Y[5];
 w3= w3+Y[6];

  
 MatrixMultiply(K,7,6,H,6,7,Z); //Z= K*H		7*7
 MatrixSub(I,Z,O,7,7 );	  //O=I-K*H
 
 MatrixMultiply(O,7,7,P1,7,7,P);
 
  // normalise quaternion
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
float mygetqval[9];	//用于存放传感器转换结果的数组

void AHRS_getQ(float * q) {

  IMU_getValues(mygetqval);	 

  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
 AHRS_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
     
  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void AHRS_getYawPitchRoll(void) {
  float q[4]; //　四元数
  
  AHRS_getQ(q); //更新全局四元数
	Angle.Yaw = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  Angle.Pitch = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  Angle.Roll = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  //if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}
// #endif

//------------------End of File----------------------------
