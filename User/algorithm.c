#include "algorithm.h"
//#include <math.h>




#define PI 3.141592654


//double norm=0;
float angleAx,angleAy,angleAz,gyroGx,gyroGy,gyroGz;
int gyro_static_err_x,gyro_static_err_y,gyro_static_err_z;


//卡尔曼滤波参数与函数

//俯仰横滚公用参数
float dt=0.01;//注意：dt的取值为kalman滤波器采样时间
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
float R_angle=0.5 ,C_0 = 1; 

//横滚轴计算中间变量，静态数据
float angle, angle_dot;//角度和角速度
//float angle_0, angle_dot_0;//采集来的角度和角速度
//float dt=20*0.001;//注意：dt的取值为kalman滤波器采样时间
//一下为运算中间变量
float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};

float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

//横滚轴
float Kalman_Filter(float angle_m,float gyro_m)
{
	angle+=(gyro_m-q_bias) * dt;
	angle_err = angle_m - angle;
	Pdot[0]=Q_angle - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3]= Q_gyro;
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];
	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	angle += K_0 * angle_err; //
	q_bias += K_1 * angle_err;
	angle_dot = gyro_m-q_bias;//
	return angle;
}


//卡尔曼滤波参数与函数
float angle1, angle_dot1;//角度和角速度
//float angle_0_1, angle_dot_0_1;//采集来的角度和角速度
//float dt=20*0.001;//注意：dt的取值为kalman滤波器采样时间
//一下为运算中间变量
float P1[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot1[4] ={ 0,0,0,0};
//float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
//float R_angle=0.5 ,C_0 = 1; 
float q_bias1, angle_err1, PCt_0_1, PCt_1_1, E_1, K_0_1, K_1_1, t_0_1, t_1_1;
//俯仰轴
float Kalman_Filter1(float angle_m,float gyro_m)
{
	angle1+=(gyro_m-q_bias1) * dt;
	angle_err1 = angle_m - angle1;
	Pdot1[0] =Q_angle - P1[0][1] - P1[1][0];
	Pdot1[1] = - P1[1][1];
	Pdot1[2] = - P1[1][1];
	Pdot1[3] = Q_gyro;
	P1[0][0] += Pdot1[0] * dt;
	P1[0][1] += Pdot1[1] * dt;
	P1[1][0] += Pdot1[2] * dt;
	P1[1][1] += Pdot1[3] * dt;
	PCt_0_1 = C_0 * P1[0][0];
	PCt_1_1 = C_0 * P1[1][0];
	E_1 = R_angle + C_0 * PCt_0_1;
	K_0_1 = PCt_0_1 / E_1;
	K_1_1 = PCt_1_1 / E_1;
	t_0_1 = PCt_0_1;
	t_1_1 = C_0 * P1[0][1];
	P1[0][0] -= K_0_1 * t_0_1;
	P1[0][1] -= K_0_1 * t_1_1;
	P1[1][0] -= K_1_1 * t_0_1;
	P1[1][1] -= K_1_1 * t_1_1;
	angle1 += K_0_1 * angle_err1; //????
	q_bias1 += K_1_1 * angle_err1;
	angle_dot1 = gyro_m-q_bias1;//?????
	return angle1;
}

/****************************************************************
*
*?????,??????????,???????????????
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
/*****Q_rsqrt*********************************************
*
*
*
**************************************************
//   ????????
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration (???????)
	return y;
} */

/*	
	Q:????,Q??,??????,???????
	R:????,R??,??????,???????	

#define KALMAN_Q        0.02
#define KALMAN_R        7.0000
//           ????????????????           
static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
   kg=p_mid/(p_mid+R); //kg?kalman filter,R???
   x_now=x_mid+kg*(ResrcData-x_mid);//???????
                
   p_now=(1-kg)*p_mid;//??????covariance       
   p_last = p_now; //??covariance?
   x_last = x_now; //???????
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
   kg=p_mid/(p_mid+R); //kg?kalman filter,R???
   x_now=x_mid+kg*(ResrcData-x_mid);//???????
                
   p_now=(1-kg)*p_mid;//??????covariance       
   p_last = p_now; //??covariance?
   x_last = x_now; //???????
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
   kg=p_mid/(p_mid+R); //kg?kalman filter,R???
   x_now=x_mid+kg*(ResrcData-x_mid);//???????
                
   p_now=(1-kg)*p_mid;//??????covariance       
   p_last = p_now; //??covariance?
   x_last = x_now; //???????
   return x_now;                
 }

 
//  ?float??????
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}

//#define Kp 1.5f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.003f                     // integral gain governs rate of convergence of gyroscope biases
float halfT = 0.005f;                 // ???????  ??? 2.5MS ????  ?? halfT?1.25MS
float Kp = 1.6f; 
float Ki = 0.001f;  
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
struct _angle Structangle;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
//	int16_t Xr,Yr;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // ???????????
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  //  float q0q3 = q0*q3;//
  float q1q1 = q1*q1;
  //  float q1q2 = q1*q2;//
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc?????
  ax = ax *norm;
  ay = ay * norm;
  az = az * norm;

  // estimated direction of gravity and flux (v and w)              ?????????/??
  vx = 2*(q1q3 - q0q2);												//????xyz???
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //???????????????
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //???????
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
	if(FL_ABS(ay) < 0.85 && FL_ABS(az-1) < 0.85)         //????????????  ?????????? ,???????
	{
    gx = gx + Kp*ex + exInt;					   							//???PI???????,???????
  }
	else
	{
    gx = gx; 
  }
	if(FL_ABS(ax) < 0.85 && FL_ABS(az-1) < 0.85)
	{
    gy = gy + Kp*ey + eyInt;
  }
	else
	{
    gy = gy; 
  }
	if(FL_ABS(ax)< 0.85 && FL_ABS(ay) < 0.85)
	{
     gz = gz + Kp*ez + ezInt;					   							 //???gz????????????????,??????????????
  }
	else
	{
    gz = gz; 
  }
									
  // integrate quaternion rate and normalise						   //????????
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  Structangle.roll = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
	Structangle.pitch = asin(-2*q1q3 + 2*q0q2); // pitch
	
	//         ????????????                         
	//??  http://baike.baidu.com/view/1239157.htm?fr=aladdin 
//	Xr = X_HMC * COS(angle.pitch) + Y_HMC * SIN(-angle.pitch) * SIN(-angle.roll) - Z_HMC * COS(angle.roll) * SIN(-angle.pitch);
//	Yr = Y_HMC * COS(angle.roll) + Z_HMC * SIN(-angle.roll);
	
//	Structangle.yaw = atan2((double)Yr,(double)Xr) * RtA; // yaw 
	Structangle.roll *= RtoA;
	Structangle.pitch *= RtoA;

}
*/
int Sample_Ris = 10;
void ADC_Data_Process( unsigned short int *Raw_Data,float Ris_Data[4])
{
	float Vol_Data[4];
/*	
	Vol_Data[0] = Raw_Data[0]*33000/4095;//33000代表参考电压的0.1mv单位的值
	Vol_Data[1] = Raw_Data[1]*33000/4095;//ADC采样值增长1，增长约0.8mv电压
	Vol_Data[2] = Raw_Data[2]*33000/4095;//以33000时，数值增大约为8，以3300时，数值增大约0.8
	Vol_Data[3] = Raw_Data[3]*33000/4095;//用哪个更好？
*/	
	Ris_Data[0] = (4095-Raw_Data[0])*Sample_Ris/Raw_Data[0];//(4095-ADCVAL)*Rsample/ADCVAL
	Ris_Data[1] = (4095-Raw_Data[1])*Sample_Ris/Raw_Data[1];//得到电阻值
	Ris_Data[2] = (4095-Raw_Data[2])*Sample_Ris/Raw_Data[2];//
	Ris_Data[3] = (4095-Raw_Data[3])*Sample_Ris/Raw_Data[3];//
	

}



