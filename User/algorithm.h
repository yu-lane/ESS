#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

//#define LAB7_AHRS
//#include "stm32f10x.h"
#include <math.h>
float Kalman_Filter(float angle_m,float gyro_m);
float Kalman_Filter1(float angle_m,float gyro_m);
//void ADC_Data_Process( unsigned short int Raw_Data[4],float Ris_Data[4]);


#endif



