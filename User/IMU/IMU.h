#ifndef __IMU_H
#define __IMU_H

//#ifdef LAB7_IMU
#include "mpu9250.h"  //�������е����� ͷ�ļ�
//#include "include.h"
#include <math.h>
#define M_PI  (float)3.1415926535

//Mini IMU AHRS �����API
void IMU_init(void); //��ʼ��
void IMU_getYawPitchRoll(void); //������̬
//uint32_t micros(void);	//��ȡϵͳ�ϵ���ʱ��  ��λ us 

//#endif

#endif
//------------------End of File----------------------------
