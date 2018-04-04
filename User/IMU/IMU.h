#ifndef __IMU_H
#define __IMU_H

//#ifdef LAB7_IMU
#include "mpu9250.h"  //包含所有的驱动 头文件
//#include "include.h"
#include <math.h>
#define M_PI  (float)3.1415926535

//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(void); //更新姿态
//uint32_t micros(void);	//读取系统上电后的时间  单位 us 

//#endif

#endif
//------------------End of File----------------------------
