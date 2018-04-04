#ifndef _AHRS_H_
#define _AHRS_H_

//#ifdef LAB7_AHRS
//#include "mpu9250.h"  //包含所有的驱动 头文件
#include <math.h>
#include "Matrix.h"
#include "include.h"
#define M_PI  (float)3.1415926535


 void AHRS_init(void);
 void AHRS_getYawPitchRoll(void);
//#endif

#endif
