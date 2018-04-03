#ifndef _AHRS_H_
#define _AHRS_H_

#include "common.h"  //包含所有的驱动 头文件
#include <math.h>
#include "Matrix.h"

#define M_PI  (float)3.1415926535

extern uint32_t micros(void);
extern void AHRS_init(void);
extern void AHRS_getYawPitchRoll(float *angle);


#endif
