#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_usart1.h"
#include "key_addr.h"
#include "zlg_zigbee.h"
#include "mpu9250.h"
#include "bsp_TiMbase.h"
#include "can.h"
#include "bsp_adc.h"
#include "algorithm.h"
#include "IMU.h"
struct Axis_f{
	float x;
	float y;
	float z;
};

struct Axis_Euler{
	float Pitch;//绕机翼的旋转-俯仰
	float Roll;//绕机身的旋转-横滚
	float Yaw;//航向
};














#endif
