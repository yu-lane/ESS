#ifndef _BSP_I2C_H
#define _BSP_I2C_H

#include <inttypes.h>
#include "stm32f10x.h"

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(u8 ack);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
uint8_t i2c_CheckDevice(uint8_t _Address);
void i2c_GPIO_Config(void);
void IIC_ReadData(u8 SLAVE_ADDRESS,u8 reg_add,unsigned char*Read,u8 num);
void IIC_WriteReg(u8 SLAVE_ADDRESS, u8 reg_add,u8 reg_dat);


#endif