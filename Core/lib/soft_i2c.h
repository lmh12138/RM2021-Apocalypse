#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f4xx.h"

extern GPIO_TypeDef *IIC_GPIO;
extern uint16_t SDA;
extern uint16_t SCL;

int IIC_ReadData(uint8_t dev_addr, uint8_t reg_addr, uint8_t * pdata, uint8_t count);
int IIC_WriteData(uint8_t dev_addr,uint8_t reg_addr,uint8_t data);

#endif
