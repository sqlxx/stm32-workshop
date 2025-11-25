#ifndef __OV7670_H
#define __OV7670_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define OV7670_I2C_HANDLE hi2c2

extern I2C_HandleTypeDef OV7670_I2C_HANDLE;


uint8_t OV7670_Write_Reg(uint8_t reg, uint8_t value);
uint8_t OV7670_Read_Reg(uint8_t reg);

void OV7670_Init(void);

#ifdef __cplusplus
}
#endif
#endif