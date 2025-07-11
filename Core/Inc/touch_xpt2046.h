#ifndef __TOUCH_XPT2046_H
#define __TOUCH_XPT2046_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define TOUCH_CS_PIN GPIO_PIN_5
#define TOUCH_CS_PORT GPIOC

#define SPI_HANDLE hspi1

extern SPI_HandleTypeDef SPI_HANDLE;

uint16_t Touch_Read_Data(uint8_t cmd);

void Touch_Get_Pos(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif
#endif