#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOB
#define LCD_A0_Pin GPIO_PIN_1
#define LCD_A0_GPIO_Port GPIOB
#define SPI_HANDLE hspi2

#define LCD_MODE_DATA() HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET)
#define LCD_MODE_CMD() HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET)

extern SPI_HandleTypeDef SPI_HANDLE;

void LCD_Hard_Reset(); 
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_Data(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif