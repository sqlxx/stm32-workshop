#ifndef __LCD_ILI9341_H
#define __LCD_ILI9341_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOB
#define LCD_A0_Pin GPIO_PIN_1
#define LCD_A0_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOB
#define SPI_HANDLE hspi2


// 常用颜色定义 (RGB565格式)
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define DEFAULT_BACKGROUND_COLOR BLUE


#define LCD_ORIENTATION_LANDSCAPE 0x28
#define LCD_ORIENTATION_PORTRAIT 0x48
#define LCD_BASE_WIDTH 240 
#define LCD_BASE_HEIGHT 320

#define LCD_ORIENTATION LCD_ORIENTATION_PORTRAIT

//默认是竖屏，所以宽度240，高度320，当横屏时，需要交换设置
#define LCD_WIDTH (LCD_ORIENTATION == LCD_ORIENTATION_PORTRAIT ? LCD_BASE_WIDTH : LCD_BASE_HEIGHT)
#define LCD_HEIGHT (LCD_ORIENTATION == LCD_ORIENTATION_PORTRAIT ? LCD_BASE_HEIGHT : LCD_BASE_WIDTH)


#define LCD_MODE_DATA() HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET)
#define LCD_MODE_CMD() HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET)
#define LCD_CHIP_SELECT() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CHIP_UNSELECT() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)

extern SPI_HandleTypeDef SPI_HANDLE;

static const uint8_t LCD_Chars[3][16] = {
  { 0x0, 0xc0, 0x20, 0x20, 0x20, 0xc0, 0x0, 0x0, 0x0, 0xf, 0x1, 0x1, 0x1, 0xf, 0x0, 0x0 }, /* (0) A */
{ 0x0, 0xe0, 0x20, 0x20, 0x20, 0xc0, 0x0, 0x0, 0x0, 0xf, 0x9, 0x9, 0x9, 0x6, 0x0, 0x0 }, /* (1) B */
{ 0x0, 0xc0, 0x20, 0x20, 0x20, 0x40, 0x0, 0x0, 0x0, 0x7, 0x8, 0x8, 0x8, 0x4, 0x0, 0x0 } /* (2) C */
};

void LCD_Hard_Reset(); 
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_Data(uint8_t *data, size_t size);
void LCD_Init();
void LCD_Clear(uint16_t color);
void LCD_Set_Oritention(uint8_t ori);
void LCD_Set_Window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

void LCD_Draw_Pixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_Draw_Square_Dot(uint16_t x, uint16_t y, uint16_t color, uint8_t width);
void LCD_Draw_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint8_t width);

void LCD_Char(uint16_t x, uint16_t y, uint8_t* ch, uint16_t color);
void LCD_Full_Char(uint16_t x, uint16_t y, uint8_t* ch, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif