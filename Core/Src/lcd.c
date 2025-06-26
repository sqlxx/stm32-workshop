#include "lcd.h"

void LCD_Hard_Reset() {
  
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(120);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(120);

}

void LCD_Send_Cmd(uint8_t cmd) {
  LCD_MODE_CMD();
  HAL_SPI_Transmit(&SPI_HANDLE, &cmd, 1, 1000);
}

void LCD_Send_Data(uint8_t *data, size_t len) {
  LCD_MODE_DATA();
  HAL_SPI_Transmit(&SPI_HANDLE, data, len, 1000);
}

void LCD_Init() {

  LCD_Send_Cmd(0x3A);
  LCD_Send_Data((uint8_t[]){0x55}, 1); // 设置成RGB565格式，默认为RGB666

  LCD_Send_Cmd(0xB1); // 设置帧率
  LCD_Send_Data((uint8_t[]){0x00, 0x15}, 2); // 默认为0x1B 70Hz, 测试发现要设置到0x15 90Hz才没有明显的闪烁

  // 退出睡眠模式
  LCD_Send_Cmd(0x11);
  HAL_Delay(10); // 手册要求至少等待5ms才能发送下一个命令，如果接下去是Sleep Mode(0x10)命令，则需要等待120ms，为保险这里直接等待10ms

  // 开启显示
  LCD_Send_Cmd(0x29);

}