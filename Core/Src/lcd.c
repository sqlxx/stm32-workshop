#include "lcd.h"

void LCD_Hard_Reset() {
  
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(120);

}

void LCD_Send_Cmd(uint8_t cmd) {
  LCD_CHIP_SELECT();
  LCD_MODE_CMD();
  HAL_SPI_Transmit(&SPI_HANDLE, &cmd, 1, 1000);
  LCD_CHIP_UNSELECT();
}

void LCD_Send_Data(uint8_t *data, size_t len) {
  LCD_CHIP_SELECT();
  LCD_MODE_DATA();
  HAL_SPI_Transmit(&SPI_HANDLE, data, len, 1000);
  LCD_CHIP_UNSELECT();
}

void LCD_Clear(uint16_t color) {
  LCD_Set_Window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

  LCD_Send_Cmd(0x2c);

  uint8_t color_data[2];
  color_data[0] = color >> 8;  // 高字节
  color_data[1] = color & 0xFF; // 低字节

  for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
    LCD_Send_Data(color_data, 2);
  }
}

void LCD_Sleep() {
  LCD_Send_Cmd(0x10);
  HAL_Delay(10);
}

void LCD_SleepOut() {
  LCD_Send_Cmd(0x11);
  HAL_Delay(10);
}

void LCD_Draw_Pixel(uint16_t x, uint16_t y, uint16_t color) {
  LCD_Set_Window(x, y, x, y);
  LCD_Send_Cmd(0x2c);
  uint8_t color_data[] = {color >> 8, color & 0xFF};
  LCD_Send_Data(color_data, 2);
}

void LCD_Draw_Square_Dot(uint16_t x, uint16_t y, uint16_t color, uint8_t width) {
  LCD_Set_Window(x - width/2, y - width/2, x + width/2, y + width/2);
  LCD_Send_Cmd(0x2c);
  for (int i = 0; i < width * width; i++) {
    uint8_t color_data[] = {color >> 8, color & 0xFF};
    LCD_Send_Data(color_data, 2);
  }
}

//右上角坐标x,y的位置显示一个16行8列的字符，需要传入一个16个8bit int的数组，每一个int代表一行，扫描从上到下，从左到右
void LCD_Char(uint16_t x, uint16_t y, uint8_t* array, uint16_t color) {
  LCD_Set_Window(x, y, x + 7, y + 15);
  LCD_Send_Cmd(0x2c);
  uint8_t color_data[] = {color >> 8, color & 0xFF};
  uint8_t default_background_data[] = {DEFAULT_BACKGROUND_COLOR >> 8, DEFAULT_BACKGROUND_COLOR & 0xFF};
  for (int i = 0; i < 16; i++) {
    for (int j = 7; j >=0; j--) {
      if (array[i] & (1 << j)) {
        LCD_Send_Data(color_data, 2);
      } else {
        LCD_Send_Data(default_background_data, 2);
      }
    }
  }

}

void LCD_Full_Char(uint16_t x, uint16_t y, uint8_t* ch, uint16_t color) {
  LCD_Set_Window(x, y, x + 15, y + 15);
  LCD_Send_Cmd(0x2c);
  uint8_t color_data[] = {color >> 8, color & 0xFF};
  uint8_t default_background_data[] = {DEFAULT_BACKGROUND_COLOR >> 8, DEFAULT_BACKGROUND_COLOR & 0xFF};
  for (int i = 0; i < 32; i++) {
    for (int j =0 ; j < 8; j++) { //字模低位在前
      if (ch[i] & (1 << j)) {
        LCD_Send_Data(color_data, 2);
      } else {
        LCD_Send_Data(default_background_data, 2);
      }
    }
  }
}

void LCD_Draw_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint8_t width) {
  uint16_t start_x, start_y, end_x, end_y;
  if (x0 < x1) {
    start_x = x0;
    end_x = x1;
  } else {
    start_x = x1;
    end_x = x0;
  }
  
  if (y0 < y1) {
    start_y = y0;
    end_y = y1;
  } else {
    start_y = y1;
    end_y = y0;
  }

  for (uint16_t i = start_x; i <= end_x; i++) {
    for (uint16_t j = start_y; j <= end_y; j++) {
      LCD_Draw_Square_Dot(i, j, color, width);
    }
  }

}
void LCD_Set_Oritention(uint8_t ori) {
  LCD_Send_Cmd(0x36);
  LCD_Send_Data(&ori, 1);
}

void LCD_Set_Window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  LCD_Send_Cmd(0x2a);
  LCD_Send_Data((uint8_t[]){x0 >> 8, x0 & 0xff, x1 >> 8, x1 & 0xff}, 4);
  LCD_Send_Cmd(0x2b);
  LCD_Send_Data((uint8_t[]){y0 >> 8, y0 & 0xff, y1 >> 8, y1 & 0xff}, 4);
}

void LCD_Init() {

  LCD_Send_Cmd(0x3A);
  LCD_Send_Data((uint8_t[]){0x55}, 1); // 设置成RGB565格式，默认为RGB666

  LCD_Send_Cmd(0xB1); // 设置帧率
  LCD_Send_Data((uint8_t[]){0x00, 0x15}, 2); // 默认为0x1B 70Hz, 测试发现要设置到0x15 90Hz才没有明显的闪烁

  LCD_Set_Oritention(LCD_ORIENTATION);

  LCD_Clear(DEFAULT_BACKGROUND_COLOR);
  // 退出睡眠模式
  LCD_SleepOut();

  // 开启显示
  LCD_Send_Cmd(0x29);

}