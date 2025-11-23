#include "ov7670.h"
#include "log_uart.h"

uint8_t OV7670_Read_Reg(uint8_t reg) {
  uint8_t value = 0;
  //  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&OV7670_I2C_HANDLE, 0x42<<1,  reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&OV7670_I2C_HANDLE, 0x42, &reg, 1, 1000);
  if (status == HAL_OK) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&OV7670_I2C_HANDLE, 0x42, &value, 1, 1000);
    // Read successful
    if (status == HAL_OK) {
      return value;
    } else {
      return 0xFF; // Indicate error
    }
  } else {
    return 0xFE; // Indicate error
  }
}

uint8_t OV7670_Write_Reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&OV7670_I2C_HANDLE, 0x42, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

const uint8_t ov7670_qvga_rgb565[][2] = {
  //  {0x12, 0x14|0x02}, // COM7: QVGA, RGB output, no downsample, 0x02为测试彩条，需要与0x70与0x71的第7位配合工作

  {0x12, 0x14}, // COM7: QVGA, RGB output, no downsample, 0x02为测试彩条
  {0x40, 0xD0}, // COM15: RGB565 full range, D0
  {0x11, 0x03}, // CLKRC: 内部时钟分频
  // {0x6B, 0x0A}, // DBLV: 默认关闭
  {0x8C, 0x00}, // RGB444 disable
  // {0x04, 0x00}, 

  {0x70, 0x3A}, //SCALING_XSC 第7位代表base颜色条
  {0x71, 0x35}, //SCALING_YSC 默认值
  // {0x42, 0x08}, // 显示DSP色彩条

  // {0x3e, 0x19}, //COM14 
  // {0x72, 0x11},
  // {0x73, 0xf1},

  // 亮度、对比度
  // {0x55, 0x10},
  // {0x56, 0x40},
  // {0x09, 0x03}, //输出驱动能力

 

  //输出窗口设置
  {0x32, 0x80}, // HREF
  {0x17, 0x14}, // HSTART
  {0x18, 0x02}, // HSTOP
  {0x19, 0x02}, // VSTART
  {0x1A, 0x7a}, // VSTOP
  {0x03, 0x0a}, // VREF

  // 以下是一些实验

  {0x73, 0xF0}, // SCALING_PCLK_DIV 这个寄存器不设置就啥都看不到，但手册上说[7:4]是reserved。。。
  {0x13, 0x81}, //AEC使能，自动白平衡与AGC关闭
  {0x14, 0x0A}, //自动增益最大值

  {0xFF, 0xFF}, // END

};

void OV7670_Init()
{
	OV7670_Write_Reg(0x12, 0x80);
  HAL_Delay(200);
  for (int i = 0; ov7670_qvga_rgb565[i][0] != 0xff; i++) {
    OV7670_Write_Reg(ov7670_qvga_rgb565[i][0], ov7670_qvga_rgb565[i][1]);
  }
    
}