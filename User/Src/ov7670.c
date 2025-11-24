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
  {0x11, 0x03}, // CLKRC: 内部时钟分频
  {0x0C, 0x00}, // COM3: 00为默认值，是否要启用缩放？** 
  {0x3E, 0x00}, // COM14: 00为默认值, Scale与PCLK分频相关？
  {0x04, 0x00}, // COM1: 00为默认值，disable CCIR656
  {0x40, 0xD0}, // COM15: RGB565 full range, D0 **, 可以试一下10，就不是full range
  {0x3A, 0x04}, // TSLB: 默认值0D, 04的话是：Set UV ordering,  do not auto-reset window
  {0x14, 0x38}, // COM9: AGC上限, 默认值是4A

  // 色彩矩阵，从UV转成RGB，会影响颜色
  {0x4F, 0x40}, // MTX1
  {0x50, 0x34}, // MTX2
  {0x51, 0x0C}, // MTX3
  {0x52, 0x17}, // MTX4
  {0x53, 0x29}, // MTX5
  {0x54, 0x40}, // MTX6
  {0x58, 0x1e}, // MTXS: Matrix Coefficient Sign, 默认值1e
  {0x3d, 0xc0}, // COM13: 默认值88， Gamma enable, UV average auto adjust


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

  {0x0E, 0x61}, // COM5 默认值01 // 保留位
  {0x0F, 0x4B}, // COM6 默认值43，感觉没啥影响



  // 以下是一些实验

  {0x16, 0x02}, // RSVD reserved
  {0x1e, 0x07}, // MVFP: 默认值01， 0x3x则表示水平垂直翻转
  {0x21, 0x02}, // 保留
  {0x22, 0x91}, // 保留
  {0x29, 0x07}, // 保留
  //{0x33, 0x0b},
  //{0x35, 0x0b},
  {0x70, 0x4a}, //SCALING_XSC, 默认3a，***
  {0x71, 0x35}, //SCALING_YSC, 默认35，
  {0x37, 0x1d}, //保留
  {0x38, 0x71}, //保留
  {0x39, 0x2a}, //保留
  {0x3c, 0x78}, // COM12 默认68
  {0x4d, 0x40}, // 保留
  {0x4e, 0x20}, // 保留
  {0x69, 0x00}, // GFIX 固定增益控制，默认00
  {0x6b, 0x0a}, // DBLV 默认0a， 4a为2xPLL ***
  {0x74, 0x10}, // REG74 默认00， 10表示手动控制数字增益
  {0x8d, 0x4f}, //保留
  {0x8e, 0x00}, //保留
  {0x8f, 0x00}, //保留
  {0x90, 0x00}, //保留
  {0x91, 0x00}, //保留
  {0x96, 0x00}, //保留
  {0x9a, 0x00}, //保留
  {0xb0, 0x84}, //保留
  {0xb1, 0x0c}, //ABLC, 默认0x00, 自动黑电平校正
  {0xb2, 0x0e}, // 保留
  {0xb3, 0x82}, //自动黑电平校正目标值，默认0x80
  {0xb8, 0x0a}, // 保留
  {0x73, 0xF0}, // SCALING_PCLK_DIV 这个寄存器不设置就啥都看不到，但手册上说[7:4]是reserved。。。
  {0x13, 0x81}, //AEC使能，自动白平衡与AGC关闭

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