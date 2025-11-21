#include "ov7670.h"
#include "log_uart.h"

uint8_t OV7670_Read_Reg(uint8_t reg) {
  uint8_t value = 0;
  // HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&OV7670_I2C_HANDLE, 0x42<<1,  reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&OV7670_I2C_HANDLE, 0x42, &reg, 1, 1000);
  if (status == HAL_OK) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&OV7670_I2C_HANDLE, 0x42, &value, 1, 1000);
    // Read successful
    if (status == HAL_OK) {
      return value;
    } else {
      // Handle error (optional)
      return 0xFF; // Indicate error
    }
  } else {
    // Handle error (optional)
    return 0xFE; // Indicate error
  }
}

uint8_t OV7670_Write_Reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&OV7670_I2C_HANDLE, 0x42, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}


void OV7670_Init()
{
  // 复位序列
  uint8_t ret = OV7670_Write_Reg(0x12, 0x80);  // COM7 - 复位所有寄存器
  log_write(LOG_LEVEL_DEBUG, "OV7670 Reset Reg Write Result: %d", ret);

  HAL_Delay(100);                // 等待复位完成
  OV7670_Write_Reg(0x11, 0x83); // CLKRC - 设置时钟分频 为4
  OV7670_Write_Reg(0x3B, 0x02); // COM11 - 设置夜间模式关闭


  // 图像格式和输出控制
  OV7670_Write_Reg(0x13, 0xf2);  // COM8: AGC、AEC使能，带宽设置
  OV7670_Write_Reg(0x00, 0x00);  // GAIN: 模拟增益控制
  OV7670_Write_Reg(0x10, 0x00);  // AECH: 曝光时间高位
  OV7670_Write_Reg(0x01, 0x80);  // BLUE: 蓝色通道增益
  OV7670_Write_Reg(0x02, 0x80);  // RED: 红色通道增益
  OV7670_Write_Reg(0x13, 0xf7);  // COM8: 更新设置，保持AGC/AEC使能

  // 图像尺寸和窗口设置
  OV7670_Write_Reg(0x12, 0x14);  // COM7：YUV输出格式 QVGA, RGB格式
  OV7670_Write_Reg(0x0c, 0x04);  // COM3: 缩放使能
  OV7670_Write_Reg(0x18, 0x4A);  // HSTOP:水平窗口结束位置高8位
  OV7670_Write_Reg(0x17, 0x22);  // HSTART: 水平窗口起始位置高8位
  OV7670_Write_Reg(0x32, 0x89);  // HREF: 水平参考控制
  OV7670_Write_Reg(0x19, 0x02);  // VSTART: 垂直窗口起始位置高位
  OV7670_Write_Reg(0x1a, 0x7a);  // VSTOP: 垂直窗口结束位置
  OV7670_Write_Reg(0x03, 0x00);  // VREF: 垂直参考控制低位

  OV7670_Write_Reg(0x1b, 0x01);  // PSHFT: 数据格式-像素延迟选择（D[7:0]相对于HREF延迟多少像素时钟周期
  OV7670_Write_Reg(0x1e, 0x01);  // MVFP: 水平镜像/竖直翻转使能, 0x01为默认值

  OV7670_Write_Reg(0x40, 0xd1);  // COM15 RGB565, 输出范围全幅度
  OV7670_Write_Reg(0x69, 0x80);  // GFIX: 固定增益控制 Gr通道1.5x
  OV7670_Write_Reg(0x6b, 0x0a);  // DBLV: PLL控制, 默认值

  OV7670_Write_Reg(0x14, 0x2e);  // COM9: 自动增益上限8倍，不固定AGC/AEC
    
}