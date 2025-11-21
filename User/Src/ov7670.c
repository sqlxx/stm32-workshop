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

const uint8_t ov7670_qvga_rgb565[][2] = {
    {0x12, 0x14}, // QVGA, RGB output, no downsample
    {0x40, 0xd0}, // RGB565, full range
    {0xb0, 0x84}, // Color mode

    //--- Clock settings ---
    {0x11, 0x3f}, // PCLK division: internal prescaler (1=div2),根据需要调

    //--- PCLK settings ---
    {0x6b, 0x00}, // PLL: x4  (可根据需要调低输出速度)

    //--- Gamma ---
    {0x7a, 0x20},
    {0x7b, 0x10},
    {0x7c, 0x1e},
    {0x7d, 0x35},
    {0x7e, 0x5a},
    {0x7f, 0x69},
    {0x80, 0x76},
    {0x81, 0x80},
    {0x82, 0x88},
    {0x83, 0x8f},
    {0x84, 0x96},
    {0x85, 0xa3},
    {0x86, 0xaf},
    {0x87, 0xc4},
    {0x88, 0xd7},
    {0x89, 0xe8},

    //--- AEC/AGC stable ---
    {0xa2, 0x02},
    {0x8f, 0xdf},
    {0x90, 0x00},
    {0x91, 0x00},
    {0x96, 0x00},
    {0x9a, 0x80},
    {0xb1, 0x0c},
    {0xb2, 0x0e},
    {0xb3, 0x80},
    {0xb8, 0x0a},

    //--- Windowing QVGA ---
    {0x17, 0x16}, // HSTART
    {0x18, 0x04}, // HSTOP
    {0x32, 0xa4}, // HREF
    {0x19, 0x02}, // VSTART
    {0x1a, 0x7a}, // VSTOP
    {0x03, 0x0a}, // VREF

    //--- Matrix / Color ---
    {0x4f, 0xb3},
    {0x50, 0xb3},
    {0x51, 0x00},
    {0x52, 0x3d},
    {0x53, 0xa7},
    {0x54, 0xe4},
    {0x58, 0x9e},

    // Enable AE, AWB
    {0x13, 0xe7},

    {0xff, 0xff}, // END
};

void OV7670_Init()
{
	OV7670_Write_Reg(0x12, 0x80);
  HAL_Delay(200);
  for (int i = 0; ov7670_qvga_rgb565[i][0] != 0xff; i++) {
    OV7670_Write_Reg(ov7670_qvga_rgb565[i][0], ov7670_qvga_rgb565[i][1]);
  }
    
}