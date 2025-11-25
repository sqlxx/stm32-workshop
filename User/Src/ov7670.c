#include "ov7670.h"
#include "log_uart.h"

/* Registers  copy from Linux kernel*/
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58


#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */

#define REG_GFIX	0x69	/* Fix gain control */

#define REG_DBLV	0x6b	/* PLL control an debugging */
#define   DBLV_BYPASS	  0x0a	  /* Bypass PLL */
#define   DBLV_X4	  0x4a	  /* clock x4 */
#define   DBLV_X6	  0x8a	  /* clock x6 */
#define   DBLV_X8	  0xca	  /* clock x8 */

#define REG_SCALING_XSC	0x70	/* Test pattern and horizontal scale factor */
#define   TEST_PATTTERN_0 0x80
#define REG_SCALING_YSC	0x71	/* Test pattern and vertical scale factor */
#define   TEST_PATTTERN_1 0x80

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */

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

  {REG_COM7, COM7_FMT_QVGA|COM7_RGB}, // COM7: QVGA, RGB output, no downsample, 0x02为测试彩条
  {REG_CLKRC, 0x03}, // CLKRC: 内部时钟分频
  {0x0C, 0x00}, // COM3: 00为默认值，是否要启用缩放？** 
  {0x3E, 0x00}, // COM14: 00为默认值, Scale与PCLK分频相关？
  {0x04, 0x00}, // COM1: 00为默认值，disable CCIR656
  {0x40, 0xD0}, // COM15: RGB565 full range, D0 **, 可以试一下10，就不是full range
  {0x3A, 0x04}, // TSLB: 默认值0D, 04的话是：Set UV ordering,  do not auto-reset window
  {0x14, 0x08}, // COM9: AGC上限, 默认值是4A

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
	OV7670_Write_Reg(REG_COM7, COM7_RESET);
  HAL_Delay(200);
  for (int i = 0; ov7670_qvga_rgb565[i][0] != 0xff; i++) {
    OV7670_Write_Reg(ov7670_qvga_rgb565[i][0], ov7670_qvga_rgb565[i][1]);
  }
    
}