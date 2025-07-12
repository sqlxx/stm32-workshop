#include "touch_xpt2046.h"

#define SAMPLE_TIMES 10

uint16_t Touch_Read_Data(uint8_t cmd) {
  uint8_t txData[3] = {cmd, 0, 0};
  uint8_t rxData[3] = {0};
  __disable_irq(); 
  HAL_GPIO_WritePin(TOUCH_CS_PORT, TOUCH_CS_PIN, GPIO_PIN_RESET);

  // HAL_SPI_Transmit(&SPI_HANDLE, txData, 1, 2000);
  // HAL_SPI_Receive(&SPI_HANDLE, rxData, 2, 2000);
  HAL_SPI_TransmitReceive(&SPI_HANDLE, txData, rxData, 3, 1000);

  HAL_GPIO_WritePin(TOUCH_CS_PORT, TOUCH_CS_PIN, GPIO_PIN_SET);
  __enable_irq();

  uint16_t data = (rxData[1] << 8) | rxData[2]; //第一个字节是发送字节，可以丢弃，第二第三个字节需要合并
  return data >> 3; // 最后3位无效值，丢弃

}

// 总是长边为x, 短边为y，需要根据实际情况转换为屏幕坐标
void Touch_Get_Pos(uint16_t* x, uint16_t* y) {
  uint16_t x_sum = 0, y_sum = 0;
  for (int i = 0; i < SAMPLE_TIMES; i++) {
    x_sum += Touch_Read_Data(0x90);
    y_sum += Touch_Read_Data(0xD0);
  }
  *x = x_sum/SAMPLE_TIMES;
  *y = y_sum/SAMPLE_TIMES;

}
