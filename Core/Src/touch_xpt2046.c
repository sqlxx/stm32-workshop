#include "touch_xpt2046.h"

uint16_t Touch_Read_Data(uint8_t cmd) {
  uint8_t txData[3] = {cmd, 0, 0};
  uint8_t rxData[3] = {0};

  HAL_GPIO_WritePin(TOUCH_CS_PORT, TOUCH_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&SPI_HANDLE, txData, rxData, 3, 1000);
  HAL_GPIO_WritePin(TOUCH_CS_PORT, TOUCH_CS_PIN, GPIO_PIN_SET);

  uint16_t data = (rxData[1] << 8) | rxData[2]; //第一个字节是发送字节，可以丢弃，第二第三个字节需要合并
  return data >> 3; // 最后3位无效值，丢弃

}
