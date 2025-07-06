#include <stdarg.h>
#include <stdio.h>
#include "log_uart.h"

// 重定向printf到UART
PUTCHAR_PROTOTYPE { 
  HAL_UART_Transmit(&UART_HANDLE, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

uint32_t get_timestamp(void) {
    return HAL_GetTick(); 
}

void log_write(uint8_t level, const char* format, ...) {
  static const char *level_str[] = {
    "[DEBUG] ", "[INFO]  ", "[WARN]  ", "[ERROR] "
  };

  if (level < LOG_LEVEL) {
    return;
  }

  char buf[256];

  va_list args;
  va_start(args, format);
  int len = sprintf(buf, "%8lu %s", get_timestamp(), level_str[level]);
  len += vsnprintf(buf + len, sizeof(buf) - len, format, args);

  HAL_UART_Transmit(&UART_HANDLE, (uint8_t *)buf, len, 0xFFFF);
  va_end(args);

}
