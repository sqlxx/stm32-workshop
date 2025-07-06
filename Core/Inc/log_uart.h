#ifndef __LOG_UART_H
#define __LOG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#include "stm32f1xx_hal.h"

#define LOG_LEVEL_DEBUG    0
#define LOG_LEVEL_INFO     1
#define LOG_LEVEL_WARNING  2
#define LOG_LEVEL_ERROR    3

// Configuration start
#define UART_HANDLE huart1
#define LOG_LEVEL LOG_LEVEL_DEBUG
// Configration end

extern UART_HandleTypeDef UART_HANDLE;

void log_write(uint8_t level, const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif