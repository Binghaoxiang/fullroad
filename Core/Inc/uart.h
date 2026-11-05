/**
  ******************************************************************************
  * @file    uart.h
  * @brief   串口驱动（中断收发 + 无浮点格式化输出）
  ******************************************************************************
  */
#ifndef __UART_H
#define __UART_H

#include "config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     uart_init(uint32_t baud);
void     uart_irq_handler(void);          /* 在 USARTx_IRQHandler 里调用 */

uint8_t  uart_read(uint8_t *ch);          /* 1 = 读到 1 字节 */
void     uart_write(const uint8_t *data, uint16_t len);
void     uart_putc(char ch);
void     uart_puts(const char *str);

/* 数字 -> 字符串（不用 printf，避免半主机/浮点库拖累） */
void     uart_put_u32(uint32_t value);
void     uart_put_i32(int32_t value);
void     uart_put_mdeg(int32_t mdeg);     /* -12.345 */
void     uart_put_deg1(int32_t mdeg);     /* 12.3，用于角速度显示 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
