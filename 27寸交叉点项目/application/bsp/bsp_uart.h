// bsp_uart.h
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include "usart.h"
#include <stdio.h>

void bsp_uart_init(void);
void bsp_uart_send(uint8_t *data, uint16_t len);
void bsp_uart_wait_done(uint32_t timeout_ms);
uint8_t bsp_uart_read_byte(uint8_t *value);

extern UART_HandleTypeDef huart2;

#endif
