// bsp_uart.c
#include "bsp_uart.h"
#include <string.h>

static volatile uint8_t uart_tx_busy = 0;
static uint8_t uart_rx_byte;
#define UART_RX_RING_SIZE 64u
static volatile uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint8_t uart_rx_head;
static volatile uint8_t uart_rx_tail;

void bsp_uart_init(void)
{
    // CubeMX 已调用 MX_UART4_Init
    uart_rx_head = uart_rx_tail = 0;
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t next = (uint8_t)((uart_rx_head + 1u) % UART_RX_RING_SIZE);
        if (next != uart_rx_tail) {
            uart_rx_ring[uart_rx_head] = uart_rx_byte;
            uart_rx_head = next;
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

uint8_t bsp_uart_read_byte(uint8_t *value)
{
    if (uart_rx_tail == uart_rx_head) return 0;
    __disable_irq();
    *value = uart_rx_ring[uart_rx_tail];
    uart_rx_tail = (uint8_t)((uart_rx_tail + 1u) % UART_RX_RING_SIZE);
    __enable_irq();
    return 1;
}

void bsp_uart_send(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TCF);
    status = HAL_UART_Transmit_DMA(&huart2, data, len);
    if (status != HAL_OK) {
        /* HAL_UART_AbortTransmit resets TX DMA/gState only; it does not touch
           the active one-byte RX interrupt.  Recover a stale BUSY_TX state
           and retry once instead of silently stopping the diagnostic stream. */
        HAL_UART_AbortTransmit(&huart2);
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TCF);
        status = HAL_UART_Transmit_DMA(&huart2, data, len);
    }
    uart_tx_busy = (status == HAL_OK) ? 1u : 0u;
}

// 等上一帧 DMA 全部发完（TC = 最后一个停止位已出引脚）。
// 组帧缓冲是静态的、正被 DMA 读，必须在"重建帧"之前调用（防止覆盖在发的帧）。
// 调用点放在每帧"等完从机、组帧之前"：串口在线时间与下一轮同步/采集/等待重叠，
// 帧周期不再叠加固定延时，最终由串口在线时间(57ms)决定 -> ~17 FPS。
void bsp_uart_wait_done(uint32_t timeout_ms)
{
    if (!uart_tx_busy) return;

    uint32_t t0 = HAL_GetTick();
    while (!(USART2->ISR & USART_ISR_TC)) {
        if (HAL_GetTick() - t0 > timeout_ms) break;
    }
    /* Ensure HAL gState and the DMA channel are reusable even if the USART TC
       interrupt was delayed/lost.  TX-only abort leaves RX enabled. */
    HAL_UART_AbortTransmit(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TCF);
    uart_tx_busy = 0;
}
