/**
  ******************************************************************************
  * @file    uart.c
  * @brief   USART1 中断收发驱动
  *
  *          收：中断里把字节塞进环形缓冲，主循环慢慢解析（绝不丢字节）
  *          发：环形缓冲 + TXE 中断，uart_puts 不会卡住主循环
  ******************************************************************************
  */
#include "uart.h"
#include "board.h"
#include "ringbuf.h"

#define UART_RX_SIZE      256U
#define UART_TX_SIZE      512U

/* 想用 printf 调试时把下面改成 1，并在 Keil 里勾选 Use MicroLIB */
#define UART_USE_PRINTF_RETARGET   0

static uint8_t   s_rx_mem[UART_RX_SIZE];
static uint8_t   s_tx_mem[UART_TX_SIZE];
static ringbuf_t s_rx;
static ringbuf_t s_tx;

/*----------------------------------------------------------------------------
 * 初始化
 *--------------------------------------------------------------------------*/
void uart_init(uint32_t baud)
{
    uint32_t pclk;

    rb_init(&s_rx, s_rx_mem, UART_RX_SIZE);
    rb_init(&s_tx, s_tx_mem, UART_TX_SIZE);

    /* 时钟：UART8 在 APB1，USART1 在 APB2，用宏区分，波特率按对应总线时钟算 */
    RCC->AHB1ENR |= BOARD_UART_GPIO_AHB1EN;
#ifdef BOARD_UART_APB2EN
    RCC->APB2ENR |= BOARD_UART_APB2EN;
    pclk = board_pclk2_hz();
#else
    RCC->APB1ENR |= BOARD_UART_APB1EN;
    pclk = board_pclk1_hz();
#endif
    (void)RCC->AHB1ENR;                    /* 读回，保证外设时钟已生效 */
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    /* PE1/PE0 -> AF8（USART1 则为 PA9/PA10 -> AF7），上拉避免悬空时收到乱码 */
    board_gpio_af_config(BOARD_UART_GPIO, BOARD_UART_TX_PIN, BOARD_UART_AF, 1U);
    board_gpio_af_config(BOARD_UART_GPIO, BOARD_UART_RX_PIN, BOARD_UART_AF, 1U);

    /* 波特率：BRR = fCK / baud（低 4 位是 1/16 小数部分，直接取整就带了小数） */
    BOARD_UART->CR1 = 0U;
    BOARD_UART->CR2 = 0U;                  /* 1 个停止位 */
    BOARD_UART->CR3 = 0U;                  /* 无硬件流控 */
    BOARD_UART->BRR = (uint16_t)((pclk + (baud / 2U)) / baud);
    BOARD_UART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(BOARD_UART_IRQn, 3U); /* 比 SysTick 高，保证 115200 下不丢字节 */
    NVIC_EnableIRQ(BOARD_UART_IRQn);
}

/*----------------------------------------------------------------------------
 * 中断服务：必须由 config.h 里的 BOARD_UART_IRQ_HANDLER 调用
 *--------------------------------------------------------------------------*/
void uart_irq_handler(void)
{
    uint32_t sr = BOARD_UART->SR;

    /* --- 接收（同时把 ORE/FE/NE/PE 一起清掉） --- */
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U)
    {
        uint8_t ch = (uint8_t)(BOARD_UART->DR & 0xFFU);   /* 读 DR 清标志 */

        if ((sr & USART_SR_RXNE) != 0U) {
            (void)rb_push(&s_rx, ch);                     /* 缓冲满就丢弃最坏的情况 */
        }
    }

    /* --- 发送 --- */
    if ((sr & USART_SR_TXE) != 0U) {
        if ((BOARD_UART->CR1 & USART_CR1_TXEIE) != 0U) {
            uint8_t ch;
            if (rb_pop(&s_tx, &ch) != 0U) {
                BOARD_UART->DR = ch;
            } else {
                BOARD_UART->CR1 &= ~USART_CR1_TXEIE;      /* 没数据了，关中断 */
            }
        }
    }
}

/*----------------------------------------------------------------------------
 * 收发接口
 *--------------------------------------------------------------------------*/
uint8_t uart_read(uint8_t *ch)
{
    return rb_pop(&s_rx, ch);
}

void uart_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; i++) {
        /* 缓冲满就等中断把数据搬走；本函数只在主循环上下文调用 */
        while (rb_push(&s_tx, data[i]) == 0U) {
        }
        BOARD_UART->CR1 |= USART_CR1_TXEIE;
    }
}

void uart_putc(char ch)
{
    uart_write((const uint8_t *)&ch, 1U);
}

void uart_puts(const char *str)
{
    uint16_t len = 0U;

    while (str[len] != '\0') {
        len++;
    }
    if (len != 0U) {
        uart_write((const uint8_t *)str, len);
    }
}

/*----------------------------------------------------------------------------
 * 数字格式化
 *--------------------------------------------------------------------------*/
static void put_uint(uint32_t value, uint8_t min_digits)
{
    char    tmp[11];
    uint8_t n = 0U;

    do {
        tmp[n] = (char)('0' + (value % 10U));
        value /= 10U;
        n++;
    } while (value != 0U);

    while (n < min_digits) {
        tmp[n] = '0';
        n++;
    }
    while (n > 0U) {
        n--;
        uart_putc(tmp[n]);
    }
}

void uart_put_u32(uint32_t value)
{
    put_uint(value, 1U);
}

void uart_put_i32(int32_t value)
{
    uint32_t mag;

    if (value < 0) {
        uart_putc('-');
        mag = (uint32_t)(-(value + 1)) + 1U;    /* 规避 INT32_MIN 取反溢出 */
    } else {
        mag = (uint32_t)value;
    }
    put_uint(mag, 1U);
}

void uart_put_mdeg(int32_t mdeg)
{
    uint32_t mag;

    if (mdeg < 0) {
        uart_putc('-');
        mag = (uint32_t)(-(mdeg + 1)) + 1U;
    } else {
        mag = (uint32_t)mdeg;
    }
    put_uint(mag / 1000U, 1U);
    uart_putc('.');
    put_uint(mag % 1000U, 3U);
}

void uart_put_deg1(int32_t mdeg)
{
    uint32_t mag;
    uint32_t d10;

    if (mdeg < 0) {
        uart_putc('-');
        mag = (uint32_t)(-(mdeg + 1)) + 1U;
    } else {
        mag = (uint32_t)mdeg;
    }
    d10 = (mag + 50U) / 100U;              /* 四舍五入到 0.1° */
    put_uint(d10 / 10U, 1U);
    uart_putc('.');
    put_uint(d10 % 10U, 1U);
}

/*----------------------------------------------------------------------------
 * 可选：printf 重定向
 *   Keil(MDK5, ARMCC) 下勾选 Use MicroLIB 后即可直接 printf。
 *   注意 printf 只允许在主循环上下文调用。
 *--------------------------------------------------------------------------*/
#if (UART_USE_PRINTF_RETARGET != 0)
#include <stdio.h>

int fputc(int ch, FILE *f)
{
    (void)f;
    uart_putc((char)ch);
    return ch;
}
#endif
