/**
  ******************************************************************************
  * @file    adc64.h
  * @brief   ADC64 采集板驱动（ADS7953 16 通道 + 4:1 行选），两条边各一块板
  *
  *  探测器板结构（从 pin64-307-2835 PcbDoc 逆向）：
  *      64 只光电二极管 D10~D4F 排成 4 行 x 16 列
  *      D<行><列>，行 = 1..4（EN1..EN4），列 = 0..F（CH0..CHF）
  *      每只二极管经一个 SOD523 S<行><列> 接到列线 CH<列>
  *
  *  两块板共用 EN1~EN4（同一行同时读两块），各自独立的 SPI + 片选：
  *      板 0 = 长边 = SPI4（PE2/PE5/PE6，CS=PE3）
  *      板 1 = 短边 = SPI5（PC10/PC11/PC12，CS=PE4）
  ******************************************************************************
  */
#ifndef __ADC64_H
#define __ADC64_H

#include "config.h"
#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC64_ROWS   4U
#define ADC64_COLS   16U

typedef struct { GPIO_TypeDef *port; uint8_t pin; } cs_t;

typedef struct
{

    /* [板][行][列]，12bit 原始码值 */
    uint16_t cell[ADC64_BOARD_COUNT][ADC64_ROWS][ADC64_COLS];
    /* 每块板按 include 掩码求和 */
    uint32_t sum[ADC64_BOARD_COUNT];
} adc64_frame_t;

void     adc64_init(void);
void     adc64_scan(adc64_frame_t *f);          /* 扫一遍 4 行 x 16 列 x 2 块 */
void     adc64_scan_dark(adc64_frame_t *f);     /* EN 全关，测本底 */
void     adc64_set_en_mask(uint8_t mask);       /* bit0~3 = EN1~EN4，调试用 */

/* 参与计算的掩码：bit = 行*16 + 列，1 = 计入（可屏蔽剪断/损坏的点） */
void     adc64_set_mask(uint8_t board, uint64_t mask);
uint64_t adc64_mask(uint8_t board);
void     adc64_default_masks(void);
void     adc64_set_acc_us(uint16_t us);
void     adc64_set_shift(uint8_t s);
void     adc64_set_en_hold(uint8_t mask);
void     adc64_read_column(uint8_t ch, uint16_t *out);
uint32_t adc64_long_max(void);
uint32_t adc64_long_max_fast(void);
uint32_t adc64_long_conc(void);
uint32_t adc64_long_width(void);
void     adc64_measure(uint16_t *x01, uint16_t *y01, uint16_t *maxl,
                       uint16_t *maxs, uint8_t *valid);
void     adc64_measure_reset(void);
uint16_t adc64_read_edge(uint8_t board, uint8_t e, uint16_t *out);
void     adc64_diag_delay(uint32_t us);
uint16_t adc64_diag_spi(void);
void     adc64_diag_bank(uint16_t *dst);
void     adc64_read_raw_board(uint8_t which, uint16_t *out64);
cs_t     k_cs_direct_get(uint8_t b);
void     k_cs_direct_set(uint8_t b, GPIO_TypeDef *port, uint8_t pin);
void     k_cs_direct_restore(uint8_t b, cs_t v);
uint8_t  adc64_set_cs_pin(uint8_t slot, char port, uint8_t pin);
uint32_t adc64_probe_cs(GPIO_TypeDef *port, uint8_t pin, uint8_t edge);
void     adc64_read_edges_all(uint16_t *b0, uint16_t *b1, uint16_t *b2, uint16_t *b3,
                              uint8_t *n_out);
uint8_t  adc64_get_en_hold(void);
uint8_t  adc64_get_shift(void);
uint16_t adc64_get_acc_us(void);
void     adc64_echo_test(uint8_t board, uint16_t *out);   /* 打印完整 16 位帧，看通道回显 */

#ifdef __cplusplus
}
#endif

#endif /* __ADC64_H */
