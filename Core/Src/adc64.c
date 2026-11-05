/**
  ******************************************************************************
  * @file    adc64.c
  * @brief   ADS7953 双板采集实现
  ******************************************************************************
  */
#include "adc64.h"
#include "board.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint8_t       pin;
} en_t;

typedef struct
{
    GPIO_TypeDef *port;
    uint8_t       pin;
} cs_t;

static const en_t k_en_a[ADC64_ROWS] = {
    { ADC64_EN_A_PORT_1, ADC64_EN_A_PIN_1 },
    { ADC64_EN_A_PORT_2, ADC64_EN_A_PIN_2 },
    { ADC64_EN_A_PORT_3, ADC64_EN_A_PIN_3 },
    { ADC64_EN_A_PORT_4, ADC64_EN_A_PIN_4 }
};

static const en_t k_en_b[ADC64_ROWS] = {
    { ADC64_EN_B_PORT_1, ADC64_EN_B_PIN_1 },
    { ADC64_EN_B_PORT_2, ADC64_EN_B_PIN_2 },
    { ADC64_EN_B_PORT_3, ADC64_EN_B_PIN_3 },
    { ADC64_EN_B_PORT_4, ADC64_EN_B_PIN_4 }
};

static const cs_t k_cs[ADC64_BOARD_COUNT] = {
    { ADC64_A_CS_PORT, ADC64_A_CS_PIN },
    { ADC64_B_CS_PORT, ADC64_B_CS_PIN }
};

static uint64_t s_mask[ADC64_BOARD_COUNT];
static uint8_t  s_acc_us    = 20U;
static uint8_t  s_cmd_shift = 12U;
static uint8_t  s_en_hold   = 0U;

void adc64_set_acc_us(uint16_t us) { s_acc_us = (us > 200U) ? 200U : (uint8_t)us; }
uint16_t adc64_get_acc_us(void)    { return (uint16_t)s_acc_us; }
void adc64_set_shift(uint8_t s)    { s_cmd_shift = (s > 15U) ? 15U : s; }
uint8_t adc64_get_shift(void)      { return s_cmd_shift; }
void adc64_set_en_hold(uint8_t m)  { s_en_hold = m; }
uint8_t adc64_get_en_hold(void)    { return s_en_hold; }

/*----------------------------------------------------------------------------
 * 片选 / 行选
 *--------------------------------------------------------------------------*/
static void cs_low(uint8_t b)  { k_cs[b].port->BSRR = (uint32_t)(1UL << (k_cs[b].pin + 16U)); }
static void cs_high(uint8_t b) { k_cs[b].port->BSRR = (uint32_t)(1UL << k_cs[b].pin); }

static void en_low(const en_t *e)  { e->port->BSRR = (uint32_t)(1UL << (e->pin + 16U)); }
static void en_high(const en_t *e) { e->port->BSRR = (uint32_t)(1UL << e->pin); }

static void en_off_all(void)
{
    uint8_t r;

    for (r = 0U; r < ADC64_ROWS; r++) {
        en_low(&k_en_a[r]);
        en_low(&k_en_b[r]);
    }
}

void adc64_set_en_mask(uint8_t mask)
{
    uint8_t r;

    for (r = 0U; r < ADC64_ROWS; r++) {
        if (((mask >> r) & 1U) != 0U) {
            en_high(&k_en_a[r]);
            en_high(&k_en_b[r]);
        } else {
            en_low(&k_en_a[r]);
            en_low(&k_en_b[r]);
        }
    }
}

/* 逐引脚设置，不做"全关再开"（避免探测器偏置跌落） */
static void en_on(uint8_t row)
{
    uint8_t r;

    for (r = 0U; r < ADC64_ROWS; r++) {
        if (r == row) {
            en_high(&k_en_a[r]);
            en_high(&k_en_b[r]);
        } else {
            en_low(&k_en_a[r]);
            en_low(&k_en_b[r]);
        }
    }
}

static void en_gpio_cfg(const en_t *e)
{
    uint32_t pos = (uint32_t)e->pin * 2U;

    e->port->MODER   = (e->port->MODER & ~(3UL << pos)) | (1UL << pos);
    e->port->OTYPER &= ~(1UL << e->pin);
    e->port->OSPEEDR = (e->port->OSPEEDR & ~(3UL << pos)) | (2UL << pos);
    e->port->PUPDR   = (e->port->PUPDR & ~(3UL << pos));
    e->port->BSRR    = (uint32_t)(1UL << (e->pin + 16U));
}

/*----------------------------------------------------------------------------
 * 初始化
 *--------------------------------------------------------------------------*/
void adc64_init(void)
{
    uint8_t i;

    board_spi_init();

    /* CS：PI8(板A) / PH4(板B) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN;
    (void)RCC->AHB1ENR;
    GPIOI->MODER = (GPIOI->MODER & ~(3UL << 16)) | (1UL << 16);
    GPIOH->MODER = (GPIOH->MODER & ~(3UL << 8))  | (1UL << 8);
    GPIOI->BSRR  = (1UL << 8);
    GPIOH->BSRR  = (1UL << 4);

    /* EN 行选时钟 */
    RCC->AHB1ENR |= ADC64_EN_GPIO_AHB1EN | RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;
    (void)RCC->AHB1ENR;

    for (i = 0U; i < ADC64_ROWS; i++) {
        en_gpio_cfg(&k_en_a[i]);
        en_gpio_cfg(&k_en_b[i]);
    }

    /* 强制重写 GPIOH 的 PH10/PH11 为推挽输出 */
    {
        uint32_t moder = GPIOH->MODER;
        moder &= ~((3UL << 20) | (3UL << 22));
        moder |=  ((1UL << 20) | (1UL << 22));
        GPIOH->MODER = moder;
        GPIOH->OTYPER &= ~((1UL << 10) | (1UL << 11));
        GPIOH->OSPEEDR |= ((2UL << 20) | (2UL << 22));
        GPIOH->PUPDR  &= ~((3UL << 20) | (3UL << 22));
    }

    en_off_all();
    adc64_default_masks();
}

/*----------------------------------------------------------------------------
 * 一帧：CS 低 -> t_ACC -> 16 拍 -> 等最后一位 -> CS 高
 *--------------------------------------------------------------------------*/
static uint16_t read_frame(uint8_t board, uint8_t col)
{
    /* 单通道读：先发选通道命令，再发一帧同命令拿回结果（2 帧流水线） */
    uint16_t cmd = (uint16_t)(0x1000U | ((uint16_t)(col & 0x0FU) << 7));
    uint16_t rx;

    cs_low(board);
    board_delay_us(s_acc_us);
    (void)board_spi_transfer16(cmd);
    board_delay_us(5);
    cs_high(board);
    board_delay_us(s_acc_us);

    cs_low(board);
    board_delay_us(s_acc_us);
    rx = board_spi_transfer16(cmd);
    board_delay_us(5);
    cs_high(board);
    board_delay_us(s_acc_us);

    return (uint16_t)(rx & 0x0FFFU);
}

/*----------------------------------------------------------------------------
 * 扫描
 *--------------------------------------------------------------------------*/
/* 读一个 bank 的 16 通道：原工程师的 18 帧流水线协议 */
static void read_bank(uint8_t board, uint16_t *dst)
{
    uint16_t f;
    uint16_t rx;

    for (f = 0U; f < (ADC64_COLS + 2U); f++) {
        uint16_t ch = (f < ADC64_COLS) ? f : 0U;
        uint16_t cmd = (uint16_t)(0x1000U | ((ch & 0x0FU) << 7));

        cs_low(board);
        board_delay_us(s_acc_us);
        rx = board_spi_transfer16(cmd);
        board_delay_us(5);
        cs_high(board);
        board_delay_us(50);

        if (f >= 2U) {
            dst[f - 2U] = (uint16_t)(rx & 0x0FFFU);
        }
    }
}

void adc64_scan(adc64_frame_t *f)
{
    uint8_t r;
    uint8_t b;
    uint8_t c;

    {
        uint8_t w;
        for (w = 0U; w < 4U; w++) {
            cs_low(0U);
            board_delay_us(s_acc_us);
            (void)board_spi_transfer16(0x0000U);
            cs_high(0U);
            board_delay_us(50);
        }
    }

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        f->sum[b] = 0U;
    }

    if (s_en_hold != 0U) {
        adc64_set_en_mask(s_en_hold);
        board_delay_us(ADC64_EN_SETTLE_US);
    }

    for (r = 0U; r < ADC64_ROWS; r++) {
        if (s_en_hold == 0U) {
            uint16_t dummy[ADC64_COLS];

            en_on(r);
            board_delay_us(ADC64_EN_SETTLE_US);
            /* 预扫一轮（丢弃），让模拟通路稳定 */
            read_bank(0U, dummy);
        }

        for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
            uint16_t buf[ADC64_COLS];
            uint8_t k;

            read_bank(b, buf);
            for (k = 0U; k < ADC64_COLS; k++) {
                uint32_t bit = (uint32_t)r * ADC64_COLS + (uint32_t)k;

                f->cell[b][r][k] = buf[k];
                if (((s_mask[b] >> bit) & 1ULL) != 0ULL) {
                    f->sum[b] += (uint32_t)buf[k];
                }
            }
        }
    }
    en_off_all();
}

void adc64_scan_dark(adc64_frame_t *f)
{
    uint8_t b;
    uint8_t r;
    uint8_t c;

    en_off_all();
    board_delay_us(ADC64_EN_SETTLE_US);

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        f->sum[b] = 0U;
    }
    for (r = 0U; r < ADC64_ROWS; r++) {
        for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
            for (c = 0U; c < ADC64_COLS; c++) {
                f->cell[b][r][c] = read_frame(b, c);
            }
        }
    }
}

/*----------------------------------------------------------------------------
 * 掩码
 *--------------------------------------------------------------------------*/
void adc64_set_mask(uint8_t board, uint64_t mask)
{
    if (board < ADC64_BOARD_COUNT) {
        s_mask[board] = mask;
    }
}

uint64_t adc64_mask(uint8_t board)
{
    return (board < ADC64_BOARD_COUNT) ? s_mask[board] : 0ULL;
}

void adc64_default_masks(void)
{
    uint8_t b;

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        s_mask[b] = 0xFFFFFFFFFFFFFFFFULL;
    }
}

/* 调试：打印 16 个通道的原始 16 位帧 */
void adc64_echo_test(uint8_t board, uint16_t *out)
{
    uint8_t c;
    uint16_t cmd;
    uint16_t rx;

    for (c = 0U; c < ADC64_COLS; c++) {
        cmd = (uint16_t)((uint16_t)c << s_cmd_shift);
        cs_low(board);
        board_delay_us(s_acc_us);
        (void)board_spi_transfer16(cmd);
        board_delay_us(5);
        cs_high(board);
        board_delay_us(50);
        cs_low(board);
        board_delay_us(s_acc_us);
        rx = board_spi_transfer16(cmd);
        board_delay_us(5);
        cs_high(board);
        board_delay_us(50);
        out[c] = rx;
    }
}

/* 只读一列（4 行）—— 调试用 */
void adc64_read_column(uint8_t ch, uint16_t *out)
{
    uint8_t r;
    uint8_t wm;
    uint8_t k;

    /* 与 adc64_scan 一致：先发 4 个 0x0000 强制进入手动模式 */
    for (k = 0U; k < 4U; k++) {
        cs_low(0U);
        board_delay_us(s_acc_us);
        (void)board_spi_transfer16(0x0000U);
        cs_high(0U);
        board_delay_us(50);
    }

    for (r = 0U; r < ADC64_ROWS; r++) {
        en_on(r);
        board_delay_us(ADC64_EN_SETTLE_US);
        /* 与 adc64_scan 完全一致的预热：两轮全通道（32 次） */
        for (wm = 0U; wm < 16U; wm++) {
            (void)read_frame(0U, wm);
        }
        for (wm = 0U; wm < 16U; wm++) {
            (void)read_frame(0U, wm);
        }
        out[r] = read_frame(0U, ch);
        for (k = 0U; k < 2U; k++) {
            out[r] = read_frame(0U, ch);
        }
    }
    en_off_all();
}

/* 长边 43 颗的最大值（用于闭环判据） */
uint32_t adc64_long_max(void)
{
    adc64_frame_t fr;
    uint16_t i;
    uint32_t mx = 0U;
    uint32_t v;

    adc64_scan(&fr);
    for (i = 21U; i < 64U; i++) {
        v = (uint32_t)fr.cell[0][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)];
        if (v > mx) { mx = v; }
    }
    return mx;
}
/* 快速版：只扫板 A 的 4 个 bank，返回长边 43 颗的最大值 */
uint32_t adc64_long_max_fast(void)
{
    uint8_t  r;
    uint8_t  k;
    uint16_t buf[ADC64_COLS];
    uint32_t mx = 0U;

    for (r = 0U; r < ADC64_ROWS; r++) {
        en_on(r);
        board_delay_us(ADC64_EN_SETTLE_US);
        read_bank(0U, buf);                       /* 板 A */
        for (k = 5U; k < ADC64_COLS; k++) {       /* 列 5~F = 长边 */
            if ((uint32_t)buf[k] > mx) { mx = (uint32_t)buf[k]; }
        }
    }
    en_off_all();
    return mx;
}
/* 归一化集中度 CONC×100 = MAX*4300/SUM（长边 43 颗）
   垂直入射时 ~950~1150，斜入射时 ~500~650 */
uint32_t adc64_long_conc(void)
{
    uint8_t  r;
    uint8_t  k;
    uint16_t buf[ADC64_COLS];
    uint32_t mx  = 0U;
    uint32_t sum = 0U;

    for (r = 0U; r < ADC64_ROWS; r++) {
        en_on(r);
        board_delay_us(ADC64_EN_SETTLE_US);
        read_bank(0U, buf);
        for (k = 5U; k < ADC64_COLS; k++) {
            sum += (uint32_t)buf[k];
            if ((uint32_t)buf[k] > mx) { mx = (uint32_t)buf[k]; }
        }
    }
    en_off_all();

    if (sum == 0U) { return 0U; }
    return (mx * 4300U) / sum;
}
/* 长边 43 颗的分布宽度 sigma×100（越小越垂直）
   垂直 ~200，斜射 ~480；光太弱(<1000)时返回 9999 视为无效 */
uint32_t adc64_long_width(void)
{
    uint8_t  r;
    uint8_t  k;
    uint16_t buf[ADC64_COLS];
    uint32_t s0 = 0U;
    uint32_t s1 = 0U;
    uint32_t s2 = 0U;
    uint32_t mx = 0U;
    uint32_t v;
    uint32_t n;
    uint32_t p;
    int64_t  mean_num;
    int64_t  var_num;
    uint32_t var;
    uint32_t x;

    for (r = 0U; r < ADC64_ROWS; r++) {
        en_on(r);
        board_delay_us(ADC64_EN_SETTLE_US);
        read_bank(0U, buf);
        for (k = 5U; k < ADC64_COLS; k++) {
            v = (uint32_t)buf[k];
            n = (uint32_t)(r * ADC64_COLS + k);   /* 用 r,k 组合成序号 */
            p = (uint32_t)((k - 5U) * ADC64_ROWS + r);
            s0 += v;
            s1 += v * p;
            s2 += v * p * p;
            if (v > mx) { mx = v; }
            (void)n;
        }
    }
    en_off_all();

    if ((s0 == 0U) || (mx < 1000U)) { return 9999U; }

    mean_num = (int64_t)s1;
    var_num  = (int64_t)s2 * (int64_t)s0 - (int64_t)s1 * (int64_t)s1;
    if (var_num <= 0) { return 0U; }
    var = (uint32_t)(var_num / ((int64_t)s0 * (int64_t)s0));

    /* 整数平方根 */
    x = var;
    {
        uint32_t y = (x + 1U) / 2U;
        while (y < x) { x = y; y = (x + var / x) / 2U; }
    }
    return x * 100U;
}