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
/* 板 C 的行选（FPC13 pin6-9 -> EN13..EN16 = PG3/PG2/PD15/PD14） */
static const en_t k_en_c[ADC64_ROWS] = {
    { GPIOG,  3U }, { GPIOG,  2U }, { GPIOD, 15U }, { GPIOD, 14U }
};

/* 板 D 的行选（FPC14 pin6-9 -> EN09..EN12 = PG7/PG6/PG5/PG4） */
static const en_t k_en_d[ADC64_ROWS] = {
    { GPIOG,  7U }, { GPIOG,  6U }, { GPIOG,  5U }, { GPIOG,  4U }
};

static cs_t k_cs[ADC64_BOARD_COUNT] = {
    { ADC64_A_CS_PORT, ADC64_A_CS_PIN },   /* E0 下长边 */
    { ADC64_B_CS_PORT, ADC64_B_CS_PIN },   /* E1 右短边 */
    /* C/D 暂不参与扫描（4 板扫描路径待修） */

};

/* 四条边的通道范围：索引 i -> 列=i/4，行=i%4+1 */
typedef struct
{
    uint8_t  i0;        /* 起始索引 */
    uint8_t  i1;        /* 结束索引（含） */
    uint8_t  rev;       /* 1 = 逆序 */
} edge_t;

static const edge_t k_edges[4] = {
    { 21U, 63U, 0U },   /* E0 下长边 D25->D4F  43 颗 */
    { 22U, 39U, 0U },   /* E1 右短边 D35->D49  18 颗 */
    {  7U, 22U, 0U },   /* E2 上长边 D41->D35  16 颗 */
    { 22U, 42U, 0U }    /* E3 左短边 D35->D3a  21 颗 */
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
        en_low(&k_en_c[r]);
        en_low(&k_en_d[r]);
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
            en_high(&k_en_c[r]);
            en_high(&k_en_d[r]);
        } else {
            en_low(&k_en_a[r]);
            en_low(&k_en_b[r]);
            en_low(&k_en_c[r]);
            en_low(&k_en_d[r]);
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

    /* 新增两板的片选：PC13（上长边 FPC13）/ PI9（左短边 FPC14） */
    /* PC13 属备份域：需先开 PWR 时钟并解除写保护 */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << 26)) | (1UL << 26);
    GPIOC->OTYPER &= ~(1UL << 13);
    GPIOC->OSPEEDR |= (3UL << 26);
    GPIOC->BSRR = (1UL << 13);

    GPIOI->MODER = (GPIOI->MODER & ~(3UL << 18)) | (1UL << 18);
    GPIOI->OTYPER &= ~(1UL << 9);
    GPIOI->OSPEEDR |= (3UL << 18);
    GPIOI->BSRR = (1UL << 9);
    GPIOH->BSRR  = (1UL << 4);

    /* EN 行选时钟 */
    RCC->AHB1ENR |= ADC64_EN_GPIO_AHB1EN | RCC_AHB1ENR_GPIOHEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    (void)RCC->AHB1ENR;
    (void)RCC->AHB1ENR;

    for (i = 0U; i < ADC64_ROWS; i++) {
        en_gpio_cfg(&k_en_a[i]);
        en_gpio_cfg(&k_en_c[i]);
        en_gpio_cfg(&k_en_d[i]);
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
    board_delay_us(60U);
    (void)board_spi_transfer16(cmd);
    board_delay_us(5);
    cs_high(board);
    board_delay_us(60U);

    cs_low(board);
    board_delay_us(60U);
    rx = board_spi_transfer16(cmd);
    board_delay_us(5);
    cs_high(board);
    board_delay_us(60U);

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
        board_delay_us(60U);
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
            board_delay_us(60U);
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
        board_delay_us(60U);
        (void)board_spi_transfer16(cmd);
        board_delay_us(5);
        cs_high(board);
        board_delay_us(50);
        cs_low(board);
        board_delay_us(60U);
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
        board_delay_us(60U);
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
    static adc64_frame_t fr;
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
/*============================================================================
 * 数据处理链：EMA 滤波 -> 暗电流扣除 -> 阈值 -> 重心定位 -> 峰值
 *==========================================================================*/
static uint16_t s_ema[2][ADC64_ROWS][ADC64_COLS];
static uint8_t  s_ema_init = 0U;

typedef struct
{
    uint16_t center;      /* 重心位置，单位 0.1 个索引 */
    uint16_t maxv;        /* 峰值 */
    uint32_t total;       /* 有效总和 */
    uint8_t  valid;       /* 1 = 有有效信号 */
} meas_t;

static void ema_filter(const adc64_frame_t *fr, uint16_t *out_l, uint16_t *out_s)
{
    uint16_t i;
    uint16_t raw;
    uint16_t v;

    for (i = 0U; i < 64U; i++) {
        uint8_t col = (uint8_t)(i / 4U);
        uint8_t row = (uint8_t)(i % 4U);
        /* 长边用板 A，短边用板 B */
        raw = fr->cell[0][row][col];
        v = (uint16_t)(((uint32_t)s_ema[0][row][col] * 3U + raw) / 4U);
        if (s_ema_init == 0U) { v = raw; }
        s_ema[0][row][col] = v;
        if (i >= 21U) { out_l[i - 21U] = v; }
    }
    for (i = 22U; i < 40U; i++) {
        uint8_t col = (uint8_t)(i / 4U);
        uint8_t row = (uint8_t)(i % 4U);
        raw = fr->cell[1][row][col];
        v = (uint16_t)(((uint32_t)s_ema[1][row][col] * 3U + raw) / 4U);
        if (s_ema_init == 0U) { v = raw; }
        s_ema[1][row][col] = v;
        out_s[i - 22U] = v;
    }
    s_ema_init = 1U;
}

/* 由一维数组求重心（0.1 单位）和峰值 */
static void measure_1d(const uint16_t *v, uint16_t n, meas_t *m)
{
    uint16_t i;
    uint32_t mx = 0U;
    uint32_t thr;
    uint32_t s0 = 0U;
    uint32_t s1 = 0U;

    m->valid = 0U;
    m->center = 0U;
    m->maxv = 0U;
    m->total = 0U;

    for (i = 0U; i < n; i++) {
        if ((uint32_t)v[i] > mx) { mx = (uint32_t)v[i]; }
    }
    if (mx < 300U) { return; }              /* 无有效光 */

    /* 阈值 = 峰值的 35%，滤掉噪声和拖尾 */
    thr = mx * 35U / 100U;
    if (thr < 200U) { thr = 200U; }

    for (i = 0U; i < n; i++) {
        if ((uint32_t)v[i] >= thr) {
            s0 += (uint32_t)v[i];
            s1 += (uint32_t)v[i] * (uint32_t)i;
        }
    }
    if (s0 == 0U) { return; }

    m->valid  = 1U;
    m->maxv   = (uint16_t)mx;
    m->total  = s0;
    m->center = (uint16_t)((s1 * 10U) / s0);   /* 0.1 索引 */
}

/* 供命令层调用：一次采集 + 全线处理 */
void adc64_measure(uint16_t *x01, uint16_t *y01, uint16_t *maxl,
                   uint16_t *maxs, uint8_t *valid)
{
    static adc64_frame_t fr;
    static uint16_t vl[43];
    static uint16_t vs[18];
    meas_t ml;
    meas_t ms;

    adc64_set_en_hold(0U);
    adc64_scan(&fr);
    ema_filter(&fr, vl, vs);
    measure_1d(vl, 43U, &ml);
    measure_1d(vs, 18U, &ms);

    *x01   = ml.center;
    *y01   = ms.center;
    *maxl  = ml.maxv;
    *maxs  = ms.maxv;
    *valid = (uint8_t)(ml.valid | (uint8_t)(ms.valid << 1));
}

void adc64_measure_reset(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t k;

    for (i = 0U; i < 2U; i++) {
        for (j = 0U; j < ADC64_ROWS; j++) {
            for (k = 0U; k < ADC64_COLS; k++) {
                s_ema[i][j][k] = 0U;
            }
        }
    }
    s_ema_init = 0U;
}

/* 读取一条边：board=板号, e=边号，输出到 out（最多 64 个） */
uint16_t adc64_read_edge(uint8_t board, uint8_t e, uint16_t *out)
{
    static adc64_frame_t fr;
    uint16_t n = 0U;
    uint8_t  i;
    uint8_t  i0;
    uint8_t  i1;

    if ((board >= ADC64_BOARD_COUNT) || (e >= 4U)) { return 0U; }

    adc64_set_en_hold(0U);
    adc64_scan(&fr);

    i0 = k_edges[e].i0;
    i1 = k_edges[e].i1;

    if (k_edges[e].rev == 0U) {
        for (i = i0; i <= i1; i++) {
            out[n] = fr.cell[board][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)];
            n++;
        }
    } else {
        for (i = i1; ; i--) {
            out[n] = fr.cell[board][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)];
            n++;
            if (i == i0) { break; }
        }
    }
    return n;
}

/* 一次扫描，提取四条边的全部数据（避免重复扫描） */
/* 把两块板的片选都拉高（未选中），避免并联 MISO 冲突 */
static void cs_all_high(void)
{
    k_cs[0].port->BSRR = (1UL << k_cs[0].pin);
    k_cs[1].port->BSRR = (1UL << k_cs[1].pin);
}

/* 提取一条边到目标缓冲（从 fr.cell[src] 取） */
static uint8_t edge_extract(const adc64_frame_t *fr, uint8_t src,
                            uint8_t e, uint16_t *dst)
{
    uint8_t  i;
    uint8_t  i0 = k_edges[e].i0;
    uint8_t  i1 = k_edges[e].i1;
    uint16_t c  = 0U;

    if (k_edges[e].rev == 0U) {
        for (i = i0; i <= i1; i++) {
            dst[c] = fr->cell[src][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)];
            c++;
        }
    } else {
        for (i = i1; ; i--) {
            dst[c] = fr->cell[src][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)];
            c++;
            if (i == i0) { break; }
        }
    }
    return (uint8_t)c;
}

void adc64_read_edges_all(uint16_t *b0, uint16_t *b1, uint16_t *b2, uint16_t *b3,
                          uint8_t *n_out)
{
    static adc64_frame_t fr;
    cs_t save;

    /* ---- 第一遍：板 A(下长边) + 板 B(右短边) ---- */
    adc64_set_en_hold(0U);
    adc64_scan(&fr);
    n_out[0] = edge_extract(&fr, 0U, 0U, b0);
    n_out[1] = edge_extract(&fr, 1U, 1U, b1);

    /* ---- 第二遍：板 C 上长边，片选临时切到 PC13 ---- */
    save = k_cs[0];
    k_cs[0].port = ADC64_C_CS_PORT;
    k_cs[0].pin  = ADC64_C_CS_PIN;
    k_cs[0].port->BSRR = (1UL << ADC64_C_CS_PIN);      /* 先拉高 */
    adc64_scan(&fr);
    k_cs[0] = save;
    n_out[2] = edge_extract(&fr, 0U, 2U, b2);

    /* ---- 第三遍：板 D 左短边，片选临时切到 PI9 ---- */
    save = k_cs[0];
    k_cs[0].port = ADC64_D_CS_PORT;
    k_cs[0].pin  = ADC64_D_CS_PIN;
    k_cs[0].port->BSRR = (1UL << ADC64_D_CS_PIN);
    adc64_scan(&fr);
    k_cs[0] = save;
    n_out[3] = edge_extract(&fr, 0U, 3U, b3);
}
/*----------------------------------------------------------------------------
 * 临时片选探测：把 k_cs[0] 换成指定的 port/pin，扫一次，取指定边的峰值
 * 不改变 BOARD_COUNT，不影响原有 2 板逻辑
 *--------------------------------------------------------------------------*/
static void gpio_clock_on(GPIO_TypeDef *p)
{
    if (p == GPIOA)      { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; }
    else if (p == GPIOB) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; }
    else if (p == GPIOC) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; }
    else if (p == GPIOD) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; }
    else if (p == GPIOE) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; }
    else if (p == GPIOF) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN; }
    else if (p == GPIOG) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; }
    else if (p == GPIOH) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; }
    else if (p == GPIOI) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; }
    else                 { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN; }
}

static void pin_set_output(GPIO_TypeDef *p, uint8_t pin)
{
    uint32_t sh = (uint32_t)pin * 2U;
    gpio_clock_on(p);
    p->MODER = (p->MODER & ~(3UL << sh)) | (1UL << sh);
    p->OTYPER &= ~(1UL << pin);
    p->OSPEEDR |= (3UL << sh);
    p->BSRR = (1UL << pin);            /* 默认高 = 未选中 */
}

/* 用指定 CS 扫描一次，返回该边峰值 */
uint32_t adc64_probe_cs(GPIO_TypeDef *port, uint8_t pin, uint8_t edge)
{
    static adc64_frame_t fr;
    cs_t save;
    uint8_t i;
    uint8_t i0;
    uint8_t i1;
    uint32_t mx = 0U;
    uint16_t v;

    pin_set_output(port, pin);

    save = k_cs[0];
    k_cs[0].port = port;
    k_cs[0].pin  = pin;
    adc64_scan(&fr);
    k_cs[0] = save;

    i0 = k_edges[edge].i0;
    i1 = k_edges[edge].i1;
    for (i = i0; i <= i1; i++) {
        v = fr.cell[0][(uint8_t)(i % 4U)][(uint8_t)(i / 4U)];
        if ((uint32_t)v > mx) { mx = (uint32_t)v; }
    }
    return mx;
}

/* 运行时改片选：board 0/1 是 A/B，2/3 视作"临时借用板0的片选"
 * port: 'A'..'J'  pin: 0..15 */
uint8_t adc64_set_cs_pin(uint8_t slot, char port, uint8_t pin)
{
    GPIO_TypeDef *p;
    uint32_t sh;

    switch (port) {
        case 'A': p = GPIOA; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; break;
        case 'B': p = GPIOB; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; break;
        case 'C': p = GPIOC; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; break;
        case 'D': p = GPIOD; RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; break;
        case 'E': p = GPIOE; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; break;
        case 'F': p = GPIOF; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN; break;
        case 'G': p = GPIOG; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; break;
        case 'H': p = GPIOH; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; break;
        case 'I': p = GPIOI; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; break;
        default:  p = GPIOJ; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN; break;
    }
    (void)RCC->AHB1ENR;

    sh = (uint32_t)pin * 2U;
    p->MODER   = (p->MODER & ~(3UL << sh)) | (1UL << sh);
    p->OTYPER &= ~(1UL << pin);
    p->OSPEEDR |= (3UL << sh);
    p->BSRR    = (1UL << pin);          /* 默认高，未选中 */

    if (slot > 1U) { slot = 0U; }       /* 2/3 借用板 0 的位置 */
    k_cs[slot].port = p;
    k_cs[slot].pin  = pin;
    return 1U;
}

/* ---- 诊断包装（供 K T1~T4 使用）---- */
void adc64_diag_delay(uint32_t us) { board_delay_us(us); }
uint16_t adc64_diag_spi(void)
{
    uint16_t r;
    cs_low(0U);
    board_delay_us(5U);
    r = board_spi_transfer16(0x1000U);
    cs_high(0U);
    return r;
}
void adc64_diag_bank(uint16_t *dst)
{
    adc64_set_en_hold(0U);
    en_on(0U);
    board_delay_us(2000U);
    read_bank(0U, dst);
    en_off_all();
}

/* 片选直接存取（供候选扫描用） */
cs_t k_cs_direct_get(uint8_t b) { return k_cs[b]; }
void k_cs_direct_set(uint8_t b, GPIO_TypeDef *port, uint8_t pin)
{
    uint32_t sh = (uint32_t)pin * 2U;
    if (port == GPIOC) { RCC->APB1ENR |= RCC_APB1ENR_PWREN; PWR->CR |= PWR_CR_DBP; }
    if (port == GPIOC) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; }
    else if (port == GPIOI) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; }
    else if (port == GPIOH) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; }
    else if (port == GPIOG) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; }
    else if (port == GPIOF) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN; }
    else { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; }
    (void)RCC->AHB1ENR;

    port->MODER = (port->MODER & ~(3UL << sh)) | (1UL << sh);
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << sh);
    port->BSRR = (1UL << pin);

    k_cs[b].port = port;
    k_cs[b].pin  = pin;
}
void k_cs_direct_restore(uint8_t b, cs_t v) { k_cs[b] = v; }
