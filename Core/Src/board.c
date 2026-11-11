/**
  ******************************************************************************
  * @file    board.c
  * @brief   板级硬件抽象实现：时钟树、SysTick、GPIO 复用、舵�?PWM
  *
  *          全部为寄存器级操作，不依赖标准外设库 / HAL�?
  ******************************************************************************
  */
#include "board.h"

/* APB 分频系数表：PPREx[2:0] 编码 -> 实际分频
 * 0xx = /1, 100 = /2, 101 = /4, 110 = /8, 111 = /16
 */
static const uint8_t k_apb_div[8] = { 1U, 1U, 1U, 1U, 2U, 4U, 8U, 16U };

static volatile uint32_t s_ms      = 0;   /* 毫秒计数，SysTick 里自�?*/
static uint8_t           s_hse_ok  = 0;   /* 1 = 当前主频来自 HSE + PLL */

/* PLL 参数：VCO 输入固定 1MHz，VCO = 360MHz，SYSCLK = 180MHz
 * PLLQ 只给 USB/SDIO 用，本板命令口走 CH340C 不需�?48MHz，取 8 -> 45MHz�?
 * 若以后要�?MCU 原生 USB：PLL_N_VALUE �?336、PLL_Q_VALUE �?7（得�?168MHz）�?*/
#define PLL_N_VALUE      360U
#define PLL_P_VALUE      2U
#define PLL_Q_VALUE      8U
#define PLL_WAIT_COUNT   0x100000UL
#define HSE_WAIT_COUNT   0x50000UL

/* 当前活动的 SPI 外设：SPI4(下/右) 或 SPI5(上/左) */
static volatile SPI_TypeDef *s_active_spi = SPI4;

/*----------------------------------------------------------------------------
 * 内部工具
 *--------------------------------------------------------------------------*/
static uint32_t apb_div_get(uint32_t ppre_bits)
{
    return (uint32_t)k_apb_div[ppre_bits & 0x7U];
}

/* 该定时器挂在 APB2 上吗（TIM1/TIM8/TIM9/10/11 �?APB2�?*/
static uint8_t tim_is_on_apb2(TIM_TypeDef *TIMx)
{
    return (uint8_t)((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) ||
                     (TIMx == TIM10) || (TIMx == TIM11));
}

/*----------------------------------------------------------------------------
 * 时钟初始化：8MHz HSE -> 180MHz（F429IGT6�?
 *----------------------------------------------------------------------------*/
void board_clock_init(void)
{
    uint32_t timeout;
    uint32_t src_hz;
    uint32_t pllm;
    uint8_t  hse_ok;

    /* --- 1. 打开 HSI 并切�?HSI 作为当前系统时钟，保证改 PLL �?CPU 有稳定时�?--- */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
    }
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
    }

    /* --- 2. 关闭 PLL，准备重新配�?--- */
    RCC->CR &= ~RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
    }

    /* --- 3. 使能外部晶振，带超时；失败则整机改用 HSI 方案 --- */
    RCC->CR |= RCC_CR_HSEON;
    timeout = HSE_WAIT_COUNT;
    while (((RCC->CR & RCC_CR_HSERDY) == 0U) && (timeout != 0U)) {
        timeout--;
    }
    hse_ok = ((RCC->CR & RCC_CR_HSERDY) != 0U) ? 1U : 0U;
    if (hse_ok == 0U) {
        RCC->CR &= ~RCC_CR_HSEON;                 /* 晶振有问题，不要继续�?*/
    }

    /* --- 4. Flash 等待周期�?68MHz 需�?5WS，并打开预取�?I/D Cache --- */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN |
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) {
    }

    /* --- 5. 总线分频：AHB=/1(168M) APB1=/4(42M) APB2=/2(84M) --- */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV4
              | RCC_CFGR_PPRE2_DIV2;

    /* --- 6. 配置 PLL ----------------------------------------------------------
     *  VCO 输入 = 时钟�?/ PLLM，这里统一做成 1MHz�?
     *      HSE 8MHz  -> M = 8
     *      HSI 16MHz -> M = 16
     *  VCO = 1MHz * 336 = 336MHz，SYSCLK = 336/2 = 168MHz，PLL48 = 336/7 = 48MHz
     *----------------------------------------------------------------------- */
    src_hz = (hse_ok != 0U) ? BOARD_HSE_HZ : 16000000UL;
    pllm   = src_hz / 1000000UL;
    if (pllm == 0U) {
        pllm = 1U;
    }

    RCC->PLLCFGR = (pllm << RCC_PLLCFGR_PLLM_Pos)
                 | (PLL_N_VALUE << RCC_PLLCFGR_PLLN_Pos)
                 | (((PLL_P_VALUE >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)
                 | (PLL_Q_VALUE << RCC_PLLCFGR_PLLQ_Pos)
                 | ((hse_ok != 0U) ? RCC_PLLCFGR_PLLSRC_HSE : 0U);

    RCC->CR |= RCC_CR_PLLON;
    timeout = PLL_WAIT_COUNT;
    while (((RCC->CR & RCC_CR_PLLRDY) == 0U) && (timeout != 0U)) {
        timeout--;
    }

    if (timeout == 0U) {
        /* PLL 没锁上：说明晶振频率与配置不匹配（比�?25MHz 板子配了 8MHz�?
         * 此时保持 HSI 直接作主频（16MHz），PWM 与波特率仍按实际时钟自动计算�?
         * 系统照样能跑，只是主频低�?*/
        RCC->CR &= ~RCC_CR_PLLON;
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2));
        hse_ok = 0U;
    } else {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        }
    }

    s_hse_ok = hse_ok;
    SystemCoreClockUpdate();       /* �?SystemCoreClock 与实际主频一�?*/
}

uint8_t board_is_hse_clock(void)
{
    return s_hse_ok;
}

/*----------------------------------------------------------------------------
 * 总线时钟
 *----------------------------------------------------------------------------*/
uint32_t board_pclk1_hz(void)
{
    uint32_t div = apb_div_get((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);
    return SystemCoreClock / div;
}

uint32_t board_pclk2_hz(void)
{
    uint32_t div = apb_div_get((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos);
    return SystemCoreClock / div;
}

/* 定时器计数时钟：APB 分频不为 1 时，定时器时�?= PCLK * 2 */
uint32_t board_timer_clk_hz(TIM_TypeDef *TIMx)
{
    uint32_t div;
    uint32_t pclk;

    if (tim_is_on_apb2(TIMx) != 0U) {
        div  = apb_div_get((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos);
    } else {
        div  = apb_div_get((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);
    }
    pclk = SystemCoreClock / div;
    return (div == 1U) ? pclk : (pclk * 2U);
}

/*----------------------------------------------------------------------------
 * SysTick 1kHz 时基
 *----------------------------------------------------------------------------*/
void board_tick_init(void)
{
    (void)SysTick_Config(SystemCoreClock / BOARD_SYSTICK_HZ);
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 2UL);   /* 低于串口，高于无 */

    /* 打开 DWT 周期计数器，�?board_delay_us 使用 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 微秒级忙等：DWT 周期计数，误�?< 1 个主频周�?*/
void board_delay_us(uint32_t us)
{
    /* 纯软件循环延时，不依赖 DWT（DWT 在部分调试场景下 CYCCNT 不计数） */
    volatile uint32_t n;

    if (us == 0U) { return; }
    if (us > 500000U) { us = 500000U; }        /* 上限保护 */

    /* 180MHz 下每次循环约 6 个周期 -> 约 30 次/微秒 */
    n = us * 30U;
    while (n != 0U) { n--; }
}

void board_tick_1ms(void)
{
    s_ms++;
}

uint32_t board_millis(void)
{
    return s_ms;
}

void board_delay_ms(uint32_t ms)
{
    uint32_t start = s_ms;
    /* 用无符号减法，天然处�?49 天溢出回�?*/
    while ((uint32_t)(s_ms - start) < ms) {
        __WFI();                   /* 等中断，不空转耗电 */
    }
}

/*----------------------------------------------------------------------------
 * GPIO 复用推挽输出配置
 *   pull: 0=浮空 1=上拉 2=下拉
 *----------------------------------------------------------------------------*/
void board_gpio_af_config(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t af, uint32_t pull)
{
    uint32_t pos = pin * 2U;

    GPIOx->MODER   = (GPIOx->MODER   & ~(3UL << pos)) | (2UL << pos); /* 复用功能 */
    GPIOx->OTYPER &= ~(1UL << pin);                                   /* 推挽 */
    GPIOx->OSPEEDR = (GPIOx->OSPEEDR & ~(3UL << pos)) | (2UL << pos); /* 高�?*/
    GPIOx->PUPDR   = (GPIOx->PUPDR   & ~(3UL << pos)) | (pull << pos);

    if (pin < 8U) {
        uint32_t shift = pin * 4U;
        GPIOx->AFR[0] = (GPIOx->AFR[0] & ~(0xFUL << shift)) | (af << shift);
    } else {
        uint32_t shift = (pin - 8U) * 4U;
        GPIOx->AFR[1] = (GPIOx->AFR[1] & ~(0xFUL << shift)) | (af << shift);
    }
}

/*----------------------------------------------------------------------------
 * 舵机 PWM：TIM4_CH1(PD12)�?0Hz�?us/计数
 *
 *  PSC 由实际定时器时钟算出，因此即�?HSE 失败退�?HSI，输出依然是精确�?
 *  50Hz / 1us，不会影响舵机定位�?
 *----------------------------------------------------------------------------*/
/* board_servo_pwm_init: 见下方软件 PWM 实现 */

/* board_servo_pwm_set_pulse: 见下方软件 PWM 实现 */

/* board_servo_pwm_enable: 见下方软件 PWM 实现 */

/*----------------------------------------------------------------------------
 * 探测�?ADC：ADC1，软件触发单次转换，12bit
 *   引脚�?config.h �?TRK_ADC_PINS / TRK_ADC_CHANNELS 配置
 *   采样时间 84 周期（ADC 时钟 22.5MHz 时约 3.7us），适合高阻抗光敏信�?
 *----------------------------------------------------------------------------*/
void board_adc_init(void)
{
    const uint8_t pins[TRK_CH_COUNT] = TRK_ADC_PINS;
    uint32_t i;

    RCC->AHB1ENR |= TRK_ADC_GPIO_AHB1EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    /* 引脚 -> 模拟模式（MODER=11），模拟模式不受内部上下拉影�?*/
    for (i = 0U; i < TRK_CH_COUNT; i++) {
        uint32_t pos = (uint32_t)pins[i] * 2U;
        TRK_ADC_GPIO->MODER = (TRK_ADC_GPIO->MODER & ~(3UL << pos)) | (3UL << pos);
    }

    /* ADC 时钟 = PCLK2 / 4 = 22.5MHz（F429 上限 36MHz�?*/
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1UL << 16);

    ADC1->CR1  = 0U;
    ADC1->CR2  = 0U;
    ADC1->SQR1 = 0U;                       /* 序列长度 1 */
    ADC1->SMPR1 = 0x24924924UL;            /* 每个通道 84 周期�?bit 场全�?100b�?*/
    ADC1->SMPR2 = 0x24924924UL;
    ADC1->CR2  = ADC_CR2_ADON;

    /* 上电稳定时间，随后丢弃一次转�?*/
    board_delay_ms(1U);
    (void)board_adc_read(0U);
}

uint16_t board_adc_read(uint8_t ch)
{
    uint32_t guard = 200000UL;

    ADC1->SQR3 = (uint32_t)ch;             /* �?1 个转�?= ch */
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (((ADC1->SR & ADC_SR_EOC) == 0U) && (guard != 0U)) {
        guard--;
    }
    return (uint16_t)(ADC1->DR & 0x0FFFU); /* �?DR 同时�?EOC */
}

/*----------------------------------------------------------------------------
 * SPI：两�?ADC64 采集板各占一�?SPI�? = SPI4 长边�? = SPI5 短边�?
 *      8 位数据帧拼成 16 位，片选由 adc64.c �?GPIO 控制
 *----------------------------------------------------------------------------*/
static uint8_t spi8(uint8_t d)
{
    uint32_t guard;
    guard = 200000UL;
    while (((s_active_spi->SR & SPI_SR_TXE) == 0U) && (guard != 0U)) { guard--; }
    *(volatile uint8_t *)&ADC64_SPI->DR = d;
    guard = 200000UL;
    while (((s_active_spi->SR & SPI_SR_RXNE) == 0U) && (guard != 0U)) { guard--; }
    return *(volatile uint8_t *)&ADC64_SPI->DR;
}


/* 切换当前使用的 SPI 外设：0=SPI4(下/右) 1=SPI5(上/左) */
void board_spi_select(uint8_t which)
{
    s_active_spi = (which != 0U) ? SPI3 : SPI4;
}

void board_spi_init(void)
{
    RCC->AHB1ENR |= ADC64_GPIO_AHB1EN;
    RCC->APB2ENR |= ADC64_SPI_APB2EN;
    (void)RCC->AHB1ENR; (void)RCC->APB2ENR;
    board_gpio_af_config(ADC64_GPIO, ADC64_SCK_PIN,  ADC64_AF, 0U);
    board_gpio_af_config(ADC64_GPIO, ADC64_MISO_PIN, ADC64_AF, 0U);
    board_gpio_af_config(ADC64_GPIO, ADC64_MOSI_PIN, ADC64_AF, 0U);
    ADC64_SPI->CR1 = 0U;
    ADC64_SPI->CR2 = 0U;
    /* DFF=1: 16 位数据帧，一次传输产生连续 16 个 SCLK（ADS7953 要求） */
    ADC64_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_DFF
                   | ((uint32_t)ADC64_SPI_BR << 3) | SPI_CR1_SPE;
}

void board_spi_set_mode(uint8_t mode)
{
    uint32_t cr1 = ADC64_SPI->CR1;
    cr1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    if ((mode & 2U) != 0U) { cr1 |= SPI_CR1_CPOL; }
    if ((mode & 1U) != 0U) { cr1 |= SPI_CR1_CPHA; }
    cr1 |= SPI_CR1_SPE;
    ADC64_SPI->CR1 = cr1;
}


uint16_t board_spi_transfer16(uint16_t tx)
{
    uint32_t guard;
    uint16_t rx;

    guard = 200000UL;
    while (((s_active_spi->SR & SPI_SR_TXE) == 0U) && (guard != 0U)) { guard--; }
    if (guard == 0U) { return 0xFFFFU; }        /* 发不出去 */
    *(volatile uint16_t *)&s_active_spi->DR = tx;   /* DFF=1：必须写 16 位 */

    guard = 200000UL;
    while (((s_active_spi->SR & SPI_SR_RXNE) == 0U) && (guard != 0U)) { guard--; }
    if (guard == 0U) { (void)*(volatile uint16_t *)&s_active_spi->DR; return 0xFFFFU; }  /* 收不到 */
    rx = *(volatile uint16_t *)&s_active_spi->DR;

    return rx;
}

/*----------------------------------------------------------------------------
 * 舵机软件 PWM（TIM2 时基 + GPIO 直接翻转）
 * 这样任何 GPIO 都能输出 50Hz 舵机脉冲，不依赖引脚的定时器复用功能
 *--------------------------------------------------------------------------*/
#define SWPWM_TIM        TIM2
#define SWPWM_APB1EN     RCC_APB1ENR_TIM2EN
#define SWPWM_PERIOD_US  20000U      /* 20ms = 50Hz */
#define SWPWM_PULSE_MIN  1000U
#define SWPWM_PULSE_MAX  2000U

static volatile uint16_t s_sw_pulse_us = 1500U;
static volatile uint8_t  s_sw_enabled  = 0U;

void TIM2_IRQHandler(void)
{
    /* 周期起点：引脚拉高，并设定本次脉宽 */
    if ((SWPWM_TIM->SR & TIM_SR_UIF) != 0U) {
        SWPWM_TIM->SR = ~TIM_SR_UIF;
        if (s_sw_enabled != 0U) {
            SERVO_GPIO->BSRR = (uint32_t)(1UL << SERVO_PIN);
            SWPWM_TIM->CCR1  = (uint32_t)s_sw_pulse_us;
        }
    }
    /* 脉宽到时：引脚拉低 */
    if ((SWPWM_TIM->SR & TIM_SR_CC1IF) != 0U) {
        SWPWM_TIM->SR = ~TIM_SR_CC1IF;
        SERVO_GPIO->BSRR = (uint32_t)(1UL << (SERVO_PIN + 16U));
    }
}

void board_servo_pwm_init(uint16_t initial_pulse_us)
{
    uint32_t timclk;
    uint32_t psc;

    if (initial_pulse_us < SWPWM_PULSE_MIN) { initial_pulse_us = SWPWM_PULSE_MIN; }
    if (initial_pulse_us > SWPWM_PULSE_MAX) { initial_pulse_us = SWPWM_PULSE_MAX; }
    s_sw_pulse_us = initial_pulse_us;
    s_sw_enabled  = 0U;

    /* --- GPIO：推挽输出，先拉低 --- */
    RCC->AHB1ENR |= SERVO_GPIO_AHB1EN;
    (void)RCC->AHB1ENR;
    SERVO_GPIO->MODER   = (SERVO_GPIO->MODER & ~(3UL << (SERVO_PIN * 2U)))
                        | (1UL << (SERVO_PIN * 2U));          /* 输出 */
    SERVO_GPIO->OTYPER &= ~(1UL << SERVO_PIN);                 /* 推挽 */
    SERVO_GPIO->OSPEEDR = (SERVO_GPIO->OSPEEDR & ~(3UL << (SERVO_PIN * 2U)))
                        | (2UL << (SERVO_PIN * 2U));          /* 高速 */
    SERVO_GPIO->BSRR    = (uint32_t)(1UL << (SERVO_PIN + 16U));/* 拉低 */

    /* --- TIM2：1MHz 计数 --- */
    RCC->APB1ENR |= SWPWM_APB1EN;
    (void)RCC->APB1ENR;

    timclk = board_timer_clk_hz(SWPWM_TIM);
    psc = (timclk + 500000UL) / 1000000UL;      /* 让计数时钟 ≈ 1MHz */
    if (psc == 0U) { psc = 1U; }

    SWPWM_TIM->CR1   = 0U;
    SWPWM_TIM->PSC   = (uint16_t)(psc - 1U);
    SWPWM_TIM->ARR   = SWPWM_PERIOD_US - 1U;
    SWPWM_TIM->CCR1  = initial_pulse_us;
    SWPWM_TIM->CCMR1 = 0U;                       /* 纯比较，不接引脚 */
    SWPWM_TIM->EGR   = TIM_EGR_UG;
    SWPWM_TIM->SR    = 0U;
    SWPWM_TIM->DIER  = TIM_DIER_UIE | TIM_DIER_CC1IE;
    SWPWM_TIM->CR1   = TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 1U);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void board_servo_pwm_set_pulse(uint16_t pulse_us)
{
    if (pulse_us < SWPWM_PULSE_MIN) { pulse_us = SWPWM_PULSE_MIN; }
    if (pulse_us > SWPWM_PULSE_MAX) { pulse_us = SWPWM_PULSE_MAX; }
    s_sw_pulse_us = pulse_us;
}

void board_servo_pwm_enable(uint8_t enable)
{
    if (enable != 0U) {
        s_sw_enabled = 1U;
    } else {
        s_sw_enabled = 0U;
        SERVO_GPIO->BSRR = (uint32_t)(1UL << (SERVO_PIN + 16U));
    }
}

/*----------------------------------------------------------------------------
 * 第二条总线：SPI3  PC10=SCK PC11=MISO PC12=MOSI (AF6, APB1)
 * 供上长边(FPC11)与左短边(FPC10)使用，MISO 走 H9
 *--------------------------------------------------------------------------*/
void board_spi5_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    board_gpio_af_config(GPIOC, 10U, 6U, 0U);   /* SCK  PC10 AF6 */
    board_gpio_af_config(GPIOC, 11U, 6U, 0U);   /* MISO PC11 AF6 */
    board_gpio_af_config(GPIOC, 12U, 6U, 0U);   /* MOSI PC12 AF6 */

    SPI3->CR1 = 0U;
    SPI3->CR2 = 0U;
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_DFF
              | ((uint32_t)ADC64_SPI_BR << 3) | SPI_CR1_SPE;
}
