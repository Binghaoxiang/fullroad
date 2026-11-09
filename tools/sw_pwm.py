#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sw_pwm.py — 把舵机 PWM 改成软件实现（任意 GPIO 可用）

原因：PD0 没有 TIM4_CH3 复用功能（STM32F429 的 TIM4_CH3 只在 PB8/PD14 上）。
      改用 TIM2 做 1MHz 时基 + 两个中断点，在任意 GPIO 上产生 50Hz 舵机脉冲。

原理：
  - TIM2: PSC 使计数时钟 = 1MHz，ARR = 19999 → 20ms 周期（50Hz）
  - 更新中断（每 20ms）：引脚拉高，CCR1 = 脉宽
  - 比较中断（脉宽到达）：引脚拉低
  - 抖动 < 5µs，对舵机完全够用
"""

import io
import re

F = "Core/Src/board.c"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 找到三个舵机函数的完整定义并替换 ----------
def replace_func(src, name, new_body):
    """把 void name(...) { ... } 整体替换"""
    m = re.search(r"void\s+" + re.escape(name) + r"\s*\([^)]*\)\s*\{", src)
    if not m:
        return src, False
    start = m.start()
    i = src.index("{", m.start())
    depth = 0
    while i < len(src):
        if src[i] == "{":
            depth += 1
        elif src[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    return src[:start] + new_body.strip() + src[end:], True


sw_block = r'''
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
'''

# 逐个替换
ok_all = True
for name in ("board_servo_pwm_init", "board_servo_pwm_set_pulse", "board_servo_pwm_enable"):
    s2, ok = replace_func(s, name, "/* %s: 见下方软件 PWM 实现 */" % name)
    if not ok:
        ok_all = False
    s = s2

# 在文件末尾追加完整实现
s = s.rstrip() + "\n" + sw_block + "\n"

io.open(F, "w", encoding="utf-8").write(s)
print("replaced:", ok_all)
