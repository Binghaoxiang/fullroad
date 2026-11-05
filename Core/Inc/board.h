/**
  ******************************************************************************
  * @file    board.h
  * @brief   板级硬件抽象：时钟树、SysTick 时基、GPIO 复用、舵�?PWM
  ******************************************************************************
  */
#ifndef __BOARD_H
#define __BOARD_H

#include "config.h"
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 时钟 ---- */
void     board_clock_init(void);        /* 必须�?main 最前面调用 */
uint8_t  board_is_hse_clock(void);      /* 1: HSE+PLL(168MHz)  0: HSI+PLL(168MHz) */
uint32_t board_pclk1_hz(void);
uint32_t board_pclk2_hz(void);
uint32_t board_timer_clk_hz(TIM_TypeDef *TIMx);

/* ---- 时基与延�?---- */
void     board_tick_init(void);         /* SysTick 1kHz */
void     board_tick_1ms(void);          /* �?SysTick 中断调用 */
uint32_t board_millis(void);
void     board_delay_ms(uint32_t ms);   /* 基于 SysTick，可被中断打断，精度 1ms */

/* ---- GPIO ---- */
/* pull: 0 = 浮空, 1 = 上拉, 2 = 下拉 */
void     board_gpio_af_config(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t af, uint32_t pull);

/* ---- 舵机 PWM（TIM4_CH1�?0Hz�?.5us 分辨率） ---- */
void     board_servo_pwm_init(uint16_t initial_pulse_us);
void     board_servo_pwm_set_pulse(uint16_t pulse_us);
void     board_servo_pwm_enable(uint8_t enable);

/* ---- 探测�?ADC（ADC1，通道�?config.h �?TRK_ADC_CHANNELS 决定�?---- */
void     board_adc_init(void);
uint16_t board_adc_read(uint8_t ch);    /* 单次转换，返�?12bit 原始�?*/

/* ---- SPI4（两�?ADC64 板共用，SDO 并联，靠各自�?CS 分时选通） ---- */
void     board_spi_init(void);
uint16_t board_spi_transfer16(uint16_t tx);
void     board_spi_set_mode(uint8_t mode);   /* 0..3 = CPOL/CPHA */

/* ---- 微秒级延时（DWT 周期计数器，精度 1 个主频周期） ---- */
void     board_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
