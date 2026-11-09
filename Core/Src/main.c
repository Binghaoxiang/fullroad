/**
  ******************************************************************************
  * @file    main.c
  * @brief   主程序：初始化 + 串口命令轮询
  *
  *  数据流：
  *    串口命令  ->  cmd.c 解析  ->  servo.c 设定目标角度
  *    SysTick   ->  servo.c 1ms 速度规划  ->  TIM4->CCR1  ->  舵机信号线
  ******************************************************************************
  */
#include "config.h"
#include "board.h"
#include "uart.h"
#include "servo.h"
#include "cmd.h"
#include "sensor.h"
#include "track.h"

static void print_banner(void)
{
    uart_puts("\r\n");
    uart_puts("=====================================================\r\n");
    uart_puts("  STM32F429IGT6  UART SERVO CONTROLLER  v" FW_VERSION "\r\n");
    uart_puts("-----------------------------------------------------\r\n");
    uart_puts("  SYSCLK=");
    uart_put_u32(SystemCoreClock / 1000000UL);
    uart_puts("MHz  SRC=");
    uart_puts((board_is_hse_clock() != 0U) ? "HSE+PLL" : "HSI+PLL");
    uart_puts("\r\n");
    uart_puts("  PWM : PD12/TIM4_CH1  50Hz  0.5us/step\r\n");
    uart_puts("  UART8: PE1/PE0 -> CH340C (COM) 115200  ");
    uart_put_u32(BOARD_UART_BAUD);
    uart_puts(" 8N1\r\n");
    uart_puts("  START=");
    uart_put_mdeg(SERVO_START_ANGLE_MDEG);
    uart_puts("deg  LIM=");
    uart_put_mdeg(SERVO_ANGLE_MIN_MDEG_DEF);
    uart_puts("..");
    uart_put_mdeg(SERVO_ANGLE_MAX_MDEG_DEF);
    uart_puts("deg\r\n");
    uart_puts("  H = help   Q = status\r\n");
    uart_puts("=====================================================\r\n");
}

int main(void)
{
    /* 1. 时钟：HSE 8MHz -> 180MHz（失败自动退回 HSI 方案） */
    board_clock_init();

    /* 2. 1kHz 时基，舵机速度规划与延时都靠它 */
    board_tick_init();

    /* 3. 调试串口 */
    uart_init(BOARD_UART_BAUD);

    /* 4. 舵机 PWM：先把 CCR 预置到中点，避免上电出现畸形脉宽 */
    board_servo_pwm_init((uint16_t)((SERVO_PULSE_MIN_US_DEF + SERVO_PULSE_MAX_US_DEF) / 2U));

    /* 5. 舵机控制层：此时脉冲还没输出，位置已预置为起始角度 */
    servo_init();

    cmd_init();

    /* 6. 探测器采集 + 跟踪状态机 */
    sensor_init();                 /* ADC 通道与引脚见 config.h 第 6 节 */
    track_init();

    print_banner();

    /* 7. 等舵机电源稳定后再打开脉冲；舵机会以限定的速度走到起始角度 */
    board_delay_ms(SERVO_START_DELAY_MS);
    servo_set_enabled(1U);


    /* 9. 上电自动对准：等光 -> 爬山找最垂直角度 -> 持续监控 */
    /* autoalign_run();   串口调试模式：不自动跟随，响应 UI 命令 */

    /* 8. 主循环：命令解析 + 20ms 跟踪节拍 + 遥测
     *    跟踪离不开 ADC 采样（一次约 1ms），放在主循环而不是中断里，
     *    避免把 1kHz 的 SysTick 拖长。 */
    {
        uint32_t last_tick = board_millis();

        for (;;) {
            cmd_poll();

            if ((uint32_t)(board_millis() - last_tick) >= 20U) {
                last_tick += 20U;
                track_tick_20ms();
            }
            cmd_telem_poll();
        }
    }
}
