/**
  ******************************************************************************
  * @file    servo.h
  * @brief   舵机控制层：角度 <-> 脉宽映射、斜率限制、使能控制
  *
  *          角度内部统一使用「毫度」(0.001°) 的 int32 整数，
  *          全程无浮点、无累计误差，这样才谈得上"精准"。
  ******************************************************************************
  */
#ifndef __SERVO_H
#define __SERVO_H

#include "config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 舵机标定参数 */
typedef struct
{
    uint16_t pulse_min_us;      /* 角度下限对应的脉宽 */
    uint16_t pulse_max_us;      /* 角度上限对应的脉宽 */
    int32_t  angle_min_mdeg;    /* 角度下限（毫度） */
    int32_t  angle_max_mdeg;    /* 角度上限（毫度） */
    uint32_t speed_mdeg_s;      /* 最大角速度（毫度/秒），0 = 不限速 */
    int32_t  trim_mdeg;         /* 零点修正（毫度），只影响输出脉宽，不改变目标 */
} servo_cfg_t;

/** 运行状态快照 */
typedef struct
{
    int32_t  pos_mdeg;          /* 当前（斜坡后）角度 */
    int32_t  target_mdeg;       /* 目标角度 */
    uint16_t pulse_us;          /* 当前实际输出脉宽 */
    uint8_t  enabled;           /* 1 = PWM 脉冲已输出 */
    uint8_t  moving;            /* 1 = 还在走向目标 */
} servo_state_t;

/* ---- 生命周期 ---- */
void servo_init(void);          /* 必须在 board_servo_pwm_init() 之后调用 */
void servo_defaults(void);      /* 恢复 config.h 里的默认参数 */
void servo_tick_1ms(void);      /* 由 SysTick 中断每 1ms 调用一次 */

/* ---- 控制 ---- */
void servo_set_angle(int32_t mdeg);     /* 设定目标角度（自动限幅） */
void servo_set_pulse(uint16_t us);      /* 直接给脉宽，用于示波器/标定验证 */
void servo_set_enabled(uint8_t enable);
/* 360 度连续旋转模式：permille -1000..+1000，ms=0 表示一直转 */
void     servo_spin(int32_t permille, uint32_t ms);
void     servo_spin_neutral(uint16_t us);
uint16_t servo_spin_neutral_get(void);
int32_t  servo_spin_permille(void);
uint8_t  servo_is_spin_mode(void); /* 使能/关闭脉冲输出 */
void servo_apply_now(void);             /* 立即按当前角度重算并刷新脉宽 */

/* ---- 查询 ---- */
servo_state_t servo_get_state(void);
servo_cfg_t   servo_get_cfg(void);
int32_t       servo_angle_min(void);
int32_t       servo_angle_max(void);
uint32_t      servo_get_speed(void);
int32_t       servo_get_trim(void);
uint16_t      servo_angle_to_pulse(int32_t cmd_mdeg);   /* 含 trim 与限幅 */
int32_t       servo_pulse_to_angle(uint16_t us);        /* 逆映射 */

/* ---- 标定 ---- */
uint8_t servo_set_pulse_range(uint16_t min_us, uint16_t max_us);   /* 1 = 成功 */
uint8_t servo_set_angle_range(int32_t min_mdeg, int32_t max_mdeg);
void    servo_set_speed(uint32_t mdeg_s);                          /* 0 = 不限速 */
void    servo_set_trim(int32_t mdeg);

/* ---- 分档位点动（多级指令转动） ----
 * 档位 1..SERVO_JOG_LEVELS，对应 SERVO_JOG_STEP_TABLE 里的步长。
 * servo_jog(+1) 按当前档位正向走一步，返回新的目标角度（已限幅）。
 */
uint8_t servo_get_jog_level(void);
int32_t servo_get_jog_step(void);
uint8_t servo_set_jog_level(uint8_t level);        /* 1 = 成功 */
int32_t servo_jog(int32_t steps);

/* ---- 等待运动结束（阻塞，带超时） ---- */
void servo_wait_done(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
