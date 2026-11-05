/**
  ******************************************************************************
  * @file    track.h
  * @brief   自动对准跟踪状态机（单边、单轴）
  *
  *  两种控制模式：
  *    A（差分锁零）：通道 >=2 时可用，S 过零即垂直入射
  *    B（强度爬坡）：只有 1 路时用，扰动观察法爬 P 峰值
  ******************************************************************************
  */
#ifndef __TRACK_H
#define __TRACK_H

#include "config.h"
#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TRK_ST_OFF = 0,   /* 未使能 */
    TRK_ST_SEARCH,    /* 无光：单向扫描找光 */
    TRK_ST_SEEK,      /* 有光：按 S 或 P 调整角度 */
    TRK_ST_LOCK,      /* 已锁定 */
    TRK_ST_LOST       /* 光丢失：保持角度等待 */
} trk_state_t;

void        track_init(void);
void        track_reset(void);

void        track_enable(uint8_t on);
uint8_t     track_enabled(void);

void        track_set_mode(uint8_t mode);      /* 1 = A 差分, 2 = B 强度 */
uint8_t     track_mode(void);
void        track_set_sign(int8_t sign);       /* +1 / -1，方向不对时翻一下 */
int8_t      track_sign(void);

void        track_tick_20ms(void);             /* 每 20ms 调用一次 */

trk_state_t track_state(void);
const char *track_state_name(void);
const sensor_meas_t *track_last(void);
uint32_t    track_lock_ms(void);

#ifdef __cplusplus
}
#endif

void    track_set_pid(int32_t kp, int32_t ki, int32_t kd);
int32_t track_get_kp(void);
int32_t track_get_ki(void);
int32_t track_get_kd(void);

#endif /* __TRACK_H */
