// kalman.h
#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include "position.h"

/* 名字保留 kalman_* 以免改调用点；内部实现是 One Euro 自适应低通，
 * 不再维护协方差矩阵（旧实现的 p_vv 只增不减，帧率一变就失配）。 */
typedef struct {
    float x, y;          /* 滤波后位置 */
    float dx, dy;        /* 低通后的速度(mm/s) */
    float last_zx, last_zy; /* 上一帧原始观测 */
    float dt;
    float pending_x, pending_y; /* 待确认的大跳变观测 */
    float missing_s;           /* 距离最后一次接受观测的时间 */
    uint16_t lost_cnt;      /* 诊断计数；交点丢失时不再外推坐标 */
    uint8_t init;
    uint8_t pending;
    uint8_t held;           /* 本帧使用最后一个有效输出，不进行外推 */
} Kalman;

void kalman_init(Kalman *k, float dt);
void kalman_update(Kalman *k, Position *z, Position *out);

#endif
