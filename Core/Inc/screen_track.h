#ifndef SCREEN_TRACK_H
#define SCREEN_TRACK_H

#include <stdint.h>

void     st_init(void);
void     st_dump_edges(void);      /* 采集 + 处理 + 打印曲线 + 坐标 */
float    st_x_mm(void);
float    st_y_mm(void);
uint8_t  st_ok(uint8_t e);
float    st_peak(uint8_t e);
uint8_t  st_n(uint8_t e);

#endif