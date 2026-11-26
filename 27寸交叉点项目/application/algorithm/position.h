// position.h
#ifndef POSITION_H
#define POSITION_H

#include <stdint.h>
#include "centroid.h"

typedef struct {
    float x;
    float y;
    uint8_t valid;
} Position;

// 交叉点定位的中间结果，供UI原样显示与算法调试。
typedef struct {
    float x_line;      // 原始多线 WLS 交点 x
    float y_line;      // 原始多线 WLS 交点 y
    uint8_t have_x;    // 交叉点是否可用
    uint8_t have_y;    // 交叉点是否可用
    float x_cross_out; // 统一下来的交叉输出 x
    float y_cross_out; // 统一下来的交叉输出 y
    uint8_t cross_out_valid;
    uint8_t cross_source;   // 0无 1当前 2保持 3降级 4四点 5三点稳健 6短基线限幅
    uint8_t cross_lines;    // 当前参与交叉的有效线数
    float cross_conf;
    float cross_diff;       // source=5时为剔除点编号(0外左/1内左/2内右/3外右)，否则-1
    float line_x1[4];       // top,bottom,left,right实际求交直线显示端点
    float line_y1[4];
    float line_x2[4];
    float line_y2[4];
    uint8_t line_valid_mask;
} PositionDebug;

void position_compute(Centroids *c, Position *p, float dt);
const PositionDebug *position_get_debug(void);

#endif
