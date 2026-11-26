// select.h
#ifndef SELECT_H
#define SELECT_H

#include <stdint.h>
#include "candidate.h"
#include "config.h"

typedef struct {
    float A, B, C;      // 归一化直线 A*x + B*y + C = 0
    float x1, y1;       // 内外探测点端点（供UI原样显示固件选中连线）
    float x2, y2;
    float w;            // 质量权重（外层 q × 内层 q × d 软权重）
    float d;            // 内层 - 外层（mm）
    float d_rel;        // d - bias
    float d_used;       // 联合内外状态约束后实际用于连线的视差(mm)
    float axis;         // 内外候选轴向平均位置（top/bottom=x, left/right=y）
    uint8_t side;       // 0=top,1=bottom,2=left,3=right
    uint8_t o_idx;      // 外层候选索引
    uint8_t i_idx;      // 内层候选索引
} SightLine;

typedef struct {
    float x, y;
    float x_axis, y_axis;    /* 协议兼容保留，不作为位置输出 */
    float x_line, y_line;   /* 稳定的交叉点输出（调试帧用） */
    float score;
    float residual;
    float confidence;
    uint8_t line_valid;     /* x_line/y_line 是否有效 */
    uint8_t cross_lines;    /* 参与交叉的有效线数 */
    uint8_t cross_source;   /* 0=无 1=交叉 2=交叉点保持 */
    float cross_conf;
    float cross_diff;
    uint8_t support_mask;   // bit0=top ... bit3=right
    uint8_t n_support;
    uint8_t hq_support;
    uint8_t line_idx[4];    // 每条边选中的线（0xFF=无效）
    uint8_t sel_idx[8];     // 每条边选中的候选（0xFF=无效）
    uint8_t mode;           // 0=invalid,2=degraded,3=product,4=high
} SelectionResult;

// 单帧内外配对 + 多边交叉投票；只输出交叉点，不做轴向融合兜底。
void select_run(const EdgeCandidates cand[8], const float bias[4],
                SelectionResult *out);

// 清空候选/速度/确认历史；无光或重新定位时禁止旧轨迹抢占新候选。
void select_reset(void);

// 读取本帧固件实际选中的视线端点；UI只绘制，不重新求交。
uint8_t select_get_line(uint8_t side, uint8_t line_idx,
                        float *x1, float *y1, float *x2, float *y2);

#endif
