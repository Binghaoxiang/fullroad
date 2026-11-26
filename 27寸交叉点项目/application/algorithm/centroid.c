// centroid.c
#include "centroid.h"
#include "config.h"
#include "runtime_params.h"
#include "bsp_uart.h"
#include <math.h>

#define ADC_MAX  4095.0f

#if CENTROID_ALGO_PEAK

/* 跟踪窗加权质心（v4：宽/窄自适应）
 *
 * up/in_up 的光斑剖面在一个位置可能是 14 个点宽的平台，右侧 in_right 只有 5 个点。
 * 固定窄高斯（W=4, sigma=1.2）在宽平台上会锁到平台左边或右边的局部峰，
 * 出现 22 ↔ 28 的两模式跳变；固定窗/宽窗能把平台平均掉。
 *
 * 现在：
 *   1) 峰附近 ±CENTROID_FLAT_SCAN 内 >= CENTROID_FLAT_RATIO*peak 的点数
 *      >= CENTROID_FLAT_MIN_CNT 时判为宽剖面，用 W=7 / sigma=2.5；
 *      否则保持 W=4 / sigma=1.2。
 *   2) 静止时窗口中心使用连续跟踪位置；全局主峰移动超过约1个索引时
 *      立即把窗口移到新峰，避免高斯窗拖着质心慢慢追赶激光。
 *   3) 计算结果再做快速连续跟踪，静止抖动交给后级中值/One Euro抑制。
 *   4) 全局峰仍只用于有效性判断和质量权重(peak)。
 */
static float   g_track_pos[8];
static uint8_t g_track_ok[8];

static void find_centroid(uint16_t *signal, uint16_t len, Centroid *c, uint8_t edge)
{
    float norm[256];
    uint16_t n = (len > 256) ? 256 : len;
    if (n == 0) {
        c->position = 0.0f; c->peak = 0.0f; c->valid = 0;
        g_track_ok[edge] = 0;
        return;
    }

    for (uint16_t i = 0; i < n; i++) {
        norm[i] = (float)signal[i] / ADC_MAX;
        if (norm[i] > 1.0f) norm[i] = 1.0f;
    }

    // 1) 全局峰值：只用于有效性判断/质量权重
    uint16_t gpk = 0;
    for (uint16_t i = 1; i < n; i++)
        if (norm[i] > norm[gpk]) gpk = i;
    float gpeak = norm[gpk];
    if (gpeak < runtime_params_get()->centroid_threshold) {
        c->position = 0.0f;
        c->peak = 0.0f;
        c->valid = 0;
        g_track_ok[edge] = 0;
        return;
    }

    // 2) 判宽剖面：峰附近有多少点仍接近峰值
    int fl = (int)gpk - CENTROID_FLAT_SCAN;
    int fh = (int)gpk + CENTROID_FLAT_SCAN;
    if (fl < 0) fl = 0;
    if (fh > (int)n - 1) fh = (int)n - 1;
    float flat_thr = gpeak * CENTROID_FLAT_RATIO;
    int flat_cnt = 0;
    for (int i = fl; i <= fh; i++)
        if (norm[i] >= flat_thr) flat_cnt++;

    uint8_t wide = (flat_cnt >= CENTROID_FLAT_MIN_CNT);
    int   W = wide ? CENTROID_WIDE_W : CENTROID_GAUSS_W;
    float sigma = wide ? CENTROID_WIDE_SIGMA : CENTROID_GAUSS_SIGMA;

    // 3) 窗口中心：静止连续，运动时立即重定到当前主峰。
    float center;
    if (g_track_ok[edge]) {
        center = g_track_pos[edge];
        if (fabsf((float)gpk - center) > CENTROID_MOVE_RECENTER)
            center = (float)gpk;
    } else {
        center = (float)gpk;
    }

    // 4) 窗内先减最小值，再加权
    int s = (int)floorf(center) - W;
    int e = (int)ceilf(center)  + W;
    if (s < 0) s = 0;
    if (e > (int)n - 1) e = (int)n - 1;

    float base = norm[s];
    for (int i = s + 1; i <= e; i++)
        if (norm[i] < base) base = norm[i];

    float sw = 0.0f, swi = 0.0f;
    const float inv2s2 = 1.0f / (2.0f * sigma * sigma);
    for (int i = s; i <= e; i++) {
        float w = norm[i] - base;
        if (w <= 0.0f) continue;
        float dx = (float)i - center;
        w *= expf(-dx * dx * inv2s2);
        sw += w;
        swi += (float)i * w;
    }
    if (sw <= 1e-9f) {
        c->position = 0.0f;
        c->peak = 0.0f;
        c->valid = 0;
        g_track_ok[edge] = 0;
        return;
    }

    float pos = swi / sw;
    c->position = pos;
    c->peak = gpeak;
    c->valid = 1;

    // 5) 连续跟踪：大移动直接跟上，小变化快速更新供下一帧定位窗口。
    if (!g_track_ok[edge] || fabsf(pos - g_track_pos[edge]) > CENTROID_TRACK_JUMP) {
        g_track_pos[edge] = pos;
    } else {
        g_track_pos[edge] += CENTROID_TRACK_EMA * (pos - g_track_pos[edge]);
    }
    g_track_ok[edge] = 1;
}

void centroids_find(uint16_t *raw[8], const uint16_t *edge_len, Centroids *c)
{
    Centroid *out[8] = {
        &c->up, &c->down, &c->left, &c->right,
        &c->in_up, &c->in_down, &c->in_left, &c->in_right
    };
    for (uint8_t e = 0; e < 8; e++)
        find_centroid(raw[e], edge_len[e], out[e], e);
}

#else

/* 旧算法：绝对阈值 + 最大能量区域 + 加权平均（保留可回退） */
static void find_centroid(uint16_t *signal, uint16_t len, Centroid *c)
{
    float norm[256];
    uint16_t n = (len > 256) ? 256 : len;

    for (uint16_t i = 0; i < n; i++) {
        norm[i] = (float)signal[i] / ADC_MAX;
        if (norm[i] > 1.0f) norm[i] = 1.0f;
    }

    int best_start = -1, best_end = -1;
    float best_sum = 0.0f;
    int in_region = 0;
    uint16_t region_start = 0;

    for (uint16_t i = 0; i < n; i++) {
        if (norm[i] > runtime_params_get()->centroid_threshold && !in_region) {
            in_region = 1;
            region_start = i;
        } else if (norm[i] <= runtime_params_get()->centroid_threshold && in_region) {
            in_region = 0;
            uint16_t width = i - region_start;
            if (width >= MIN_REGION_WIDTH) {
                float sum = 0.0f;
                for (uint16_t j = region_start; j < i; j++) sum += norm[j];
                if (sum > best_sum) {
                    best_sum = sum;
                    best_start = region_start;
                    best_end = i;
                }
            }
        }
    }
    if (in_region) {
        uint16_t width = n - region_start;
        if (width >= MIN_REGION_WIDTH) {
            float sum = 0.0f;
            for (uint16_t j = region_start; j < n; j++) sum += norm[j];
            if (sum > best_sum) {
                best_start = region_start;
                best_end = n;
            }
        }
    }

    if (best_start >= 0) {
        float sum_w = 0.0f, sum_s = 0.0f, peak = 0.0f;
        for (int i = best_start; i < best_end; i++) {
            sum_w += (float)i * norm[i];
            sum_s += norm[i];
            if (norm[i] > peak) peak = norm[i];
        }
        if (sum_s > 0.0f) {
            c->position = sum_w / sum_s;
            c->peak = peak;
            c->valid = 1;
            return;
        }
    }
    c->position = 0.0f;
    c->peak = 0.0f;
    c->valid = 0;
}

void centroids_find(uint16_t *raw[8], const uint16_t *edge_len, Centroids *c)
{
    find_centroid(raw[EDGE_UP],       edge_len[EDGE_UP],       &c->up);
    find_centroid(raw[EDGE_DOWN],     edge_len[EDGE_DOWN],     &c->down);
    find_centroid(raw[EDGE_LEFT],     edge_len[EDGE_LEFT],     &c->left);
    find_centroid(raw[EDGE_RIGHT],    edge_len[EDGE_RIGHT],    &c->right);
    find_centroid(raw[EDGE_IN_UP],    edge_len[EDGE_IN_UP],    &c->in_up);
    find_centroid(raw[EDGE_IN_DOWN],  edge_len[EDGE_IN_DOWN],  &c->in_down);
    find_centroid(raw[EDGE_IN_LEFT],  edge_len[EDGE_IN_LEFT],  &c->in_left);
    find_centroid(raw[EDGE_IN_RIGHT], edge_len[EDGE_IN_RIGHT], &c->in_right);
}

#endif
