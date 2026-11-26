#include "app_core.h"
#include "config.h"

#if MCU_IS_MASTER

#include "bsp_sensor.h"
#include "mirror.h"
#include "centroid.h"
#include "candidate.h"
#include "select.h"
#include "centroid_filter.h"
#include "position.h"
#include "kalman.h"
#include "can_proto.h"
#include "frame.h"
#include "bsp_uart.h"
#include "board_map.h"
#include "runtime_params.h"
#include <string.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include <stdio.h>

// 每条边实际数据槽数（按主控逐段板型表计算，支持混合 30/35cm 板）
static uint16_t  edge_len[EDGE_COUNT];
static Centroids centroids;
static Position  pos_raw, pos_filt;
static Kalman    kalman;
static EdgeCandidates product_candidates[EDGE_COUNT];
static SelectionResult product_selection;
static PositionDebug product_debug;
static float product_select_bias[4];
static uint8_t last_centroid_filter_enable;
static uint16_t  master_peak_idx = 0xFFFF;
static uint16_t  master_peak_val;
static uint8_t   master_pulse_aligned;
static uint32_t  cf_last_tick;
typedef struct {
    float coord;
    float previous[4];
    float reanchor_center;
    uint8_t previous_valid[4];
    uint8_t reanchor_count;
    uint8_t initialized;
} VerticalAxisTracker;
static VerticalAxisTracker vertical_axis_tracker[2];
#if WAVE_FILTER_ENABLE
typedef struct {
    float accepted;
    float pending;
    uint8_t accepted_valid;
    uint8_t pending_count;
} WaveTrack;

static WaveTrack wave_track[EDGE_COUNT];
static uint16_t wave_sort[MAX_SENSORS_PER_EDGE];
/* Dense history for amplitude-only temporal smoothing.  Samples outside the
   current confirmed single lobe are cleared immediately, so this filter
   cannot leave a moving/vanished optical trail behind. */
static uint16_t wave_temporal[FRAME_RAW_BYTES_MAX / sizeof(uint16_t)];

static void wave_sort_u16(uint16_t *v, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++) {
        uint16_t key = v[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }
}

/* Robustly reduce one complete detector edge to its most credible single
   spatial lobe.  Detection uses median/MAD; retained samples stay in the
   board's amplitude units so the UI remains an honest firmware waveform. */
static uint8_t wave_filter_edge(uint8_t edge, uint16_t *v, uint16_t n,
                                float *peak_pos)
{
    uint16_t median, mad, core_gate, wing_gate;
    uint16_t best_lo = 0, best_hi = 0, best_pk = 0;
    uint32_t best_energy = 0;
    uint8_t found = 0;

    if (n < 5u || n > MAX_SENSORS_PER_EDGE) {
        memset(v, 0, n * sizeof(*v));
        return 0;
    }
    memcpy(wave_sort, v, n * sizeof(*v));
    wave_sort_u16(wave_sort, n);
    median = wave_sort[n / 2u];
    for (uint16_t i = 0; i < n; i++)
        wave_sort[i] = (v[i] >= median) ? (uint16_t)(v[i] - median)
                                        : (uint16_t)(median - v[i]);
    wave_sort_u16(wave_sort, n);
    mad = wave_sort[n / 2u];

    {
        uint32_t d = (uint32_t)WAVE_NOISE_MAD_K * (uint32_t)(mad + 1u);
        if (d < WAVE_NOISE_ABS) d = WAVE_NOISE_ABS;
        if (d > 65535u - median) d = 65535u - median;
        core_gate = (uint16_t)(median + d);
        d = (uint32_t)WAVE_WING_MAD_K * (uint32_t)(mad + 1u);
        if (d < WAVE_WING_ABS) d = WAVE_WING_ABS;
        if (d > 65535u - median) d = 65535u - median;
        wing_gate = (uint16_t)(median + d);
    }

    for (uint16_t i = 0; i < n;) {
        if (v[i] < core_gate) { i++; continue; }
        uint16_t lo = i, hi = i, pk = i;
        uint32_t energy = 0;
        while (hi + 1u < n && v[hi + 1u] >= core_gate) hi++;
        for (uint16_t k = lo; k <= hi; k++) {
            energy += (uint32_t)(v[k] - median);
            if (v[k] > v[pk]) pk = k;
        }
        uint16_t core_width = (uint16_t)(hi - lo + 1u);
        uint32_t single_gate = (uint32_t)WAVE_SINGLE_CORE_MAD_K *
                               (uint32_t)(mad + 1u);
        if (single_gate < WAVE_SINGLE_CORE_ABS)
            single_gate = WAVE_SINGLE_CORE_ABS;
        uint8_t single_has_wing = 0u;
        if (core_width == 1u) {
            if (pk > 0u && v[pk - 1u] >= wing_gate)
                single_has_wing = 1u;
            if (pk + 1u < n && v[pk + 1u] >= wing_gate)
                single_has_wing = 1u;
        }
        if ((core_width >= WAVE_MIN_CORE_WIDTH ||
             (core_width == 1u && single_has_wing &&
              (uint32_t)(v[pk] - median) >= single_gate)) &&
            pk > WAVE_EDGE_GUARD_SENSORS &&
            pk + WAVE_EDGE_GUARD_SENSORS + 1u < n &&
            (!found || energy > best_energy)) {
            best_lo = lo; best_hi = hi; best_pk = pk;
            best_energy = energy; found = 1;
        }
        i = (uint16_t)(hi + 1u);
    }
    if (!found) {
        memset(v, 0, n * sizeof(*v));
        wave_track[edge].accepted_valid = 0;
        wave_track[edge].pending_count = 0;
        return 0;
    }

    while (best_lo > 0u && v[best_lo - 1u] >= wing_gate &&
           (uint16_t)(best_hi - best_lo + 1u) < WAVE_MAX_SUPPORT_WIDTH) best_lo--;
    while (best_hi + 1u < n && v[best_hi + 1u] >= wing_gate &&
           (uint16_t)(best_hi - best_lo + 1u) < WAVE_MAX_SUPPORT_WIDTH) best_hi++;

    {
        float sw = 0.0f, sx = 0.0f;
        for (uint16_t i = best_lo; i <= best_hi; i++) {
            float w = (v[i] > median) ? (float)(v[i] - median) : 0.0f;
            sw += w; sx += w * (float)i;
        }
        *peak_pos = (sw > 0.0f) ? sx / sw : (float)best_pk;
    }

    WaveTrack *t = &wave_track[edge];
    if (t->accepted_valid &&
        fabsf(*peak_pos - t->accepted) <= WAVE_TRACK_JUMP_SENSORS) {
        t->accepted = *peak_pos;
        t->pending_count = 0;
    } else {
        if (t->pending_count &&
            fabsf(*peak_pos - t->pending) <= WAVE_TRACK_JUMP_SENSORS) {
            if (t->pending_count < 255u) t->pending_count++;
        } else {
            t->pending = *peak_pos;
            t->pending_count = 1u;
        }
        /* 已跟踪的强单峰远移只需相邻两帧确认；首次出光仍需3帧。
           避免快速笔迹每跨60mm都被额外清空两帧。 */
        uint8_t required = t->accepted_valid ? 2u : WAVE_CONFIRM_FRAMES;
        if (t->pending_count < required) {
            memset(v, 0, n * sizeof(*v));
            return 0;
        }
        t->accepted = *peak_pos;
        t->accepted_valid = 1u;
        t->pending_count = 0;
    }

    for (uint16_t i = 0; i < n; i++) {
        if (i < best_lo || i > best_hi || v[i] < wing_gate) v[i] = 0;
        else v[i] = (v[i] > median) ? (uint16_t)(v[i] - median) : 0u;
    }
    return 1;
}

static void wave_filter_all(uint16_t *raw[EDGE_COUNT], const uint16_t *len)
{
    uint8_t ok[EDGE_COUNT] = {0};
    float delta[EDGE_COUNT] = {0};
    for (uint8_t e = 0; e < EDGE_COUNT; e++) {
        float pos;
        float previous = wave_track[e].accepted;
        uint8_t tracked = wave_track[e].accepted_valid;
        /* Validate and retain every edge independently.  Missing CAN strips or
           a cross clipped near the membrane boundary must not erase healthy
           waveforms on the other strips; 2-D geometry enforces its own
           per-axis support requirements later. */
        ok[e] = wave_filter_edge(e, raw[e], len[e], &pos);
        if (ok[e] && tracked) delta[e] = pos - previous;
    }

    /* 一轴至少两边同向移动才切换快速幅值响应；单边噪声不触发。
       四条边使用同一增益，避免幅值历史滞后不同制造内外视差。 */
    uint8_t moving[2] = {0};
    for (uint8_t axis = 0; axis < 2; axis++) {
        uint8_t positive = 0, negative = 0;
        for (uint8_t e = 0; e < EDGE_COUNT; e++) {
            if ((e % 4u < 2u ? 0u : 1u) != axis || !ok[e]) continue;
            if (delta[e] > 0.15f) positive++;
            if (delta[e] < -0.15f) negative++;
        }
        moving[axis] = (positive >= 2u || negative >= 2u);
    }

    {
        uint16_t dense = 0u;
        for (uint8_t e = 0; e < EDGE_COUNT; e++) {
            for (uint16_t i = 0; i < len[e]; i++, dense++) {
                uint16_t cur = ok[e] ? raw[e][i] : 0u;
                uint16_t old = wave_temporal[dense];
                if (cur == 0u) {
                    /* Immediate clear prevents an EMA tail or a second lobe. */
                    wave_temporal[dense] = 0u;
                    raw[e][i] = 0u;
                } else if (old == 0u) {
                    /* Acquire a newly illuminated diode without EMA latency. */
                    wave_temporal[dense] = cur;
                    raw[e][i] = cur;
                } else {
                    uint32_t alpha = moving[e % 4u < 2u ? 0u : 1u] ?
                                     240u : WAVE_TEMPORAL_ALPHA_Q8;
                    uint32_t mixed = (256u - alpha) * old + alpha * cur + 128u;
                    wave_temporal[dense] = (uint16_t)(mixed >> 8);
                    raw[e][i] = wave_temporal[dense];
                }
            }
        }
    }
}
#endif
#if DEBUG_UI_STREAM || SEND_FULL_FRAME || (FULL_FRAME_EVERY_N > 0) || SEND_CAND_DEBUG_FRAME
static uint16_t  tx_frame_idx;
#endif
#if SEND_CAND_DEBUG_FRAME
static EdgeCandidates cand_debug[EDGE_COUNT];
static SelectionResult sel_debug;
static float select_bias[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static Centroids cand_centroids;
static Position cand_pos_raw, cand_pos_filt, cand_stable;
static Kalman cand_kalman;
static uint8_t cand_stable_valid;
static uint8_t cand_lost;
static float cand_pick_pos[EDGE_COUNT];
static float cand_pick_peak[EDGE_COUNT];
static uint8_t cand_pick_valid[EDGE_COUNT];
static Position cand_gate_last;
static uint8_t cand_gate_valid;
static Position cand_pending;
static uint8_t cand_pending_valid;
static uint8_t cand_pending_count;

#define CAND_OUT_JUMP_GATE_MM   60.0f
#define CAND_OUT_CONFIRM_FRAMES 2

/* 每条边独立选一个候选：优先用投票结果，投票无效时回退到
   上一帧附近/最强峰。这样缺边/少边时，单边候选也不会被丢掉。 */
static uint8_t pick_edge_candidate(uint8_t edge, const EdgeCandidates *ec,
                                   uint8_t preferred)
{
    uint8_t best = 0, k;
    float best_peak = -1.0f;

    if (ec->n == 0) {
        cand_pick_valid[edge] = 0;
        return 0xFF;
    }
    if (preferred != 0xFF && preferred < ec->n) {
        best = preferred;
    } else if (cand_pick_valid[edge]) {
        uint8_t near = 0xFF, max_i = 0;
        float near_peak = -1.0f, max_peak = -1.0f;
        for (k = 0; k < ec->n && k < MAX_CAND; k++) {
            float p = ec->c[k].peak;
            if (p > max_peak) { max_peak = p; max_i = k; }
            if (fabsf(ec->c[k].pos - cand_pick_pos[edge]) <= SELECT_CAND_TRACK &&
                p > near_peak) {
                near_peak = p;
                near = k;
            }
        }
        if (near != 0xFF && near_peak * SELECT_CAND_UPGRADE >= max_peak)
            best = near;
        else
            best = max_i;
    } else {
        for (k = 0; k < ec->n && k < MAX_CAND; k++) {
            if (ec->c[k].peak > best_peak) {
                best_peak = ec->c[k].peak;
                best = k;
            }
        }
    }

    cand_pick_pos[edge] = ec->c[best].pos;
    cand_pick_peak[edge] = ec->c[best].peak;
    cand_pick_valid[edge] = 1;
    return best;
}
#endif
extern UART_HandleTypeDef huart2;

static void sort_float_small(float *v, uint8_t n)
{
    for (uint8_t i = 1u; i < n; i++) {
        float key = v[i];
        uint8_t j = i;
        /* Keep the insertion index in [0, i].  Casting -1 to uint8_t before
           adding one produced index 256 and a HardFault on descending data. */
        while (j > 0u && v[j - 1u] > key) {
            v[j] = v[j - 1u];
            j--;
        }
        v[j] = key;
    }
}

/* In vertical-incidence mode the four strips measure one physical axis
   coordinate.  Their disagreement is calibration/noise residual, not slope. */
static uint8_t vertical_axis_fuse(const Centroids *c, uint8_t group,
                                  float *out_coord, float *out_rms,
                                  float *out_spread, uint8_t *out_points)
{
    const Centroid *all[8] = {
        &c->up, &c->down, &c->left, &c->right,
        &c->in_up, &c->in_down, &c->in_left, &c->in_right
    };
    const uint8_t edge_group[2][4] = {
        { EDGE_UP, EDGE_DOWN, EDGE_IN_UP, EDGE_IN_DOWN },
        { EDGE_LEFT, EDGE_RIGHT, EDGE_IN_LEFT, EDGE_IN_RIGHT }
    };
    float value[4], ordered[4], base_w[4], robust_w[4];
    uint8_t value_slot[4];
    uint8_t keep[4] = {0u, 0u, 0u, 0u};
    uint8_t n = 0u, kept = 0u, iteration, k;
    float center, rms, vmin = 0.0f, vmax = 0.0f;

    for (k = 0u; k < 4u; k++) {
        uint8_t edge = edge_group[group][k];
        const Centroid *ce = all[edge];
        float along;
        if (!ce->valid || !isfinite(ce->position) || ce->peak <= 0.0f) continue;
        along = runtime_params_map_edge(edge, ce->position);
        if (!isfinite(along)) continue;
        value[n] = ordered[n] = along;
        value_slot[n] = k;
        /* Strong strips count more, but sqrt keeps one strip from dominating. */
        base_w[n] = 0.20f + sqrtf(ce->peak);
        n++;
    }
    if (n < VERTICAL_FUSE_MIN_POINTS) {
        memset(&vertical_axis_tracker[group], 0,
               sizeof(vertical_axis_tracker[group]));
        return 0u;
    }

    sort_float_small(ordered, n);
    center = (n & 1u) ? ordered[n / 2u]
                      : 0.5f * (ordered[n / 2u - 1u] + ordered[n / 2u]);
    for (k = 0u; k < n; k++) {
        /* With only two strips both measurements are needed to form the
           common coordinate.  With 3/4 strips, discard only a gross isolated
           outlier; the 1 mm target remains visible as a quality diagnostic. */
        if (n == 2u || fabsf(value[k] - center) <= VERTICAL_FUSE_OUTLIER_MM) {
            keep[k] = 1u;
            robust_w[k] = base_w[k];
            kept++;
        }
    }
    if (kept < VERTICAL_FUSE_MIN_POINTS) {
        /* A split/noisy set should remain observable during commissioning.
           Keep all finite measurements and report its poor spread/confidence
           instead of silently suppressing the line. */
        kept = n;
        for (k = 0u; k < n; k++) {
            keep[k] = 1u;
            robust_w[k] = base_w[k];
        }
    }

    for (iteration = 0u; iteration < 3u; iteration++) {
        float sw = 0.0f, sv = 0.0f;
        for (k = 0u; k < n; k++) {
            if (!keep[k]) continue;
            sw += robust_w[k];
            sv += robust_w[k] * value[k];
        }
        if (sw <= 1e-6f) return 0u;
        center = sv / sw;
        for (k = 0u; k < n; k++) {
            float r, huber;
            if (!keep[k]) continue;
            r = fabsf(value[k] - center);
            huber = (r <= VERTICAL_FUSE_HUBER_MM || r <= 1e-6f)
                  ? 1.0f : VERTICAL_FUSE_HUBER_MM / r;
            robust_w[k] = base_w[k] * huber;
        }
    }

    {
        float err2 = 0.0f;
        uint8_t first = 1u;
        for (k = 0u; k < n; k++) {
            float r;
            if (!keep[k]) continue;
            r = value[k] - center;
            err2 += r * r;
            if (first) { vmin = vmax = value[k]; first = 0u; }
            else {
                if (value[k] < vmin) vmin = value[k];
                if (value[k] > vmax) vmax = value[k];
            }
        }
        rms = sqrtf(err2 / (float)kept);
    }
    {
        VerticalAxisTracker *tracker = &vertical_axis_tracker[group];
        float delta[4];
        uint8_t delta_n = 0u;

        if (tracker->initialized) {
            for (k = 0u; k < n; k++) {
                uint8_t slot = value_slot[k];
                if (tracker->previous_valid[slot])
                    delta[delta_n++] = value[k] - tracker->previous[slot];
            }
        }

        if (!tracker->initialized || delta_n < VERTICAL_FUSE_MIN_POINTS) {
            tracker->coord = center;
            tracker->reanchor_center = center;
            tracker->reanchor_count = 0u;
            tracker->initialized = 1u;
        } else {
            float common_delta;
            float predicted;
            float correction;
            sort_float_small(delta, delta_n);
            common_delta = (delta_n & 1u)
                         ? delta[delta_n / 2u]
                         : 0.5f * (delta[delta_n / 2u - 1u] +
                                   delta[delta_n / 2u]);
            predicted = tracker->coord + common_delta;
            correction = center - predicted;
            if (fabsf(correction) > VERTICAL_FUSE_REANCHOR_MM) {
                /* A line tens of millimetres away from all current centroids
                   is stale state, not useful filtering.  Require a stable
                   current consensus for a few frames, then reacquire it. */
                if (tracker->reanchor_count > 0u &&
                    fabsf(center - tracker->reanchor_center) <=
                        VERTICAL_FUSE_REANCHOR_STABLE_MM) {
                    if (tracker->reanchor_count < 255u)
                        tracker->reanchor_count++;
                } else {
                    tracker->reanchor_center = center;
                    tracker->reanchor_count = 1u;
                }
                if (tracker->reanchor_count >= VERTICAL_FUSE_REANCHOR_FRAMES) {
                    tracker->coord = center;
                    tracker->reanchor_count = 0u;
                } else {
                    tracker->coord = predicted;
                }
            } else {
                tracker->reanchor_count = 0u;
                if (correction > VERTICAL_FUSE_TETHER_MAX_MM)
                    correction = VERTICAL_FUSE_TETHER_MAX_MM;
                else if (correction < -VERTICAL_FUSE_TETHER_MAX_MM)
                    correction = -VERTICAL_FUSE_TETHER_MAX_MM;
                tracker->coord = predicted +
                    VERTICAL_FUSE_TETHER_ALPHA * correction;
            }
        }

        memset(tracker->previous_valid, 0, sizeof(tracker->previous_valid));
        for (k = 0u; k < n; k++) {
            uint8_t slot = value_slot[k];
            tracker->previous[slot] = value[k];
            tracker->previous_valid[slot] = 1u;
        }
        center = tracker->coord;
    }
    *out_coord = center;
    *out_rms = rms;
    *out_spread = vmax - vmin;
    *out_points = kept;
    return 1u;
}

static void __attribute__((unused)) product_debug_update(SelectionResult *selection, const Centroids *c)
{
    uint8_t axis_points[2] = {0u, 0u};
    float axis_rms[2] = {0.0f, 0.0f};
    float axis_spread[2] = {0.0f, 0.0f};
    float x = 0.0f, y = 0.0f;
    uint8_t have_x, have_y;
    memset(&product_debug, 0, sizeof(product_debug));
    product_debug.cross_diff = -1.0f;

    have_x = vertical_axis_fuse(c, 0u, &x, &axis_rms[0],
                                &axis_spread[0], &axis_points[0]);
    have_y = vertical_axis_fuse(c, 1u, &y, &axis_rms[1],
                                &axis_spread[1], &axis_points[1]);

    if (have_x) {
        product_debug.line_x1[0] = x; product_debug.line_y1[0] = -1000.0f;
        product_debug.line_x2[0] = x; product_debug.line_y2[0] =  1000.0f;
        product_debug.line_valid_mask |= 0x01u;
    }
    if (have_y) {
        product_debug.line_x1[1] = -1000.0f; product_debug.line_y1[1] = y;
        product_debug.line_x2[1] =  1000.0f; product_debug.line_y2[1] = y;
        product_debug.line_valid_mask |= 0x02u;
    }
    product_debug.line_valid_mask |= (uint8_t)((axis_points[0] & 0x07u) << 2);
    product_debug.line_valid_mask |= (uint8_t)((axis_points[1] & 0x07u) << 5);
    product_debug.cross_lines = (uint8_t)have_x + (uint8_t)have_y;

    if (have_x && have_y && isfinite(x) && isfinite(y)) {
        selection->x = selection->x_line = x;
        selection->y = selection->y_line = y;
        selection->support_mask = 0u;
        if (c->up.valid && c->in_up.valid) selection->support_mask |= 0x01u;
        if (c->down.valid && c->in_down.valid) selection->support_mask |= 0x02u;
        if (c->left.valid && c->in_left.valid) selection->support_mask |= 0x04u;
        if (c->right.valid && c->in_right.valid) selection->support_mask |= 0x08u;
        selection->n_support = (uint8_t)__builtin_popcount(selection->support_mask);
        selection->mode = (axis_points[0] == 4u && axis_points[1] == 4u) ? 4u :
                          ((axis_points[0] >= 3u && axis_points[1] >= 3u) ? 3u : 2u);
        selection->residual = 0.5f * (axis_rms[0] + axis_rms[1]);
        {
            float max_spread = fmaxf(axis_spread[0], axis_spread[1]);
            float alignment_quality = 1.0f /
                (1.0f + max_spread / VERTICAL_ALIGNMENT_TARGET_MM);
            selection->confidence =
                ((float)(axis_points[0] + axis_points[1]) / 8.0f) * alignment_quality;
        }
        product_debug.x_line = x;
        product_debug.y_line = y;
        product_debug.have_x = 1u;
        product_debug.have_y = 1u;
        product_debug.x_cross_out = x;
        product_debug.y_cross_out = y;
        product_debug.cross_out_valid = 1u;
        product_debug.cross_source = 1u;
        product_debug.cross_conf = selection->confidence;
        product_debug.cross_diff = fmaxf(axis_spread[0], axis_spread[1]);
        return;
    }

    /* 任一方向综合线不足时不输出二维点，不回退到隐藏算法。 */
    selection->mode = 0;
    selection->support_mask = 0;
    selection->n_support = 0;
    selection->confidence = 0.0f;
    product_debug.cross_out_valid = 0u;
}

#if TIMING_DEBUG
/* ==================== 分步耗时实测（仅主控；TIMING_DEBUG=0 时全部剔除） ====================
   原理: DWT->CYCCNT 自由运行周期计数器; 1µs = SystemCoreClock/1e6 个周期(170MHz 下 170)
   口径: acq=本板采集         单点均值 = acq / MY_SENSOR_COUNT
         can=等待全部边到齐(含从机采集/仲裁/传输), canTO=超时次数(总线饱和判据)
         copy=关中断拷贝 cent=八边质心 pos=坐标换算 kal=卡尔曼
         sendCpu=组帧CPU(拆字节+XOR,不含DMA) sendOn=串口在线时间(需 TIMING_MEASURE_ONLINE=1)
         period=相邻两帧起点间隔(含 HAL_Delay)
   ======================================================================================= */
enum { T_ACQ, T_CAN, T_COPY, T_CENT, T_POS, T_KAL, T_SENDCPU, T_SENDON, T_PERIOD, T_N };

static uint32_t t_lap, t_prev;
static uint32_t t_acc[T_N], t_max[T_N];
static uint16_t t_n, t_can_to;

static void timing_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    t_lap = t_prev = 0;
    t_n = t_can_to = 0;
    memset(t_acc, 0, sizeof(t_acc));
    memset(t_max, 0, sizeof(t_max));
}

static inline void tic(void) { t_lap = DWT->CYCCNT; }

static inline void toc(int i)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t d = now - t_lap;
    t_acc[i] += d;
    if (d > t_max[i]) t_max[i] = d;
    t_lap = now;
}

#define T_US(c) ((unsigned long)((c) / (SystemCoreClock / 1000000u)))

static void timing_tick_period(void)
{
    uint32_t now = DWT->CYCCNT;
    if (t_prev) {
        uint32_t p = now - t_prev;
        t_acc[T_PERIOD] += p;
        if (p > t_max[T_PERIOD]) t_max[T_PERIOD] = p;
    }
    t_prev = now;
}

static void timing_report(void)
{
    char m[256];
    uint16_t n = t_n ? t_n : 1;
    int len = sprintf(m,
        "T acq=%lu/%lu can=%lu/%lu copy=%lu cent=%lu pos=%lu kal=%lu sendCpu=%lu sendOn=%lu period=%lu | canTO=%u/%u seen=%08lX fL=%lu pk=%u\r\n",
        T_US(t_acc[T_ACQ] / n), T_US(t_max[T_ACQ]),
        T_US(t_acc[T_CAN] / n), T_US(t_max[T_CAN]),
        T_US(t_acc[T_COPY] / n), T_US(t_acc[T_CENT] / n), T_US(t_acc[T_POS] / n), T_US(t_acc[T_KAL] / n),
        T_US(t_acc[T_SENDCPU] / n),
        T_US(t_acc[T_SENDON] / n),
        T_US(t_acc[T_PERIOD] / n),
        (unsigned)t_can_to, (unsigned)t_n, (unsigned long)can_proto_get_seen_mask(),
        (unsigned long)bsp_sensor_get_laser_freq(), (unsigned)master_peak_idx);
    /* UART TX 用 DMA 且未开 USART2 中断, gState 会卡在 BUSY_TX;
       先 Abort 强制回 READY 再阻塞发送(此刻上一帧 DMA 已完成, Abort 无副作用) */
    HAL_UART_AbortTransmit(&huart2);
    HAL_UART_Transmit(&huart2, (uint8_t *)m, len, 100);
    memset(t_acc, 0, sizeof(t_acc));
    memset(t_max, 0, sizeof(t_max));
    t_n = t_can_to = 0;
}
#endif

void app_core_init(void)
{
    runtime_params_init();
    memset(&centroids, 0, sizeof(centroids));
    memset(&pos_raw, 0, sizeof(pos_raw));
    memset(&pos_filt, 0, sizeof(pos_filt));
    memset(product_candidates, 0, sizeof(product_candidates));
    memset(&product_selection, 0, sizeof(product_selection));
    memset(&product_debug, 0, sizeof(product_debug));
    memset(product_select_bias, 0, sizeof(product_select_bias));
    memset(vertical_axis_tracker, 0, sizeof(vertical_axis_tracker));
    for (int i = 0; i < EDGE_COUNT; i++) edge_len[i] = board_edge_data_count(i);
    can_proto_init();
    kalman_init(&kalman, SAMPLE_PERIOD_MS / 1000.0f);
    last_centroid_filter_enable = runtime_params_get()->centroid_filter_enable;
#if SEND_CAND_DEBUG_FRAME
    kalman_init(&cand_kalman, SAMPLE_PERIOD_MS / 1000.0f);
    memset(&cand_centroids, 0, sizeof(cand_centroids));
    memset(&cand_stable, 0, sizeof(cand_stable));
    memset(cand_pick_pos, 0, sizeof(cand_pick_pos));
    memset(cand_pick_peak, 0, sizeof(cand_pick_peak));
    memset(cand_pick_valid, 0, sizeof(cand_pick_valid));
    memset(&cand_gate_last, 0, sizeof(cand_gate_last));
    memset(&cand_pending, 0, sizeof(cand_pending));
    cand_gate_valid = 0;
    cand_pending_valid = 0;
    cand_pending_count = 0;
    cand_stable_valid = 0;
    cand_lost = 0;
#endif
    centroid_filter_init();
    select_reset();
    bsp_sensor_enable_default_long();   /* 无激光时也先用 4 周期长窗口 */
#if TIMING_DEBUG
    timing_init();
#endif
}

void app_core_run(void)
{
    runtime_params_poll();
    if (runtime_params_take_algorithm_reset()) {
        /* A coordinate map changed: do not let pre-calibration tracking and
           filter history pull the first calibrated result toward an old point. */
        select_reset();
        centroid_filter_init();
        kalman_init(&kalman, SAMPLE_PERIOD_MS / 1000.0f);
        memset(&pos_raw, 0, sizeof(pos_raw));
        memset(&pos_filt, 0, sizeof(pos_filt));
        memset(&product_selection, 0, sizeof(product_selection));
        memset(&product_debug, 0, sizeof(product_debug));
        memset(vertical_axis_tracker, 0, sizeof(vertical_axis_tracker));
#if SEND_CAND_DEBUG_FRAME
        kalman_init(&cand_kalman, SAMPLE_PERIOD_MS / 1000.0f);
        memset(&cand_stable, 0, sizeof(cand_stable));
        cand_stable_valid = 0;
        cand_gate_valid = 0;
        cand_pending_valid = 0;
#endif
    }
#if TIMING_DEBUG
    if (t_n >= TIMING_REPORT_EVERY) timing_report();   /* 打印放在 tic 之前, 不污染分段统计 */
    timing_tick_period();
    tic();
#endif

    // 0. 主控：先测实际 18kHz 基频，再扫描采样相位；50%+ 占空比不再用 36kHz 二次谐波
    if (!master_pulse_aligned && master_peak_idx != 0xFFFF && master_peak_val > 80) {
        uint32_t f = bsp_sensor_measure_laser(master_peak_idx);
        if (f == 0) f = DEMOD_FREQ_HZ;
        bsp_sensor_align_pulse(master_peak_idx, f);
        master_pulse_aligned = 1;
    }

    // 1. 先广播同步帧：各从机收到后立刻开始采集（与主机并行，时序对齐）
    can_proto_send_sync();

    // 2. 主机本地采集（与从机一致，镜像边需翻转本段，保证整条边顺序统一）
    //    段前缀按逐段板型表计算(同边混板时前面各段槽数可能不同)
    uint16_t *my_raw = can_proto_get_edge_data(MY_EDGE) + board_seg_prefix(MY_EDGE, MY_SEGMENT);
    bsp_sensor_acquire(my_raw, MY_SENSOR_COUNT, SENSOR_MODE);
#if TIMING_DEBUG
    toc(T_ACQ);
#endif
#if MY_MIRROR
    mirror_apply(my_raw, MY_SENSOR_COUNT);
#endif
    {
        uint16_t pk = 0;
        for (uint16_t i = 1; i < MY_SENSOR_COUNT; i++)
            if (my_raw[i] > my_raw[pk]) pk = i;
        master_peak_idx = pk;
        master_peak_val = my_raw[pk];
    }
    can_proto_mark_local_ready();

    // 3. 等所有边到齐（超时也继续：台架/缺板调试时缺失的边保持 0）
#if TIMING_DEBUG
    tic();
    if (!can_proto_wait_all_edges(DATA_TIMEOUT_MS)) t_can_to++;
    toc(T_CAN);
#else
    can_proto_wait_all_edges(DATA_TIMEOUT_MS);
#endif

    // 4. 关中断拷贝
    uint16_t *raw_ptrs[EDGE_COUNT];
    /* Dense snapshot: only reserve the system's real maximum point count,
       rather than 512 slots for every edge. */
    static uint16_t local_raw[FRAME_RAW_BYTES_MAX / sizeof(uint16_t)];
    uint8_t cycle_complete_mask = can_proto_get_cycle_edge_mask();
    __disable_irq();
    uint16_t local_offset = 0;
    for (int i = 0; i < EDGE_COUNT; i++) {
        raw_ptrs[i] = &local_raw[local_offset];
        if (cycle_complete_mask & (1u << i))
            memcpy(raw_ptrs[i], can_proto_get_edge_data(i), edge_len[i] * sizeof(uint16_t));
        else
            memset(raw_ptrs[i], 0, edge_len[i] * sizeof(uint16_t));
        local_offset = (uint16_t)(local_offset + edge_len[i]);
    }
    __enable_irq();

#if WAVE_FILTER_ENABLE
    wave_filter_all(raw_ptrs, edge_len);
#endif

    /* 物理段序与默认段序相反的边：整条边反向，不能只做段内镜像 */
#if REVERSE_EDGE_DOWN
    mirror_apply(raw_ptrs[EDGE_DOWN], edge_len[EDGE_DOWN]);
#endif
#if REVERSE_EDGE_IN_DOWN
    mirror_apply(raw_ptrs[EDGE_IN_DOWN], edge_len[EDGE_IN_DOWN]);
#endif
#if REVERSE_EDGE_LEFT
    mirror_apply(raw_ptrs[EDGE_LEFT], edge_len[EDGE_LEFT]);
#endif
#if REVERSE_EDGE_IN_LEFT
    mirror_apply(raw_ptrs[EDGE_IN_LEFT], edge_len[EDGE_IN_LEFT]);
#endif

#if TIMING_DEBUG
    toc(T_COPY);
#endif

    // // ===== 调试：打印拷贝后的从机数据 =====
    // static uint32_t last_print = 0;
    // if (HAL_GetTick() - last_print > 2000) {
    //     last_print = HAL_GetTick();
    //     uint16_t *val = raw_ptrs[MY_EDGE] + 100;  // 从机数据在100-199
    //     char msg[512];
    //     int pos = 0;
    //     pos += sprintf(msg + pos, "COPY all 100:\r\n");
    //     for (int i = 0; i < 100; i++) {
    //         pos += sprintf(msg + pos, "%4d ", val[i]);
    //         if ((i + 1) % 20 == 0) pos += sprintf(msg + pos, "\r\n");
    //     }
    //     HAL_UART_Transmit(&huart2, (uint8_t*)msg, pos, 100);
    // }
    // // ===== 结束 =====

#if RAW_ONLY
    // 纯包络调试：只发全部传感器包络原值，不做质心/坐标/交点
    bsp_uart_wait_done(200);
    frame_send_raw(raw_ptrs, edge_len);
#else
    // 5. 八边质心提取
    centroids_find(raw_ptrs, edge_len, &centroids);

    // 5.1 每边时间稳定：One Euro 自适应低通（dt 由实际帧间隔得到）
    float frame_dt = SAMPLE_PERIOD_MS * 0.001f;
    {
        uint32_t cf_now = HAL_GetTick();
        frame_dt = cf_last_tick ? (float)(cf_now - cf_last_tick) * 0.001f
                                : (SAMPLE_PERIOD_MS * 0.001f);
        cf_last_tick = cf_now;
        uint8_t cf_enable = runtime_params_get()->centroid_filter_enable;
        if (cf_enable && !last_centroid_filter_enable) centroid_filter_init();
        last_centroid_filter_enable = cf_enable;
        if (cf_enable)
            centroid_filter_apply(&centroids, frame_dt);
    }
#if TIMING_DEBUG
    toc(T_CENT);
#endif

    // 6. 候选层保留用于波形/峰值调试；产品几何严格使用同侧内外两个
    //    当前质心连线。上/下二选一、左/右二选一，选中的两线直接求交。
    candidates_find_k(raw_ptrs, edge_len, DEBUG_K, product_candidates);
    runtime_params_calibration_update(&centroids);
    select_run(product_candidates, product_select_bias, &product_selection);
    position_compute(&centroids, &pos_raw, frame_dt);
    {
        const PositionDebug *line_dbg = position_get_debug();
        if (line_dbg != 0)
            product_debug = *line_dbg;
        else {
            memset(&product_debug, 0, sizeof(product_debug));
            product_debug.cross_diff = -1.0f;
        }
    }

    product_selection.x = pos_raw.x;
    product_selection.y = pos_raw.y;
    product_selection.x_line = product_debug.x_line;
    product_selection.y_line = product_debug.y_line;
    product_selection.mode = pos_raw.valid ? 2u : 0u;
    product_selection.confidence = product_debug.cross_conf;
    product_selection.residual = 0.0f;

    pos_raw.valid = (pos_raw.valid &&
                     product_selection.x >= -PRODUCT_SCREEN_MARGIN_MM &&
                     product_selection.x <= SCREEN_W + PRODUCT_SCREEN_MARGIN_MM &&
                     product_selection.y >= -PRODUCT_SCREEN_MARGIN_MM &&
                     product_selection.y <= SCREEN_H + PRODUCT_SCREEN_MARGIN_MM);
#if TIMING_DEBUG
    toc(T_POS);
#endif

    // 7. 位置级 One Euro 滤波（dt 必须用实际帧间隔，旧代码一直用初始化时的 5ms）
    kalman.dt = frame_dt;
    /* 范围保护和异常确认始终执行；运行时开关只控制自适应平滑。
       原始两线交点保留在x_line/y_line，屏幕上的交点使用最终输出。 */
    kalman_update(&kalman, &pos_raw, &pos_filt);
    product_debug.x_cross_out = pos_filt.x;
    product_debug.y_cross_out = pos_filt.y;
    product_debug.cross_out_valid = pos_filt.valid;
    if (kalman.held) product_debug.cross_source = 2u;
    else if (!pos_filt.valid) product_debug.cross_source = 0u;
#if TIMING_DEBUG
    toc(T_KAL);
#endif

    // 8. 等上一帧串口发完再组帧（帧缓冲被 DMA 引用，防止覆盖正在发送的帧）。
    //    放在这里而不是帧尾固定 HAL_Delay(55)：串口在线时间与下一轮采集/等待重叠，
    //    帧周期由 max(串口57ms, 采集+等从机) 决定，而不是两者相加
    bsp_uart_wait_done(200);

    // 9. 单帧下发
#if DEBUG_UI_STREAM
    // Unified diagnostic stream: the UI displays the board-processed layer and
    // firmware's own results without running any signal/position algorithm.
    frame_send_diagnostic(raw_ptrs, edge_len, &centroids,
                          &pos_filt, &product_debug, product_candidates,
                          &product_selection, tx_frame_idx,
                          HAL_GetTick());
    tx_frame_idx++;
#elif SEND_CAND_DEBUG_FRAME
    // Iteration 02 验证：原始波形 + C 候选 + 多边投票结果
    candidates_find_k(raw_ptrs, edge_len, DEBUG_K, cand_debug);
    select_run(cand_debug, select_bias, &sel_debug);

    /* 候选层只负责“每条边选哪个峰”；最终坐标走 position_compute 的
       多线交叉点 + 交点时域稳定。交点无效时不回退到轴向融合点。 */
    {
        Centroid *cp[8] = {
            &cand_centroids.up, &cand_centroids.down,
            &cand_centroids.left, &cand_centroids.right,
            &cand_centroids.in_up, &cand_centroids.in_down,
            &cand_centroids.in_left, &cand_centroids.in_right
        };
        int e;
        memset(&cand_centroids, 0, sizeof(cand_centroids));
        for (e = 0; e < EDGE_COUNT; e++) {
            uint8_t k = pick_edge_candidate((uint8_t)e, &cand_debug[e],
                                            sel_debug.sel_idx[e]);
            sel_debug.sel_idx[e] = k;
            if (k != 0xFF) {
                cp[e]->position = cand_debug[e].c[k].pos;
                cp[e]->peak = cand_debug[e].c[k].peak;
                cp[e]->valid = 1;
            }
        }
        position_compute(&cand_centroids, &cand_pos_raw, frame_dt);

        if (cand_pos_raw.valid) {
            Position gated = cand_pos_raw;
            cand_lost = 0;

            if (!cand_gate_valid) {
                cand_gate_last = cand_pos_raw;
                cand_gate_valid = 1;
                cand_pending_valid = 0;
                cand_pending_count = 0;
            } else {
                float gdx = cand_pos_raw.x - cand_gate_last.x;
                float gdy = cand_pos_raw.y - cand_gate_last.y;
                float gd = sqrtf(gdx * gdx + gdy * gdy);

                if (gd <= CAND_OUT_JUMP_GATE_MM) {
                    cand_gate_last = cand_pos_raw;
                    cand_pending_valid = 0;
                } else if (cand_pending_valid &&
                           (cand_pos_raw.x - cand_pending.x) *
                                   (cand_pos_raw.x - cand_pending.x) +
                           (cand_pos_raw.y - cand_pending.y) *
                                   (cand_pos_raw.y - cand_pending.y) <=
                               CAND_OUT_JUMP_GATE_MM * CAND_OUT_JUMP_GATE_MM) {
                    if (++cand_pending_count >= CAND_OUT_CONFIRM_FRAMES) {
                        cand_gate_last = cand_pos_raw;
                        cand_pending_valid = 0;
                        gated = cand_pos_raw;
                    } else {
                        gated = cand_gate_last;
                    }
                } else {
                    cand_pending = cand_pos_raw;
                    cand_pending_valid = 1;
                    cand_pending_count = 1;
                    gated = cand_gate_last;
                }
            }
            gated.valid = 1;

            cand_kalman.dt = frame_dt;
            kalman_update(&cand_kalman, &gated, &cand_pos_filt);
            if (cand_pos_filt.valid) {
                cand_stable = cand_pos_filt;
                cand_stable_valid = 1;
            }
        } else if (cand_stable_valid) {
            cand_pending_valid = 0;
            /* 允许短暂丢点保持，但无激光持续超过 5 帧就彻底失效，
               不能一直把上一帧的点当成当前点。 */
            if (++cand_lost > 5) {
                cand_stable_valid = 0;
                cand_lost = 0;
                cand_gate_valid = 0;
                cand_pending_valid = 0;
                kalman_init(&cand_kalman, SAMPLE_PERIOD_MS / 1000.0f);
            }
        }

        if (cand_stable_valid) {
            sel_debug.x = cand_stable.x;
            sel_debug.y = cand_stable.y;
            if (sel_debug.mode < 2) sel_debug.mode = 2;
            if (sel_debug.confidence < 0.15f) sel_debug.confidence = 0.15f;
        } else {
            sel_debug.x = 0.0f;
            sel_debug.y = 0.0f;
            sel_debug.mode = 0;
            sel_debug.confidence = 0.0f;
            sel_debug.support_mask = 0;
            sel_debug.n_support = 0;
            sel_debug.hq_support = 0;
        }

        {
            const PositionDebug *dbg = position_get_debug();
            if (dbg != 0) {
                sel_debug.x_line = dbg->x_cross_out;
                sel_debug.y_line = dbg->y_cross_out;
                sel_debug.line_valid = dbg->cross_out_valid;
                sel_debug.cross_lines = dbg->cross_lines;
                sel_debug.cross_source = dbg->cross_source;
                sel_debug.cross_conf = dbg->cross_conf;
                sel_debug.cross_diff = dbg->cross_diff;
            } else {
                sel_debug.x_line = 0.0f;
                sel_debug.y_line = 0.0f;
                sel_debug.line_valid = 0;
                sel_debug.cross_lines = 0;
                sel_debug.cross_source = 0;
                sel_debug.cross_conf = 0.0f;
                sel_debug.cross_diff = -1.0f;
            }
        }
    }

    frame_send_candidates(raw_ptrs, edge_len, cand_debug, &sel_debug);
    tx_frame_idx++;
#elif SEND_FULL_FRAME
    // 全帧：原始波形 + 8边质心 + 坐标 + 交点调试（调试/看波形；会把帧率锁在 ~17FPS）
    frame_send_full(raw_ptrs, edge_len, &centroids, &pos_filt, &product_debug);
    tx_frame_idx++;
#elif FULL_FRAME_EVERY_N > 0
    // 产品模式为主，每 N 帧插一帧全帧供调试
    if ((tx_frame_idx % FULL_FRAME_EVERY_N) == 0) {
        frame_send_full(raw_ptrs, edge_len, &centroids, &pos_filt, &product_debug);
    } else {
        frame_send_position(&pos_filt);
    }
    tx_frame_idx++;
#else
    // 坐标帧：x/y/valid 14B，产品模式
    frame_send_position(&pos_filt);
#endif
#endif /* RAW_ONLY */

#if TIMING_DEBUG
    toc(T_SENDCPU);
#if TIMING_MEASURE_ONLINE
    bsp_uart_wait_done(2000);
    toc(T_SENDON);
#endif
    t_n++;
#endif
}

#else
void app_core_init(void) {}
void app_core_run(void) {}
#endif
