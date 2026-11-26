// select.c
//
// 同边内外候选配对成视线 + 多边交叉一致性投票。
// 输出每条边选中的候选索引、支持数/高质量支持/残差/交叉点；
// 不使用轴向融合点，也不在交叉失效时回退到轴向位置。
#include "select.h"
#include "board_map.h"
#include "config.h"
#include "runtime_params.h"

#include <math.h>
#include <string.h>

#define SIDE_TOP     0
#define SIDE_BOTTOM  1
#define SIDE_LEFT    2
#define SIDE_RIGHT   3

static const uint8_t s_outer_edge[4] = {
    EDGE_UP, EDGE_DOWN, EDGE_LEFT, EDGE_RIGHT
};
static const uint8_t s_inner_edge[4] = {
    EDGE_IN_UP, EDGE_IN_DOWN, EDGE_IN_LEFT, EDGE_IN_RIGHT
};

static SightLine s_lines[4][MAX_LINES];
static SelectionResult s_prev;
static SelectionResult s_pending;
static float s_out_x, s_out_y;
static float s_vel_x, s_vel_y;
static uint8_t s_prev_valid;
static float s_axis_prev[4];
static uint8_t s_axis_prev_valid[4];
static float s_cand_prev[8];
static float s_cand_prev_peak[8];
static uint8_t s_cand_prev_valid[8];
static uint8_t s_track_gate[8];
static uint8_t s_pending_valid;
static uint8_t s_same_count;
static uint8_t s_pending_count;
static uint8_t s_hold_count;

/* 同侧内外边不是两个独立点，而是一个“共同位置 q + 视差 d”传感器。
   q 可随光点快速移动；d 只有在连续多帧得到证实时才允许成为真实斜率。
   这样单边质心跳动会首先表现为 d 突变，而不会立即倾斜整条视线。 */
typedef struct {
    float q;
    float vq;
    float d;
    float vd;
    float pending_d;
    uint8_t valid;
    uint8_t slant;
    /* 斜率模型渐变系数，避免 d=0 与完整 d_rel 之间单帧跳变。 */
    float slant_blend;
    uint8_t pending_count;
    uint8_t quiet_count;
} PairState;

static PairState s_pair_state[4];

static float clamp_abs(float v, float limit)
{
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}

/* 给候选内外对计算联合跟踪权重，并给出真正用于构线的视差。
   垂直模型下 d_used=0；斜入射模型下只使用经过状态约束的 d。 */
static float pair_constraint(uint8_t side, float axis, float d_meas,
                             float *d_used)
{
    const PairState *s = &s_pair_state[side];
    float w = 1.0f;

    if (!s->valid) {
        *d_used = 0.0f;
        /* 首次出现大视差只轻度降权，保留连续多帧确认真实斜入射的机会。 */
        if (fabsf(d_meas) > SELECT_PAIR_VERTICAL_GATE_MM)
            w /= 1.0f + (fabsf(d_meas) - SELECT_PAIR_VERTICAL_GATE_MM) /
                         SELECT_PAIR_ACQUIRE_SCALE_MM;
        return w;
    }

    {
        float q_pred = s->q + s->vq;
        float dq = fabsf(axis - q_pred);
        w /= 1.0f + dq / SELECT_PAIR_COMMON_SCALE_MM;
    }

    if (!s->slant) {
        float ad = fabsf(d_meas);
        /* 退出斜率模型后也允许系数平滑衰减，避免突然拉直。 */
        *d_used = s->d * s->slant_blend;
        if (ad > SELECT_PAIR_VERTICAL_GATE_MM)
            w /= 1.0f + (ad - SELECT_PAIR_VERTICAL_GATE_MM) /
                         SELECT_PAIR_ACQUIRE_SCALE_MM;
    } else {
        float d_pred = s->d + s->vd;
        float innovation = d_meas - d_pred;
        float limited = clamp_abs(innovation, SELECT_PAIR_D_STEP_MM);
        *d_used = (d_pred + SELECT_PAIR_D_MEAS_ALPHA * limited) *
                  s->slant_blend;
        /* 单边跳变造成的差分突变会被强降权；真实角度改变可在连续确认后重获。 */
        w /= 1.0f + fabsf(innovation) / SELECT_PAIR_D_INNOV_SCALE_MM;
    }
    return w;
}

static uint8_t pending_matches(PairState *s, float d)
{
    if (s->pending_count > 0u &&
        fabsf(d - s->pending_d) <= SELECT_PAIR_D_CONFIRM_GATE_MM) {
        s->pending_d += SELECT_PAIR_PENDING_ALPHA * (d - s->pending_d);
        if (s->pending_count < 255u) s->pending_count++;
    } else {
        s->pending_d = d;
        s->pending_count = 1u;
    }
    return s->pending_count >= SELECT_PAIR_D_CONFIRM_FRAMES;
}

static void pair_state_update(uint8_t side, const SightLine *line)
{
    PairState *s = &s_pair_state[side];
    float q = line->axis;
    float d = line->d_rel;

    if (!s->valid) {
        s->q = q;
        s->vq = 0.0f;
        s->d = 0.0f;
        s->vd = 0.0f;
        s->valid = 1u;
        s->slant = 0u;
        s->slant_blend = 0.0f;
        s->pending_count = 0u;
        s->quiet_count = 0u;
    } else {
        float dq = q - s->q;
        s->vq = SELECT_PAIR_Q_VEL_ALPHA * dq +
                (1.0f - SELECT_PAIR_Q_VEL_ALPHA) * s->vq;
        s->q += SELECT_PAIR_Q_ALPHA * dq;
    }

    if (!s->slant) {
        if (fabsf(d) >= SELECT_PAIR_SLANT_ENTER_MM) {
            if (pending_matches(s, d)) {
                s->slant = 1u;
                s->d = s->pending_d;
                s->vd = 0.0f;
                s->slant_blend = 0.0f;
                s->pending_count = 0u;
                s->quiet_count = 0u;
            }
        } else {
            s->pending_count = 0u;
            s->d += SELECT_PAIR_D_ZERO_ALPHA * (0.0f - s->d);
        }
    } else {
        float d_pred = s->d + s->vd;
        float innovation = d - d_pred;
        if (fabsf(innovation) <= SELECT_PAIR_D_STEP_MM) {
            float old_d = s->d;
            s->d += SELECT_PAIR_D_STATE_ALPHA * (d - s->d);
            s->vd = SELECT_PAIR_D_VEL_ALPHA * (s->d - old_d) +
                    (1.0f - SELECT_PAIR_D_VEL_ALPHA) * s->vd;
            s->pending_count = 0u;
        } else if (pending_matches(s, d)) {
            /* 角度确实发生改变：连续多帧一致后才重置视差状态。 */
            s->d = s->pending_d;
            s->vd = 0.0f;
            s->pending_count = 0u;
        }

        if (fabsf(d) <= SELECT_PAIR_SLANT_EXIT_MM) {
            if (s->quiet_count < 255u) s->quiet_count++;
            if (s->quiet_count >= SELECT_PAIR_D_EXIT_FRAMES) {
                s->slant = 0u;
                s->vd = 0.0f;
                s->pending_count = 0u;
                s->quiet_count = 0u;
            }
        } else {
            s->quiet_count = 0u;
        }
    }

    /* 斜率只渐变进入/退出，不允许单帧改变整条线方向。 */
    if (s->slant) {
        s->slant_blend += 0.20f;
        if (s->slant_blend > 1.0f) s->slant_blend = 1.0f;
    } else {
        s->slant_blend -= 0.20f;
        if (s->slant_blend < 0.0f) s->slant_blend = 0.0f;
    }
}

static float edge_to_x(uint8_t edge, float pos)
{
    return runtime_params_map_edge(edge, pos);
}

static float edge_to_y(uint8_t edge, float pos)
{
    return runtime_params_map_edge(edge, pos);
}

static float cross_seam_weight(uint8_t eo, float po, uint8_t ei, float pi)
{
    uint16_t bo, bi;

    if (eo != EDGE_LEFT || ei != EDGE_IN_LEFT) return 1.0f;
    bo = board_seg_prefix(EDGE_LEFT, 1);
    bi = board_seg_prefix(EDGE_IN_LEFT, 1);
    if ((po < (float)bo) != (pi < (float)bi)) return SELECT_SEAM_W;
    return 1.0f;
}

static uint8_t make_line(float x1, float y1, float x2, float y2,
                         float w, uint8_t side, uint8_t oi, uint8_t ii,
                         float d, float d_rel, SightLine *L)
{
    float A = y2 - y1;
    float B = x1 - x2;
    float C = x2 * y1 - x1 * y2;
    float n = sqrtf(A * A + B * B);
    if (n < 1e-6f || w < 1e-6f) return 0;

    L->A = A / n;
    L->B = B / n;
    L->C = C / n;
    L->x1 = x1;
    L->y1 = y1;
    L->x2 = x2;
    L->y2 = y2;
    L->w = w;
    L->side = side;
    L->o_idx = oi;
    L->i_idx = ii;
    L->d = d;
    L->d_rel = d_rel;
    return 1;
}

static uint8_t cand_near_prev(uint8_t edge, const EdgeCandidate *c)
{
    return (fabsf(c->pos - s_cand_prev[edge]) <= SELECT_CAND_TRACK);
}

/* 候选连续性权重：优先保留上一帧附近、且峰值没有明显变强的候选，
   避免鬼影/弱峰把同一光斑的候选抢走。 */
static float cand_track_factor(uint8_t edge, const EdgeCandidate *c)
{
    float d;
    if (!s_cand_prev_valid[edge] || !s_track_gate[edge]) return 1.0f;
    if (c->peak > s_cand_prev_peak[edge] * SELECT_CAND_UPGRADE) return 1.0f;
    d = fabsf(c->pos - s_cand_prev[edge]);
    if (d <= SELECT_CAND_TRACK) return 1.0f;
    return SELECT_CAND_TRACK / (SELECT_CAND_TRACK + d);
}

static uint8_t edge_has_near_prev(uint8_t edge, const EdgeCandidates *c)
{
    uint8_t k, near = 0;

    if (!s_cand_prev_valid[edge]) return 0;
    for (k = 0; k < c->n && k < MAX_CAND; k++) {
        if (cand_near_prev(edge, &c->c[k])) {
            near = 1;
        } else if (c->c[k].peak >
                   s_cand_prev_peak[edge] * SELECT_CAND_UPGRADE) {
            return 0;
        }
    }
    return near;
}

static void build_lines(const EdgeCandidates cand[8], const float bias[4],
                        uint8_t n_lines[4])
{
    uint8_t side;

    for (side = 0; side < 4; side++) {
        const EdgeCandidates *co = &cand[s_outer_edge[side]];
        const EdgeCandidates *ci = &cand[s_inner_edge[side]];
        uint8_t o, i;

        n_lines[side] = 0;
        if (co->n == 0 || ci->n == 0) continue;

        for (o = 0; o < co->n && o < MAX_CAND; o++) {
            for (i = 0; i < ci->n && i < MAX_CAND; i++) {
                float outer_along, inner_along;
                float x1, y1, x2, y2;
                float d, d_rel, d_used, wd, w, axis, pair_w;

                if (side == SIDE_TOP || side == SIDE_BOTTOM) {
                    outer_along = edge_to_x(s_outer_edge[side], co->c[o].pos);
                    inner_along = edge_to_x(s_inner_edge[side], ci->c[i].pos);
                    if (side == SIDE_TOP) {
                        x1 = outer_along; y1 = SCREEN_H + IN_UP_OFFSET_Y;
                        x2 = inner_along; y2 = SCREEN_H;
                    } else {
                        x1 = outer_along; y1 = -IN_DOWN_OFFSET_Y;
                        x2 = inner_along; y2 = 0.0f;
                    }
                } else {
                    outer_along = edge_to_y(s_outer_edge[side], co->c[o].pos);
                    inner_along = edge_to_y(s_inner_edge[side], ci->c[i].pos);
                    if (side == SIDE_LEFT) {
                        x1 = -IN_LEFT_OFFSET_X; y1 = outer_along;
                        x2 = 0.0f; y2 = inner_along;
                    } else {
                        x1 = SCREEN_W + IN_RIGHT_OFFSET_X; y1 = outer_along;
                        x2 = SCREEN_W; y2 = inner_along;
                    }
                }

                /* 视线方向必须来自当前帧这一对内外候选。
                   旧实现用每侧一个共享状态过滤 d，多候选组合会相互覆盖，
                   有可能把A组合的方向用到B组合上。 */
                d = inner_along - outer_along;
                d_rel = d - bias[side];
                if (fabsf(d_rel) > SELECT_D_HARD) continue;

                axis = (outer_along + inner_along) * 0.5f;
                /* 当前边的共同位置先做时间约束，再参与组合评分和构线。
                   小幅跳动不会把两条综合线的交点放大到远处；真实移动
                   超过快门限时立即跟随，避免拖尾。 */
                if (s_axis_prev_valid[side]) {
                    float da = axis - s_axis_prev[side];
                    float aa = (fabsf(da) >= SELECT_AXIS_FAST_GATE_MM)
                             ? 1.0f : SELECT_AXIS_SMOOTH_ALPHA;
                    axis = s_axis_prev[side] + aa * da;
                }
                pair_w = pair_constraint(side, axis, d_rel, &d_used);
                /* 固定装配偏差由标定/bias去除；以共同中心为锚点重建端点。
                   垂直模型 d_used=0，环境引起的小错位不会变成假斜率。 */
                outer_along = axis - 0.5f * d_used;
                inner_along = axis + 0.5f * d_used;
                if (side == SIDE_TOP || side == SIDE_BOTTOM) {
                    x1 = outer_along;
                    x2 = inner_along;
                } else {
                    y1 = outer_along;
                    y2 = inner_along;
                }
                wd = expf(-0.5f * (d_rel / SELECT_D_SOFT) *
                                 (d_rel / SELECT_D_SOFT));
                w = co->c[o].quality * ci->c[i].quality * wd
                  * cross_seam_weight(s_outer_edge[side], co->c[o].pos,
                                      s_inner_edge[side], ci->c[i].pos);
                w *= pair_w;
                w *= 0.5f + sqrtf(co->c[o].peak * ci->c[i].peak) / 0.06f;
                w *= cand_track_factor(s_outer_edge[side], &co->c[o]);
                w *= cand_track_factor(s_inner_edge[side], &ci->c[i]);
                if (s_axis_prev_valid[side]) {
                    float dd = fabsf(axis - s_axis_prev[side]);
                    w *= 1.0f / (1.0f + dd / SELECT_AXIS_TEMP_SCALE);
                }
                if (w < 1e-5f) continue;

                if (n_lines[side] >= MAX_LINES) break;
                if (make_line(x1, y1, x2, y2, w, side, o, i, d, d_rel,
                              &s_lines[side][n_lines[side]])) {
                    s_lines[side][n_lines[side]].axis = axis;
                    s_lines[side][n_lines[side]].d_used = d_used;
                    n_lines[side]++;
                }
            }
        }
    }
}

static void score_point(float px, float py, const uint8_t n_lines[4],
                        SelectionResult *h)
{
    uint8_t side;

    memset(h, 0, sizeof(*h));
    for (side = 0; side < 4; side++) h->line_idx[side] = 0xFF;
    for (side = 0; side < 8; side++) h->sel_idx[side] = 0xFF;

    h->x = px;
    h->y = py;

    {
        float axis_val[4] = {0, 0, 0, 0};
        uint8_t axis_has[4] = {0, 0, 0, 0};

        for (side = 0; side < 4; side++) {
            uint8_t k, best = 0xFF;
            float best_d = 1e9f;

            for (k = 0; k < n_lines[side]; k++) {
                const SightLine *L = &s_lines[side][k];
                float d = fabsf(L->A * px + L->B * py + L->C);
                if (d < best_d) {
                    best_d = d;
                    best = k;
                }
            }
            if (best == 0xFF) continue;
            if (s_lines[side][best].w >= SELECT_MIN_LINE_W) {
                axis_val[side] = s_lines[side][best].axis;
                axis_has[side] = 1;
            }
            if (best_d > SELECT_RES_GATE) continue;
            if (s_lines[side][best].w < SELECT_MIN_LINE_W) continue;

            h->support_mask |= (uint8_t)(1u << side);
            h->n_support++;
            h->line_idx[side] = best;
            if (s_lines[side][best].w >= SELECT_HQ_W) h->hq_support++;
        }

        {
            float xmin = 1e9f, xmax = -1e9f;
            float ymin = 1e9f, ymax = -1e9f;
            float spread = 0.0f;
            uint8_t nx = 0, ny = 0;
            for (side = 0; side < 4; side++) {
                if (!axis_has[side]) continue;
                if (side == SIDE_TOP || side == SIDE_BOTTOM) {
                    if (axis_val[side] < xmin) xmin = axis_val[side];
                    if (axis_val[side] > xmax) xmax = axis_val[side];
                    nx++;
                } else {
                    if (axis_val[side] < ymin) ymin = axis_val[side];
                    if (axis_val[side] > ymax) ymax = axis_val[side];
                    ny++;
                }
            }
            if (nx >= 2) spread += xmax - xmin;
            if (ny >= 2) spread += ymax - ymin;
            h->score = 10.5f * (float)h->n_support
                     + 3.0f * (float)h->hq_support
                     - SELECT_AXIS_SPREAD_W * spread;
        }
    }

    {
        float sumw = 0.0f, sumwd2 = 0.0f;
        for (side = 0; side < 4; side++) {
            uint8_t idx = h->line_idx[side];
            if (idx == 0xFF) continue;
            {
                const SightLine *L = &s_lines[side][idx];
                float dist = fabsf(L->A * px + L->B * py + L->C);
                sumw += L->w;
                sumwd2 += L->w * dist * dist;
            }
        }
        h->residual = (sumw > 1e-9f) ? sqrtf(sumwd2 / sumw) : 0.0f;
        h->score -= 0.2f * h->residual;
    }

    h->confidence = (float)h->n_support / 4.0f;
    if (h->hq_support < 2) h->confidence *= 0.5f;
    h->confidence *= 1.0f / (1.0f + h->residual / 30.0f);
    if (h->confidence > 1.0f) h->confidence = 1.0f;
}

static uint8_t finite_cross(float x, float y)
{
    return (isfinite(x) && isfinite(y));
}

/* 对当前假设选中的多条视线做鲁棒加权最小二乘：
 *
 *   min Σ wi * rho(Ai*x + Bi*y + Ci)
 *
 * Ai/Bi 已归一化，因此残差单位为 mm。每次迭代先由 score_point() 在
 * top/bottom/left/right 各选离当前交点最近且通过质量门限的一条视线，
 * 再用全部已选视线共同求解。Huber 权重使斜入射或单边误选不会把其余
 * 三边拖偏；下一轮会在新交点附近重新选择候选组合。
 *
 * 返回0表示线束退化（只有同向线或数值条件太差），调用者保留原始
 * 正交线对交点，不会产生无穷远坐标。 */
static uint8_t robust_refine_point(const uint8_t n_lines[4],
                                   float *px, float *py)
{
    uint8_t iteration;
    float x = *px, y = *py;
    uint8_t fixed_idx[4];
    SelectionResult initial;

    /* 先确定一组候选线，IRLS期间固定这组线。若每轮都重新按最近距离
       换线，两个相近峰会在迭代中交替成为“最近线”，交点方向随之跳变。 */
    score_point(x, y, n_lines, &initial);
    if (initial.n_support < 2) return 0;
    memcpy(fixed_idx, initial.line_idx, sizeof(fixed_idx));

    for (iteration = 0; iteration < SELECT_WLS_ITERATIONS; iteration++) {
        float m00 = 0.0f, m01 = 0.0f, m11 = 0.0f;
        float b0 = 0.0f, b1 = 0.0f;
        float trace, det, det_ratio;
        uint8_t side, used = 0;

        for (side = 0; side < 4; side++) {
            uint8_t idx = fixed_idx[side];
            const SightLine *L;
            float residual, robust, weight, rhs;
            if (idx == 0xFF) continue;
            L = &s_lines[side][idx];

            residual = fabsf(L->A * x + L->B * y + L->C);
            robust = (residual <= SELECT_WLS_HUBER_MM || residual <= 1e-6f)
                   ? 1.0f : SELECT_WLS_HUBER_MM / residual;

            /* sqrt压缩强弱边的动态范围：强边仍更可信，但不会完全淹没
               另一正交方向的弱边。 */
            weight = sqrtf(L->w) * robust;
            rhs = -L->C;
            m00 += weight * L->A * L->A;
            m01 += weight * L->A * L->B;
            m11 += weight * L->B * L->B;
            b0  += weight * L->A * rhs;
            b1  += weight * L->B * rhs;
            used++;
        }

        if (used < 2) return 0;
        trace = m00 + m11;
        det = m00 * m11 - m01 * m01;
        det_ratio = (trace > 1e-9f) ? det / (trace * trace) : 0.0f;
        if (det <= 1e-9f || det_ratio < SELECT_WLS_MIN_DET_RATIO) return 0;

        {
            float nx = (b0 * m11 - b1 * m01) / det;
            float ny = (m00 * b1 - m01 * b0) / det;
            if (!finite_cross(nx, ny)) return 0;
            x = nx;
            y = ny;
        }
    }

    *px = x;
    *py = y;
    return 1;
}

static float selected_line_weight(const SelectionResult *h)
{
    float sum = 0.0f;
    uint8_t side;
    for (side = 0; side < 4; side++) {
        uint8_t idx = h->line_idx[side];
        if (idx != 0xFF) sum += s_lines[side][idx].w;
    }
    return sum;
}

void select_reset(void)
{
    memset(s_lines, 0, sizeof(s_lines));
    memset(&s_prev, 0, sizeof(s_prev));
    memset(&s_pending, 0, sizeof(s_pending));
    memset(s_axis_prev, 0, sizeof(s_axis_prev));
    memset(s_axis_prev_valid, 0, sizeof(s_axis_prev_valid));
    memset(s_cand_prev, 0, sizeof(s_cand_prev));
    memset(s_cand_prev_peak, 0, sizeof(s_cand_prev_peak));
    memset(s_cand_prev_valid, 0, sizeof(s_cand_prev_valid));
    memset(s_track_gate, 0, sizeof(s_track_gate));
    memset(s_pair_state, 0, sizeof(s_pair_state));
    s_out_x = s_out_y = 0.0f;
    s_vel_x = s_vel_y = 0.0f;
    s_prev_valid = 0;
    s_pending_valid = 0;
    s_same_count = 0;
    s_pending_count = 0;
    s_hold_count = 0;
}

uint8_t select_get_line(uint8_t side, uint8_t line_idx,
                        float *x1, float *y1, float *x2, float *y2)
{
    const SightLine *line;
    if (side >= 4 || line_idx == 0xFF || line_idx >= MAX_LINES) return 0;
    line = &s_lines[side][line_idx];
    if (!isfinite(line->x1) || !isfinite(line->y1) ||
        !isfinite(line->x2) || !isfinite(line->y2)) return 0;
    if (x1) *x1 = line->x1;
    if (y1) *y1 = line->y1;
    if (x2) *x2 = line->x2;
    if (y2) *y2 = line->y2;
    return 1;
}

void select_run(const EdgeCandidates cand[8], const float bias[4],
                SelectionResult *out)
{
    uint8_t n_lines[4];
    uint8_t s1, s2;
    SelectionResult best;

    /* 只在旧候选仍然存在时才使用时域加权。光斑快速移出跟踪半径后
       立即重新获取，不让已消失的旧位置压低所有新候选。 */
    for (s1 = 0; s1 < 8; s1++)
        s_track_gate[s1] = edge_has_near_prev(s1, &cand[s1]);
    build_lines(cand, bias, n_lines);

    memset(&best, 0, sizeof(best));
    for (s1 = 0; s1 < 4; s1++) best.line_idx[s1] = 0xFF;
    for (s1 = 0; s1 < 8; s1++) best.sel_idx[s1] = 0xFF;
    best.score = -1e9f;

    for (s1 = SIDE_TOP; s1 <= SIDE_BOTTOM; s1++) {
        uint8_t i1;
        for (i1 = 0; i1 < n_lines[s1]; i1++) {
            const SightLine *L1 = &s_lines[s1][i1];
            for (s2 = SIDE_LEFT; s2 <= SIDE_RIGHT; s2++) {
                uint8_t i2;
                for (i2 = 0; i2 < n_lines[s2]; i2++) {
                    const SightLine *L2 = &s_lines[s2][i2];
                    float det = L1->A * L2->B - L2->A * L1->B;
                    float x, y;
                    SelectionResult h;

                    if (fabsf(det) < SELECT_DET_MIN) continue;

                    x = (L1->B * L2->C - L2->B * L1->C) / det;
                    y = (L2->A * L1->C - L1->A * L2->C) / det;
                    if (!finite_cross(x, y)) continue;

                    /* 正交线对只负责产生RANSAC种子；最终坐标由种子附近
                       top/bottom/left/right全部一致视线共同鲁棒拟合。 */
                    (void)robust_refine_point(n_lines, &x, &y);
                    score_point(x, y, n_lines, &h);
                    h.score += SELECT_PAIR_W * selected_line_weight(&h);
                    if (s_prev_valid) {
                        float pdx = x - (s_out_x + s_vel_x);
                        float pdy = y - (s_out_y + s_vel_y);
                        h.score -= SELECT_COMBO_POS_W * sqrtf(pdx * pdx + pdy * pdy);
                        if (h.line_idx[s1] == s_prev.line_idx[s1])
                            h.score += SELECT_COMBO_LINE_W;
                        if (h.line_idx[s2] == s_prev.line_idx[s2])
                            h.score += SELECT_COMBO_LINE_W;
                    }
                    /* 探测边数量采用硬优先级：4边一定优先于3边，3边一定
                       优先于2边；只有参与边数相同时才比较质量、强度、
                       残差与时间连续性综合分数。 */
                    if (h.n_support > best.n_support ||
                        (h.n_support == best.n_support && h.score > best.score))
                        best = h;
                }
            }
        }
    }

    if (best.n_support >= 4) best.mode = 4;
    else if (best.n_support == 3) best.mode = 3;
    else if (best.n_support == 2) best.mode = 2;
    else best.mode = 0;

    /* 把获胜线还原成每条边选中的候选索引 */
    for (s1 = 0; s1 < 4; s1++) {
        uint8_t idx = best.line_idx[s1];
        if (idx == 0xFF) continue;
        best.sel_idx[s_outer_edge[s1]] = s_lines[s1][idx].o_idx;
        best.sel_idx[s_inner_edge[s1]] = s_lines[s1][idx].i_idx;
    }

    /* best.x/best.y 始终保留视线交点；禁止轴向融合替换。 */
    best.x_axis = best.x;
    best.y_axis = best.y;

    /* 杩唬 3锛氱疆淇″害鍑嗗叆 + 杩炵画纭 + 涓㈢偣淇濇寔 1 甯?*/
    {
        uint8_t orthogonal = ((best.support_mask & 0x03u) != 0u &&
                              (best.support_mask & 0x0Cu) != 0u);
        uint8_t sane = (best.x >= -SELECT_INTERSECTION_MARGIN_MM &&
                        best.x <= SCREEN_W + SELECT_INTERSECTION_MARGIN_MM &&
                        best.y >= -SELECT_INTERSECTION_MARGIN_MM &&
                        best.y <= SCREEN_H + SELECT_INTERSECTION_MARGIN_MM);
        uint8_t valid = (best.n_support >= 2 && orthogonal && sane &&
                         best.mode >= 2 &&
                         best.confidence >= SELECT_CONF_MIN);
        if (valid) {
            float speed = sqrtf(s_vel_x * s_vel_x + s_vel_y * s_vel_y);
            float gate = SELECT_TEMPORAL_GATE_MM + SELECT_VEL_GATE_GAIN * speed;
            float meas_x = best.x, meas_y = best.y;
            float dist_prev = 1e9f;
            uint8_t accept = 0;
            uint8_t fast_accept = 0;

            s_hold_count = 0;
            if (s_prev_valid) {
                float dx = meas_x - s_prev.x;
                float dy = meas_y - s_prev.y;
                dist_prev = sqrtf(dx * dx + dy * dy);
            }

            if (!s_prev_valid) {
                float pdx = meas_x - s_pending.x;
                float pdy = meas_y - s_pending.y;
                float pdist = sqrtf(pdx * pdx + pdy * pdy);
                if (s_pending_valid && pdist <= SELECT_TEMPORAL_GATE_MM) {
                    if (s_pending_count < 255) s_pending_count++;
                } else {
                    s_pending = best;
                    s_pending_count = 1;
                    s_pending_valid = 1;
                }
                if (s_pending_count >= SELECT_CONFIRM_FRAMES) {
                    s_out_x = meas_x;
                    s_out_y = meas_y;
                    s_vel_x = s_vel_y = 0.0f;
                    s_same_count = SELECT_CONFIRM_FRAMES;
                    s_pending_valid = 0;
                    accept = 1;
                } else {
                    best.mode = 0;
                    best.support_mask = 0;
                    best.n_support = 0;
                    best.hq_support = 0;
                    best.confidence = 0.0f;
                    for (s1 = 0; s1 < 4; s1++) best.line_idx[s1] = 0xFF;
                    for (s1 = 0; s1 < 8; s1++) best.sel_idx[s1] = 0xFF;
                }
            } else if (dist_prev <= gate) {
                accept = 1;
                s_pending_valid = 0;
                if (s_same_count < 255) s_same_count++;
            } else {
                float pdx = meas_x - s_pending.x;
                float pdy = meas_y - s_pending.y;
                float pdist = sqrtf(pdx * pdx + pdy * pdy);

                if (s_pending_valid && pdist <= SELECT_TEMPORAL_GATE_MM) {
                    if (s_pending_count < 255) s_pending_count++;
                } else {
                    s_pending = best;
                    s_pending_count = 1;
                    s_pending_valid = 1;
                }

                if (s_pending_count >= SELECT_CONFIRM_FRAMES) {
                    accept = 1;
                    fast_accept = 1;
                    s_same_count = SELECT_CONFIRM_FRAMES;
                    s_pending_valid = 0;
                } else {
                    best = s_prev;
                    best.x = s_out_x;
                    best.y = s_out_y;
                    best.confidence *= 0.5f;
                }
            }

            if (accept) {
                if (s_prev_valid) {
                    float dvx = meas_x - s_prev.x;
                    float dvy = meas_y - s_prev.y;
                    float speed_meas = sqrtf(dvx * dvx + dvy * dvy);
                    float alpha;

                    s_vel_x = SELECT_VEL_ALPHA * dvx + (1.0f - SELECT_VEL_ALPHA) * s_vel_x;
                    s_vel_y = SELECT_VEL_ALPHA * dvy + (1.0f - SELECT_VEL_ALPHA) * s_vel_y;

                    if (fast_accept) {
                        alpha = 1.0f;
                    } else {
                        alpha = SELECT_OUTPUT_ALPHA_MIN +
                                speed_meas / SELECT_OUTPUT_SPEED_SCALE;
                        if (alpha > 1.0f) alpha = 1.0f;
                    }
                    s_out_x += alpha * (meas_x - s_out_x);
                    s_out_y += alpha * (meas_y - s_out_y);
                }
                if (s_same_count < SELECT_CONFIRM_FRAMES) {
                    best.confidence *= 0.5f;
                }
                for (s1 = 0; s1 < 4; s1++) {
                    uint8_t li = best.line_idx[s1];
                    if (li != 0xFF) {
                        s_axis_prev[s1] = s_lines[s1][li].axis;
                        s_axis_prev_valid[s1] = 1;
                        pair_state_update(s1, &s_lines[s1][li]);
                    } else {
                        s_axis_prev_valid[s1] = 0;
                    }
                }
                for (s1 = 0; s1 < 8; s1++) {
                    uint8_t ci = best.sel_idx[s1];
                    if (ci != 0xFF && ci < cand[s1].n) {
                        s_cand_prev[s1] = cand[s1].c[ci].pos;
                        s_cand_prev_peak[s1] = cand[s1].c[ci].peak;
                        s_cand_prev_valid[s1] = 1;
                    } else {
                        s_cand_prev_valid[s1] = 0;
                    }
                }
                s_prev = best;
                s_prev.x = meas_x;
                s_prev.y = meas_y;
                s_prev_valid = 1;
                best.x = s_out_x;
                best.y = s_out_y;
            }
        } else {
            s_pending_valid = 0;
            if (s_prev_valid && s_hold_count < SELECT_HOLD_FRAMES) {
                best = s_prev;
                best.x = s_out_x;
                best.y = s_out_y;
                best.confidence *= 0.5f;
                s_hold_count++;
            } else {
                s_prev_valid = 0;
                s_same_count = 0;
                s_pending_valid = 0;
                s_hold_count = 0;
                s_out_x = s_out_y = 0.0f;
                s_vel_x = s_vel_y = 0.0f;
                best.mode = 0;
                best.support_mask = 0;
                best.n_support = 0;
                best.hq_support = 0;
                best.confidence = 0.0f;
                for (s1 = 0; s1 < 4; s1++) best.line_idx[s1] = 0xFF;
                for (s1 = 0; s1 < 8; s1++) best.sel_idx[s1] = 0xFF;
                memset(s_axis_prev_valid, 0, sizeof(s_axis_prev_valid));
                memset(s_cand_prev_valid, 0, sizeof(s_cand_prev_valid));
                memset(s_pair_state, 0, sizeof(s_pair_state));
            }
        }
    }

    /* 保持上一帧结果时，当前帧候选可能已经变化；把失效的索引清掉，
       避免上位机看到指向不存在候选的 sel_idx。 */
    for (s1 = 0; s1 < 8; s1++) {
        uint8_t ci = best.sel_idx[s1];
        if (ci != 0xFF && ci >= cand[s1].n) best.sel_idx[s1] = 0xFF;
    }

    best.x_line = best.x;
    best.y_line = best.y;
    best.line_valid = (best.mode >= 2);
    best.cross_lines = best.n_support;
    best.cross_source = best.line_valid ? 1u : 0u;
    best.cross_conf = best.confidence;
    best.cross_diff = best.residual;
    *out = best;
}
