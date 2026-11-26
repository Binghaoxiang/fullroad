// position.c
//
// 定位：上下多点自适应拟合 + 左右近侧内外连线 + 两线交点。
//
//   1) 同侧内外边构造一条“视线”：
//        top:    (x_up,377)            -> (x_in_up,352)
//        bottom: (x_down,-25)          -> (x_in_down,0)
//        left:   (-25,y_left)          -> (0,y_in_left)
//        right:  (624,y_right)         -> (599,y_in_right)
//      上下4个质心信号接近时用4点求加权X，3个接近时用3点；
//      竖向光路线始终强制为 x=常数。不足3个时按信号质量选上侧或
//      下侧内外两点求加权X。左右仍按信号质量
//      选择左侧或右侧内外两点。最终两条方向线求交。
//      缺少任一方向、两线近平行或方向不可信 -> 不输出二维交点。
//   2) 产品坐标只来自当前有效交点。当前交点无效就立即输出无效，
//      不保持旧点，也不回退到轴向融合点。
#include "position.h"
#include "centroid.h"
#include "config.h"
#include "board_map.h"
#include "runtime_params.h"
#include "bsp_uart.h"
#include <math.h>
#include <string.h>

static PositionDebug g_dbg;

/* ==================== 内外边装配偏置在线自学习 ==================== */
#if POSITION_BIAS_ENABLE
typedef struct {
    float bias;
    uint8_t init;
    uint8_t n;
} PairBias;

static PairBias pb_top, pb_bot, pb_lft, pb_rgt;

static void pair_bias_update(PairBias *p, float diff, uint8_t valid)
{
    if (!valid || !isfinite(diff) || fabsf(diff) > POSITION_BIAS_MAX_MM) return;

    if (!p->init) {
        p->bias = diff;
        p->init = 1;
        p->n = 1;
        return;
    }
    if (p->n < POSITION_BIAS_INIT_N) {
        p->n++;
        p->bias += (diff - p->bias) / (float)p->n;
        return;
    }

    float e = diff - p->bias;
    if (fabsf(e) > POSITION_BIAS_RESET_MM) {
        p->bias = diff;
        p->n = 1;
    } else if (fabsf(e) <= POSITION_BIAS_GATE_MM) {
        p->bias += POSITION_BIAS_ALPHA * e;
    }
}
#endif

/* ==================== 连线方向慢状态 ==================== */
typedef struct {
    float d;       // 平滑后的内-外差值(mm)
    float var;     // 新息方差 EMA(mm²)
    uint8_t init;
} DirState;

static DirState ds_top, ds_bot, ds_lft, ds_rgt;

static float dir_update(DirState *s, float raw)
{
    if (!isfinite(raw) || fabsf(raw) > LINE_DIR_MAX_MM) {
        s->init = 0;
        return s->d;
    }
    if (!s->init) {
        s->d = raw;
        s->var = 0.0f;
        s->init = 1;
        return s->d;
    }

    float e = raw - s->d;
    if (fabsf(e) > LINE_DIR_RESET_MM) {
        s->d = raw;
        s->var = 0.0f;
    } else {
        s->d = LINE_DIR_EMA * s->d + (1.0f - LINE_DIR_EMA) * raw;
        s->var += DIR_VAR_ALPHA * (e * e - s->var);
        if (s->var < 0.0f) s->var = 0.0f;
    }
    return s->d;
}

static float dir_conf(const DirState *s)
{
    float c = 1.0f / (1.0f + s->var / POSITION_CROSS_WEIGHT_DIR_REF);
    if (c < 0.05f) c = 0.05f;
    return c;
}

/* ==================== 单边质心 → 沿边屏幕坐标 ==================== */
static float edge_to_coord(Centroid *c, uint8_t edge)
{
    if (!c->valid) return -1.0f;
    return runtime_params_map_edge(edge, c->position);
}

/* ==================== 相邻边选线 + 两线交点 ==================== */
typedef struct {
    float A, B, C, w;
    float dir_conf;
    uint8_t side;
} LineObs;

static uint8_t make_line(float x1, float y1, float x2, float y2,
                         float w, float dir_conf, uint8_t side, LineObs *L)
{
    float A = y2 - y1;
    float B = x1 - x2;
    float C = x2 * y1 - x1 * y2;
    float n = sqrtf(A * A + B * B);
    if (n < 1e-6f || w <= 0.0f) return 0;
    L->A = A / n;
    L->B = B / n;
    L->C = C / n;
    L->w = w;
    L->dir_conf = dir_conf;
    L->side = side;
    return 1;
}

/* 上下4点的相近信号判据。相对于当前最强点达到60%，认为可共同拟合。
 * 选中3/4点后按峰值加权求共同X，强制构造 x=常数 的竖直光路线。
 * 这一步有意不拟合斜率，避免质心小偏差在远处被斜率放大。 */
#define VERTICAL_MULTI_REL_MIN  0.60f

typedef struct {
    float x;
    float y;
    float q;
    uint8_t bit;
} VerticalPoint;

static uint8_t fit_vertical_points(const VerticalPoint *p, uint8_t n,
                                   uint8_t selected_mask, LineObs *L)
{
    float sw = 0.0f, sx = 0.0f;
    uint8_t used = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!(selected_mask & p[i].bit)) continue;
        float w = p[i].q;
        if (!isfinite(p[i].x) || !isfinite(p[i].y) || w <= 0.0f) continue;
        sw += w;
        sx += w * p[i].x;
        used++;
    }
    if (used < 3 || sw <= 1e-9f) return 0;

    float xbar = sx / sw;
    float y1 = -IN_DOWN_OFFSET_Y;
    float y2 = SCREEN_H + IN_UP_OFFSET_Y;
    return make_line(xbar, y1, xbar, y2,
                     sw / (float)used, 1.0f, 0, L);
}

/* 同侧上下两个质心回退时也禁止直接连出斜率。两点只用于求共同X，
 * 输出仍是绝对竖线。 */
static uint8_t make_vertical_pair(float x_outer, float q_outer,
                                  float x_inner, float q_inner,
                                  uint8_t side, LineObs *L, float *x_out)
{
    float sw = q_outer + q_inner;
    if (!isfinite(x_outer) || !isfinite(x_inner) ||
        q_outer <= 0.0f || q_inner <= 0.0f || sw <= 1e-9f) return 0;
    float x = (q_outer * x_outer + q_inner * x_inner) / sw;
    if (x_out) *x_out = x;
    return make_line(x, -IN_DOWN_OFFSET_Y,
                     x, SCREEN_H + IN_UP_OFFSET_Y,
                     sqrtf(q_outer * q_inner), 1.0f, side, L);
}

/* 应急竖线：没有任何完整的上/下内外点对时，只要上下四边中仍有
 * 有效质心，就用全部当前有效点求共同X，避免X方向整条线消失。 */
static uint8_t make_vertical_available(const VerticalPoint *p, uint8_t n,
                                       uint8_t valid_mask, LineObs *L,
                                       float *x_out)
{
    float sw = 0.0f, sx = 0.0f;
    uint8_t used = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!(valid_mask & p[i].bit) || !isfinite(p[i].x)) continue;
        float w = (p[i].q > 0.0f) ? p[i].q : 1.0f;
        sw += w;
        sx += w * p[i].x;
        used++;
    }
    if (used == 0 || sw <= 1e-9f) return 0;
    float x = sx / sw;
    if (x_out) *x_out = x;
    return make_line(x, -IN_DOWN_OFFSET_Y,
                     x, SCREEN_H + IN_UP_OFFSET_Y,
                     sw / (float)used, 1.0f, 0, L);
}

typedef struct {
    float x;
    float y;
    float q;
    uint8_t valid;
} LateralPoint;

#define LATERAL_MAX_FULL_RISE_MM         30.0f

/* 左右同侧内外点对都不存在时的降级线：
 *   >=2个有效点：对所有有效左右质心做加权 y=a*x+b；
 *   仅1个有效点：以该质心Y生成水平线。
 * 因而左右四边只要还有任意有效质心，Y方向线就不会消失。 */
static uint8_t fit_lateral_points(const LateralPoint *p, uint8_t n,
                                  uint8_t side, LineObs *L)
{
    float sw = 0.0f, sx = 0.0f, sy = 0.0f;
    uint8_t used = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!p[i].valid || !isfinite(p[i].x) || !isfinite(p[i].y)) continue;
        float w = (p[i].q > 0.0f) ? p[i].q : 1.0f;
        sw += w;
        sx += w * p[i].x;
        sy += w * p[i].y;
        used++;
    }
    if (used == 0 || sw <= 1e-9f) return 0;

    float xbar = sx / sw;
    float ybar = sy / sw;
    float sxx = 0.0f, sxy = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        if (!p[i].valid || !isfinite(p[i].x) || !isfinite(p[i].y)) continue;
        float w = (p[i].q > 0.0f) ? p[i].q : 1.0f;
        float dx = p[i].x - xbar;
        sxx += w * dx * dx;
        sxy += w * dx * (p[i].y - ybar);
    }

    float x1 = -IN_LEFT_OFFSET_X;
    float x2 = SCREEN_W + IN_RIGHT_OFFSET_X;
    float y1 = ybar, y2 = ybar;
    if (used >= 2 && sxx > 1e-6f) {
        float a = sxy / sxx;
        /* 所有横向线统一限制整块外框宽度上的Y变化。尤其是同侧内外
         * 仅25mm的短基线，几毫米质心误差不能被无限外推成上百毫米。 */
        float full_span = x2 - x1;
        float a_max = LATERAL_MAX_FULL_RISE_MM / full_span;
        if (a > a_max) a = a_max;
        if (a < -a_max) a = -a_max;
        float b = ybar - a * xbar;
        y1 = a * x1 + b;
        y2 = a * x2 + b;
    }
    return make_line(x1, y1, x2, y2,
                     sw / (float)used, 1.0f, side, L);
}

/* 从左右4个质心中寻找稳健共识：
 * - 4点总跨度<=门限：保留4点；
 * - 否则选择跨度更小且<=门限的连续3点，另1点判为离群；
 * - 找不到3点共识：返回0，调用方回退到近侧内外两点。
 * mask位顺序与LateralPoint数组一致。 */
static uint8_t lateral_consensus_mask(const LateralPoint *p, uint8_t n,
                                      float max_span, int8_t *rejected)
{
    uint8_t idx[4];
    float val[4];
    uint8_t count = 0;
    if (rejected) *rejected = -1;
    for (uint8_t i = 0; i < n && i < 4; i++) {
        if (!p[i].valid || !isfinite(p[i].y)) continue;
        idx[count] = i;
        val[count] = p[i].y;
        count++;
    }
    if (count < 3) return 0;

    for (uint8_t i = 1; i < count; i++) {
        float vy = val[i];
        uint8_t vi = idx[i];
        uint8_t j = i;
        while (j > 0 && val[j - 1] > vy) {
            val[j] = val[j - 1];
            idx[j] = idx[j - 1];
            j--;
        }
        val[j] = vy;
        idx[j] = vi;
    }

    if (val[count - 1] - val[0] <= max_span) {
        uint8_t mask = 0;
        for (uint8_t i = 0; i < count; i++) mask |= (uint8_t)(1u << idx[i]);
        return mask;
    }
    if (count != 4) return 0;

    float low_span = val[2] - val[0];
    float high_span = val[3] - val[1];
    uint8_t start = (low_span <= high_span) ? 0u : 1u;
    float best_span = (start == 0u) ? low_span : high_span;
    if (best_span > max_span) return 0;

    uint8_t mask = 0;
    for (uint8_t i = start; i < start + 3u; i++)
        mask |= (uint8_t)(1u << idx[i]);
    if (rejected) *rejected = (int8_t)idx[(start == 0u) ? 3u : 0u];
    return mask;
}

/* 每个方向只选靠近光斑、成对信号更可靠的一侧。
 * top/bottom 二选一用于 X 光路，left/right 二选一用于 Y 光路。
 * 中部两侧质量接近时保持当前选择；挑战侧连续更强才切换，避免线闪烁。 */
#define SIDE_SWITCH_RATIO   1.35f
#define SIDE_SWITCH_FRAMES  3u
#define LATERAL_FOUR_POINT_MAX_DIFF_MM  30.0f

typedef struct {
    uint8_t valid;
    uint8_t selected_side;
    uint8_t pending_side;
    uint8_t pending_n;
} SideSelector;

static SideSelector s_tb_selector, s_lr_selector;

static int find_side(const LineObs *lines, uint8_t n, uint8_t side)
{
    for (uint8_t i = 0; i < n; i++)
        if (lines[i].side == side) return (int)i;
    return -1;
}

static int select_family_line(const LineObs *lines, uint8_t n,
                              uint8_t side_a, uint8_t side_b,
                              SideSelector *s)
{
    int ia = find_side(lines, n, side_a);
    int ib = find_side(lines, n, side_b);
    if (ia < 0 && ib < 0) {
        s->valid = 0;
        s->pending_n = 0;
        return -1;
    }

    if (ia < 0 || ib < 0) {
        int only = (ia >= 0) ? ia : ib;
        s->selected_side = lines[only].side;
        s->valid = 1;
        s->pending_n = 0;
        return only;
    }

    int best = (lines[ia].w >= lines[ib].w) ? ia : ib;
    if (!s->valid ||
        (s->selected_side != side_a && s->selected_side != side_b)) {
        s->selected_side = lines[best].side;
        s->valid = 1;
        s->pending_n = 0;
        return best;
    }

    int current = (s->selected_side == side_a) ? ia : ib;
    if (best == current ||
        lines[best].w < lines[current].w * SIDE_SWITCH_RATIO) {
        s->pending_n = 0;
        return current;
    }

    if (s->pending_n == 0 || s->pending_side != lines[best].side) {
        s->pending_side = lines[best].side;
        s->pending_n = 1;
    } else if (s->pending_n < SIDE_SWITCH_FRAMES) {
        s->pending_n++;
    }
    if (s->pending_n >= SIDE_SWITCH_FRAMES) {
        s->selected_side = lines[best].side;
        s->pending_n = 0;
        return best;
    }
    return current;
}

/* 把固件实际参与求交的无限直线裁到外层探测器框，供UI直接显示。 */
static void debug_store_line(uint8_t side, const LineObs *L,
                             float fallback_x1, float fallback_y1,
                             float fallback_x2, float fallback_y2)
{
    float px[4], py[4];
    uint8_t n = 0;
#define ADD_POINT(xx, yy) do { \
    float ax = (xx), ay = (yy); \
    if (ax >= -IN_LEFT_OFFSET_X - 1e-3f && \
        ax <= SCREEN_W + IN_RIGHT_OFFSET_X + 1e-3f && \
        ay >= -IN_DOWN_OFFSET_Y - 1e-3f && \
        ay <= SCREEN_H + IN_UP_OFFSET_Y + 1e-3f) { \
        uint8_t duplicate = 0; \
        for (uint8_t q = 0; q < n; q++) \
            if (fabsf(px[q] - ax) < 1e-3f && fabsf(py[q] - ay) < 1e-3f) duplicate = 1; \
        if (!duplicate && n < 4) { px[n] = ax; py[n] = ay; n++; } \
    } \
} while (0)
    if (fabsf(L->B) > 1e-6f) {
        ADD_POINT(-IN_LEFT_OFFSET_X, -(L->A * (-IN_LEFT_OFFSET_X) + L->C) / L->B);
        ADD_POINT(SCREEN_W + IN_RIGHT_OFFSET_X,
                  -(L->A * (SCREEN_W + IN_RIGHT_OFFSET_X) + L->C) / L->B);
    }
    if (fabsf(L->A) > 1e-6f) {
        ADD_POINT(-(L->B * (-IN_DOWN_OFFSET_Y) + L->C) / L->A,
                  -IN_DOWN_OFFSET_Y);
        ADD_POINT(-(L->B * (SCREEN_H + IN_UP_OFFSET_Y) + L->C) / L->A,
                  SCREEN_H + IN_UP_OFFSET_Y);
    }
#undef ADD_POINT
    if (n >= 2) {
        g_dbg.line_x1[side] = px[0]; g_dbg.line_y1[side] = py[0];
        g_dbg.line_x2[side] = px[1]; g_dbg.line_y2[side] = py[1];
    } else {
        g_dbg.line_x1[side] = fallback_x1; g_dbg.line_y1[side] = fallback_y1;
        g_dbg.line_x2[side] = fallback_x2; g_dbg.line_y2[side] = fallback_y2;
    }
    g_dbg.line_valid_mask |= (uint8_t)(1u << side);
}

static uint8_t solve_cross(const LineObs *L, uint8_t n,
                           float *x, float *y,
                           float dir_conf_min, float *conf_out)
{
    if (n < POSITION_CROSS_MIN_LINES) return 0;

    float Sxx = 0.0f, Sxy = 0.0f, Syy = 0.0f, Sxc = 0.0f, Syc = 0.0f;
    float wsum = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float w = L[i].w;
        Sxx += w * L[i].A * L[i].A;
        Sxy += w * L[i].A * L[i].B;
        Syy += w * L[i].B * L[i].B;
        Sxc += w * L[i].A * L[i].C;
        Syc += w * L[i].B * L[i].C;
        wsum += w;
    }

    float tr = Sxx + Syy;
    if (tr <= 1e-9f) return 0;

    float det = Sxx * Syy - Sxy * Sxy;
    float det_ratio = det / (tr * tr);
    if (det_ratio < POSITION_CROSS_MIN_DET_RATIO) return 0;

    float xx = (-Sxc * Syy + Syc * Sxy) / det;
    float yy = (-Sxx * Syc + Sxy * Sxc) / det;

    /* 残差（n=2 时必然≈0；n>2 用来发现线间不一致） */
    float res2 = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float r = L[i].A * xx + L[i].B * yy + L[i].C;
        res2 += L[i].w * r * r;
    }
    float rms = (wsum > 0.0f) ? sqrtf(res2 / wsum) : 0.0f;

    float angle_conf = sqrtf(det_ratio / 0.25f);
    if (angle_conf > 1.0f) angle_conf = 1.0f;
    float res_conf = 1.0f / (1.0f + (rms / POSITION_CROSS_RESIDUAL_MAX_MM) *
                                    (rms / POSITION_CROSS_RESIDUAL_MAX_MM));
    float conf = angle_conf * dir_conf_min * res_conf;
    if (conf > 1.0f) conf = 1.0f;
    if (conf < 0.0f) conf = 0.0f;

    *x = xx;
    *y = yy;
    if (conf_out) *conf_out = conf;
    return 1;
}

/* ==================== 主定位 ==================== */
void position_compute(Centroids *c, Position *p, float dt)
{
    (void)dt;
    memset(&g_dbg, 0, sizeof(g_dbg));
    g_dbg.cross_diff = -1.0f;

    // 1) 八个质心 → 屏幕坐标(mm)
    float x_up       = edge_to_coord(&c->up,       EDGE_UP);
    float x_down     = edge_to_coord(&c->down,     EDGE_DOWN);
    float x_in_up    = edge_to_coord(&c->in_up,    EDGE_IN_UP);
    float x_in_down  = edge_to_coord(&c->in_down,  EDGE_IN_DOWN);

    float y_left     = edge_to_coord(&c->left,     EDGE_LEFT);
    float y_right    = edge_to_coord(&c->right,    EDGE_RIGHT);
    float y_in_left  = edge_to_coord(&c->in_left,  EDGE_IN_LEFT);
    float y_in_right = edge_to_coord(&c->in_right, EDGE_IN_RIGHT);

    // 2) 内外边装配偏置在线学习，修正内圈坐标
    uint8_t top_ok = (c->up.valid && c->in_up.valid &&
                      isfinite(x_up) && isfinite(x_in_up) &&
                      c->up.peak >= POSITION_BIAS_MIN_PEAK &&
                      c->in_up.peak >= POSITION_BIAS_MIN_PEAK);
    uint8_t bot_ok = (c->down.valid && c->in_down.valid &&
                      isfinite(x_down) && isfinite(x_in_down) &&
                      c->down.peak >= POSITION_BIAS_MIN_PEAK &&
                      c->in_down.peak >= POSITION_BIAS_MIN_PEAK);
    uint8_t lft_ok = (c->left.valid && c->in_left.valid &&
                      isfinite(y_left) && isfinite(y_in_left) &&
                      c->left.peak >= POSITION_BIAS_MIN_PEAK &&
                      c->in_left.peak >= POSITION_BIAS_MIN_PEAK);
    uint8_t rgt_ok = (c->right.valid && c->in_right.valid &&
                      isfinite(y_right) && isfinite(y_in_right) &&
                      c->right.peak >= POSITION_BIAS_MIN_PEAK &&
                      c->in_right.peak >= POSITION_BIAS_MIN_PEAK);

    float x_in_up_corr   = x_in_up;
    float x_in_down_corr = x_in_down;
    float y_in_left_corr = y_in_left;
    float y_in_right_corr= y_in_right;
#if POSITION_BIAS_ENABLE
    pair_bias_update(&pb_top, x_in_up - x_up, top_ok);
    pair_bias_update(&pb_bot, x_in_down - x_down, bot_ok);
    pair_bias_update(&pb_lft, y_in_left - y_left, lft_ok);
    pair_bias_update(&pb_rgt, y_in_right - y_right, rgt_ok);
    if (top_ok && pb_top.init)   x_in_up_corr    = x_in_up - pb_top.bias;
    if (bot_ok && pb_bot.init)   x_in_down_corr  = x_in_down - pb_bot.bias;
    if (lft_ok && pb_lft.init)   y_in_left_corr  = y_in_left - pb_lft.bias;
    if (rgt_ok && pb_rgt.init)   y_in_right_corr = y_in_right - pb_rgt.bias;
#endif

    // 3) 构造竖向光路线：优先使用信号相近的3/4个上下质心。
    LineObs lines[6];
    uint8_t ln = 0;
    int vertical_idx = -1;
    uint8_t vertical_point_mask = 0;

    VerticalPoint vp[4] = {
        { x_up, SCREEN_H + IN_UP_OFFSET_Y, c->up.peak, 0x01u },
        { x_in_up_corr - POSITION_INNER_BIAS_TOP, SCREEN_H,
          c->in_up.peak, 0x02u },
        { x_in_down_corr - POSITION_INNER_BIAS_BOT, 0.0f,
          c->in_down.peak, 0x04u },
        { x_down, -IN_DOWN_OFFSET_Y, c->down.peak, 0x08u },
    };
    uint8_t vv[4] = {
        (uint8_t)(c->up.valid && isfinite(x_up) &&
                  c->up.peak >= POSITION_BIAS_MIN_PEAK),
        (uint8_t)(c->in_up.valid && isfinite(x_in_up_corr) &&
                  c->in_up.peak >= POSITION_BIAS_MIN_PEAK),
        (uint8_t)(c->in_down.valid && isfinite(x_in_down_corr) &&
                  c->in_down.peak >= POSITION_BIAS_MIN_PEAK),
        (uint8_t)(c->down.valid && isfinite(x_down) &&
                  c->down.peak >= POSITION_BIAS_MIN_PEAK),
    };
    uint8_t vertical_valid_mask = 0;
    for (uint8_t i = 0; i < 4; i++)
        if (vv[i]) vertical_valid_mask |= vp[i].bit;
    float vmax_q = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
        if (vv[i] && vp[i].q > vmax_q) vmax_q = vp[i].q;
    if (vmax_q > 0.0f) {
        for (uint8_t i = 0; i < 4; i++) {
            if (vv[i] && vp[i].q >= vmax_q * VERTICAL_MULTI_REL_MIN)
                vertical_point_mask |= vp[i].bit;
        }
    }
    uint8_t vertical_point_count =
        (uint8_t)__builtin_popcount((unsigned)vertical_point_mask);
    if (vertical_point_count >= 3 &&
        fit_vertical_points(vp, 4, vertical_point_mask, &lines[ln])) {
        vertical_idx = (int)ln;
        float fy1 = -IN_DOWN_OFFSET_Y;
        float fy2 = SCREEN_H + IN_UP_OFFSET_Y;
        float fx1 = (fabsf(lines[ln].A) > 1e-6f) ?
            -(lines[ln].B * fy1 + lines[ln].C) / lines[ln].A : vp[0].x;
        float fx2 = (fabsf(lines[ln].A) > 1e-6f) ?
            -(lines[ln].B * fy2 + lines[ln].C) / lines[ln].A : vp[0].x;
        debug_store_line(0, &lines[ln],
                         fx1, fy1, fx2, fy2);
        ln++;
    } else {
        /* 不足3个相近点时回退到靠近激光的一侧内外两点。 */
        vertical_point_mask = 0;
    }

    if (vertical_idx < 0 && top_ok) {
        float inner_x = x_in_up_corr - POSITION_INNER_BIAS_TOP;
        float raw_d = inner_x - x_up;
        (void)dir_update(&ds_top, raw_d);
        float x_vertical = 0.0f;
        if (make_vertical_pair(x_up, c->up.peak,
                               inner_x, c->in_up.peak,
                               0, &lines[ln], &x_vertical)) {
            debug_store_line(0, &lines[ln],
                             x_vertical, -IN_DOWN_OFFSET_Y,
                             x_vertical, SCREEN_H + IN_UP_OFFSET_Y);
            ln++;
        }
    }

    if (vertical_idx < 0 && bot_ok) {
        float inner_x = x_in_down_corr - POSITION_INNER_BIAS_BOT;
        float raw_d = inner_x - x_down;
        (void)dir_update(&ds_bot, raw_d);
        float x_vertical = 0.0f;
        if (make_vertical_pair(x_down, c->down.peak,
                               inner_x, c->in_down.peak,
                               1, &lines[ln], &x_vertical)) {
            debug_store_line(1, &lines[ln],
                             x_vertical, -IN_DOWN_OFFSET_Y,
                             x_vertical, SCREEN_H + IN_UP_OFFSET_Y);
            ln++;
        }
    }

    if (lft_ok) {
        float inner_y = y_in_left_corr - POSITION_INNER_BIAS_LFT;
        float raw_d = inner_y - y_left;
        (void)dir_update(&ds_lft, raw_d);
        float dc = ds_lft.init ? dir_conf(&ds_lft) : 1.0f;
        float w = sqrtf(c->left.peak * c->in_left.peak);
        if (make_line(-IN_LEFT_OFFSET_X, y_left, 0.0f, inner_y,
                      w, dc, 2, &lines[ln])) {
            debug_store_line(2, &lines[ln], -IN_LEFT_OFFSET_X, y_left, 0.0f, inner_y);
            ln++;
        }
    } else ds_lft.init = 0;

    if (rgt_ok) {
        float inner_y = y_in_right_corr - POSITION_INNER_BIAS_RGT;
        float raw_d = inner_y - y_right;
        (void)dir_update(&ds_rgt, raw_d);
        float dc = ds_rgt.init ? dir_conf(&ds_rgt) : 1.0f;
        float w = sqrtf(c->right.peak * c->in_right.peak);
        if (make_line(SCREEN_W + IN_RIGHT_OFFSET_X, y_right, SCREEN_W, inner_y,
                      w, dc, 3, &lines[ln])) {
            debug_store_line(3, &lines[ln], SCREEN_W + IN_RIGHT_OFFSET_X, y_right, SCREEN_W, inner_y);
            ln++;
        }
    } else ds_rgt.init = 0;

    // 4) 竖向使用3/4点拟合线或近侧两点线；横向仍按左/右近侧选择。
    LineObs selected[2];
    uint8_t selected_n = 0;
    int tb = vertical_idx;
    if (tb < 0) {
        tb = select_family_line(lines, ln, 0, 1, &s_tb_selector);
        if (tb >= 0)
            vertical_point_mask = (lines[tb].side == 0) ? 0x03u : 0x0Cu;
    }
    if (tb < 0 && vertical_valid_mask != 0) {
        float x_vertical = 0.0f;
        if (make_vertical_available(vp, 4, vertical_valid_mask,
                                    &lines[ln], &x_vertical)) {
            tb = (int)ln;
            vertical_point_mask = vertical_valid_mask;
            debug_store_line(0, &lines[ln],
                             x_vertical, -IN_DOWN_OFFSET_Y,
                             x_vertical, SCREEN_H + IN_UP_OFFSET_Y);
            ln++;
        }
    }

    LateralPoint lp[4] = {
        { -IN_LEFT_OFFSET_X, y_left, c->left.peak,
          (uint8_t)(c->left.valid && isfinite(y_left)) },
        { 0.0f, y_in_left_corr - POSITION_INNER_BIAS_LFT,
          c->in_left.peak,
          (uint8_t)(c->in_left.valid && isfinite(y_in_left_corr)) },
        { SCREEN_W, y_in_right_corr - POSITION_INNER_BIAS_RGT,
          c->in_right.peak,
          (uint8_t)(c->in_right.valid && isfinite(y_in_right_corr)) },
        { SCREEN_W + IN_RIGHT_OFFSET_X, y_right, c->right.peak,
          (uint8_t)(c->right.valid && isfinite(y_right)) },
    };

    int lr = select_family_line(lines, ln, 2, 3, &s_lr_selector);
    uint8_t lateral_mode = 0; /* 0=主侧两点 1=缺点降级 2=四点 3=三点 4=短基线限幅 */
    int8_t lateral_rejected = -1;

    /* 稳健一致性现在直接以“当前质心有效”作为唯一入口，不再依赖
     * lft_ok/rgt_ok、成对峰值门限或旧选边状态。只要3/4个左右质心
     * 能形成30mm内的共识，就必须优先使用该共识线。旧选边器只负责
     * 标明主侧；没有完整点对时按两侧有效信号总量决定主侧。 */
    uint8_t consensus = lateral_consensus_mask(
        lp, 4, LATERAL_FOUR_POINT_MAX_DIFF_MM, &lateral_rejected);
    if (consensus != 0) {
        LateralPoint fitp[4];
        memcpy(fitp, lp, sizeof(fitp));
        uint8_t fit_count = 0;
        float left_q = 0.0f, right_q = 0.0f;
        for (uint8_t i = 0; i < 4; i++) {
            if (!(consensus & (uint8_t)(1u << i))) {
                fitp[i].valid = 0;
            } else {
                fit_count++;
                float q = (fitp[i].q > 0.0f) ? fitp[i].q : 1.0f;
                if (i < 2u) left_q += q;
                else right_q += q;
            }
        }
        uint8_t primary_side = (lr >= 0) ? lines[lr].side :
                               ((right_q > left_q) ? 3u : 2u);
        if (fit_lateral_points(fitp, 4, primary_side, &lines[ln])) {
            lr = (int)ln;
            lateral_mode = (fit_count == 4u) ? 2u : 3u;
            float fx1 = -IN_LEFT_OFFSET_X;
            float fx2 = SCREEN_W + IN_RIGHT_OFFSET_X;
            float fy1 = -(lines[ln].A * fx1 + lines[ln].C) / lines[ln].B;
            float fy2 = -(lines[ln].A * fx2 + lines[ln].C) / lines[ln].B;
            debug_store_line(primary_side, &lines[ln],
                             fx1, fy1, fx2, fy2);
            ln++;
        }
    }

    /* 没有3/4点共识但存在靠近激光侧的完整内外点对时，不能直接使用
     * make_line生成的原始25mm短基线斜率。用同一对质心重新拟合并执行
     * 全宽30mm方向限幅；该侧的加权中心仍是Y锚点。 */
    if (consensus == 0 && lr >= 0) {
        uint8_t primary_side = lines[lr].side;
        LateralPoint pairp[4];
        memset(pairp, 0, sizeof(pairp));
        if (primary_side == 2u) {
            pairp[0] = lp[0];
            pairp[1] = lp[1];
        } else {
            pairp[2] = lp[2];
            pairp[3] = lp[3];
        }
        if (fit_lateral_points(pairp, 4, primary_side, &lines[ln])) {
            lr = (int)ln;
            lateral_mode = 4;
            float fx1 = -IN_LEFT_OFFSET_X;
            float fx2 = SCREEN_W + IN_RIGHT_OFFSET_X;
            float fy1 = -(lines[ln].A * fx1 + lines[ln].C) / lines[ln].B;
            float fy2 = -(lines[ln].A * fx2 + lines[ln].C) / lines[ln].B;
            debug_store_line(primary_side, &lines[ln], fx1, fy1, fx2, fy2);
            ln++;
        }
    }

    if (lr < 0) {
        if (fit_lateral_points(lp, 4, 2, &lines[ln])) {
            lr = (int)ln;
            lateral_mode = 1;
            float fx1 = -IN_LEFT_OFFSET_X;
            float fx2 = SCREEN_W + IN_RIGHT_OFFSET_X;
            float fy1 = (fabsf(lines[ln].B) > 1e-6f) ?
                -(lines[ln].A * fx1 + lines[ln].C) / lines[ln].B : 0.0f;
            float fy2 = (fabsf(lines[ln].B) > 1e-6f) ?
                -(lines[ln].A * fx2 + lines[ln].C) / lines[ln].B : 0.0f;
            debug_store_line(2, &lines[ln],
                             fx1, fy1, fx2, fy2);
            ln++;
        }
    }
    if (tb >= 0) selected[selected_n++] = lines[tb];
    if (lr >= 0) selected[selected_n++] = lines[lr];

    /* UI 只显示实际参与求交的两条线，不再显示未选中的候选线。 */
    g_dbg.line_valid_mask = 0;
    if (tb >= 0) g_dbg.line_valid_mask |= (uint8_t)(1u << lines[tb].side);
    if (lr >= 0) g_dbg.line_valid_mask |= (uint8_t)(1u << lines[lr].side);
    /* 高4位逐位标记本帧参与竖向拟合的点：外上、内上、内下、外下。 */
    g_dbg.line_valid_mask |= (uint8_t)(vertical_point_mask << 4);

    for (uint8_t i = 0; i < selected_n; i++) {
        /* 两条无限直线的交点与权重无关。统一权重，避免亮度差降低
         * 正交性置信度；亮度只负责上述选边。 */
        selected[i].w = 1.0f;
    }

    // 5) 被选中的两条线求交
    float x_cross = 0.0f, y_cross = 0.0f, cross_conf = 0.0f;
    uint8_t cross_solved = solve_cross(selected, selected_n, &x_cross, &y_cross,
                                       1.0f, &cross_conf);

    uint8_t cross_usable = 0;
    if (cross_solved && selected_n == 2 &&
        isfinite(x_cross) && isfinite(y_cross)) {
        if (cross_conf >= POSITION_CROSS_2LINE_MIN_CONF) cross_usable = 1;
    }

    // 6) 当前两线交点直接输出。不得在交点层保持旧点或做跳变确认；
    //    后级位置滤波器负责必要的轻量平滑。
    if (cross_usable) {
        p->x = x_cross;
        p->y = y_cross;
        p->valid = 1;
    } else {
        p->x = 0.0f;
        p->y = 0.0f;
        p->valid = 0;
    }

    // 8) 调试量
    g_dbg.x_line = cross_solved ? x_cross : 0.0f;
    g_dbg.y_line = cross_solved ? y_cross : 0.0f;
    g_dbg.have_x = cross_usable;
    g_dbg.have_y = cross_usable;
    g_dbg.x_cross_out = cross_usable ? x_cross : 0.0f;
    g_dbg.y_cross_out = cross_usable ? y_cross : 0.0f;
    g_dbg.cross_out_valid = cross_usable;
    g_dbg.cross_source = cross_usable ?
        ((lateral_mode == 1) ? 3u :
         ((lateral_mode == 2) ? 4u :
          ((lateral_mode == 3) ? 5u :
           ((lateral_mode == 4) ? 6u : 1u)))) : 0u;
    g_dbg.cross_lines = selected_n;
    g_dbg.cross_conf = cross_conf;
    /* 稳健3点模式下传出被剔除的左右质心编号：
     * 0=外左 1=内左 2=内右 3=外右；其他模式为-1。 */
    g_dbg.cross_diff = (lateral_mode == 3 && lateral_rejected >= 0) ?
        (float)lateral_rejected : -1.0f;
}

const PositionDebug *position_get_debug(void)
{
    return &g_dbg;
}
