/* 交点范围保护、异常确认和 One Euro 平滑；不修改质心或求交直线。 */
#include "kalman.h"
#include "config.h"
#include "runtime_params.h"
#include <math.h>
#include <string.h>

static float alpha_lp(float cutoff_hz, float dt)
{
    return dt / (dt + 1.0f / (6.28318531f * cutoff_hz));
}
static float clamp(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}
static void output(Kalman *k, Position *out)
{
    out->x = k->x;
    out->y = k->y;
    out->valid = 1;
}
static void reject(Kalman *k, Position *out, float elapsed)
{
    if (k->lost_cnt < UINT16_MAX) k->lost_cnt++;
    k->missing_s += elapsed;
    k->held = 0;
    /* 短暂保持最后一个真实结果，不进行速度外推。 */
    if (k->init && k->missing_s <= KALMAN_HOLD_SECONDS) {
        k->held = 1;
        output(k, out);
    } else {
        k->init = 0;
        k->pending = 0;
        out->x = out->y = 0.0f;
        out->valid = 0;
    }
}
void kalman_init(Kalman *k, float dt)
{
    memset(k, 0, sizeof(*k));
    k->dt = dt;
}
void kalman_update(Kalman *k, Position *z, Position *out)
{
    float elapsed = (isfinite(k->dt) && k->dt > 0.0f) ? k->dt : 0.06f;
    float dt = clamp(elapsed, 0.002f, 0.25f);
    k->held = 0;
    /* 5mm内的小幅越界贴边；大幅越界拒绝，不伪造角落坐标。 */
    if (!z->valid || !isfinite(z->x) || !isfinite(z->y) ||
        z->x < -PRODUCT_SCREEN_MARGIN_MM ||
        z->x > SCREEN_W + PRODUCT_SCREEN_MARGIN_MM ||
        z->y < -PRODUCT_SCREEN_MARGIN_MM ||
        z->y > SCREEN_H + PRODUCT_SCREEN_MARGIN_MM) {
        k->pending = 0;
        reject(k, out, elapsed);
        return;
    }
    float zx = clamp(z->x, 0.0f, SCREEN_W);
    float zy = clamp(z->y, 0.0f, SCREEN_H);
    if (!k->init) {
        k->x = k->last_zx = zx;
        k->y = k->last_zy = zy;
        k->dx = k->dy = 0.0f;
        k->missing_s = 0.0f;
        k->lost_cnt = 0;
        k->pending = 0;
        k->init = 1;
        output(k, out);
        return;
    }
    float ux = zx - k->last_zx;
    float uy = zy - k->last_zy;
    float distance = hypotf(ux, uy);
    float speed = hypotf(k->dx, k->dy);
    float jump_cap = runtime_params_get()->position_jump_mm;
    if (!isfinite(jump_cap) || jump_cap < KALMAN_GATE_MIN_MM)
        jump_cap = KALMAN_GATE_MIN_MM;
    float gate = clamp(4.0f + 3.0f * speed * dt,
                       KALMAN_GATE_MIN_MM, jump_cap);
    if (distance > gate) {
        uint8_t confirmed = 0;
        if (k->pending) {
            float vx = k->pending_x - k->last_zx;
            float vy = k->pending_y - k->last_zy;
            float previous_distance = hypotf(vx, vy);
            /* 新位置连续一致，或快速移动方向连续一致，才接受。 */
            confirmed = hypotf(zx - k->pending_x, zy - k->pending_y)
                            <= KALMAN_PENDING_MM;
            if (!confirmed && previous_distance > gate &&
                distance >= previous_distance * 0.25f &&
                distance <= previous_distance * 3.0f &&
                hypotf(zx - k->pending_x, zy - k->pending_y) <=
                    KALMAN_MAX_SPEED_MMPS * (dt + k->missing_s) &&
                ux * vx + uy * vy >= 0.0f)
                confirmed = 1;
        }
        if (!confirmed) {
            k->pending_x = zx;
            k->pending_y = zy;
            k->pending = 1;
            reject(k, out, elapsed);
            return;
        }
    }
    k->pending = 0;
    k->lost_cnt = 0;
    float observation_dt = clamp(dt + k->missing_s, 0.002f, 0.5f);
    k->missing_s = 0.0f;
    float ad = alpha_lp(KALMAN_D_CUTOFF, observation_dt);
    k->dx += ad * (ux / observation_dt - k->dx);
    k->dy += ad * (uy / observation_dt - k->dy);
    k->last_zx = zx;
    k->last_zy = zy;
    float mx = zx - k->x;
    float my = zy - k->y;
    if (runtime_params_get()->position_filter_enable) {
        /* 静止小死区；慢移动累积超过死区后仍能跟随。 */
        if (hypotf(mx, my) <= KALMAN_STILL_MM) {
            output(k, out);
            return;
        }
        mx *= alpha_lp(KALMAN_MIN_CUTOFF + KALMAN_BETA * fabsf(k->dx), dt);
        my *= alpha_lp(KALMAN_MIN_CUTOFF + KALMAN_BETA * fabsf(k->dy), dt);
    }
    /* 确认后连续追踪，不直接重置到大位移的新位置。 */
    float step = hypotf(mx, my);
    float maximum_step = KALMAN_MAX_SPEED_MMPS * dt;
    if (step > maximum_step) {
        mx *= maximum_step / step;
        my *= maximum_step / step;
    }
    k->x = clamp(k->x + mx, 0.0f, SCREEN_W);
    k->y = clamp(k->y + my, 0.0f, SCREEN_H);
    output(k, out);
}
