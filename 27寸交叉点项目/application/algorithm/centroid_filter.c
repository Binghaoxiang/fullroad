/* 每边质心稳定：同一方向共享运动增益，不强制各边坐标相等。
 * 无固定三帧中值延迟；不外推丢失的质心；标定映射仍使用本边真实质心。 */
#include "centroid_filter.h"
#include "config.h"
#include <math.h>
#include <string.h>

typedef struct {
    float pos, last_raw, dx;
    uint8_t init;
} CentroidFilter;
static CentroidFilter cf[8];
static float alpha_lp(float cutoff, float dt)
{
    return dt / (dt + 1.0f / (6.28318531f * cutoff));
}
void centroid_filter_init(void)
{
    memset(cf, 0, sizeof(cf));
}
void centroid_filter_apply(Centroids *c, float dt)
{
    if (!isfinite(dt) || dt < 0.002f) dt = 0.002f;
    if (dt > 0.5f) dt = 0.5f;
    Centroid *a[8] = {&c->up,&c->down,&c->left,&c->right,
                     &c->in_up,&c->in_down,&c->in_left,&c->in_right};
    uint8_t positive[2] = {0}, negative[2] = {0};
    float speed[2] = {0};
    for (uint8_t e = 0; e < 8; e++) {
        CentroidFilter *f = &cf[e];
        if (!a[e]->valid || !isfinite(a[e]->position)) {
            f->init = 0; /* 不让丢帧期间的速度外推污染下一次真实位置 */
            a[e]->valid = 0;
            continue;
        }
        float z = a[e]->position;
        if (!f->init) {
            f->pos = f->last_raw = z;
            f->dx = 0;
            f->init = 1;
        }
        float delta = z - f->last_raw;
        f->dx += alpha_lp(CF_D_CUTOFF, dt) * (delta / dt - f->dx);
        uint8_t axis = e % 4u < 2u ? 0u : 1u;
        if (delta > 0.15f) positive[axis]++;
        if (delta < -0.15f) negative[axis]++;
        if (fabsf(f->dx) > speed[axis]) speed[axis] = fabsf(f->dx);
    }
    for (uint8_t e = 0; e < 8; e++) {
        if (!a[e]->valid) continue;
        CentroidFilter *f = &cf[e];
        float z = a[e]->position;
        uint8_t axis = e % 4u < 2u ? 0u : 1u;
        uint8_t moving = positive[axis] >= 2u || negative[axis] >= 2u;
        float gain = alpha_lp(0.85f + 0.10f * speed[axis], dt);
        if (moving && gain < 0.90f) gain = 0.90f;
        /* 单边有明确位移时仍可快速跟随，不能因为另一边弱光而冻结。 */
        if (fabsf(z - f->pos) > 1.0f && gain < 0.85f) gain = 0.85f;
        if (moving || fabsf(z - f->pos) > 0.06f)
            f->pos += gain * (z - f->pos);
        f->last_raw = z;
        a[e]->position = f->pos;
    }
}
