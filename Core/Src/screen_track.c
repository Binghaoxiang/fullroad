/**
 * screen_track.c  -  固件端信号处理 + 屏幕坐标定位
 *
 * 把原来在上位机 Python 里做的处理全部搬到固件：
 *   edge_dsp.process_edge()  ->  edge_proc()
 *       自适应基线 / 3点中值 / Savitzky-Golay 5点 / 自适应阈值 / 主峰重心
 *   screen_pos.localize()    ->  st_dump_edges() 里的融合
 *       逐边线性标定 + 峰值平方加权融合
 */

#include <math.h>
#include "config.h"
#include "board.h"
#include "adc64.h"
#include "uart.h"
#include "screen_track.h"

#define E0_K     194.86f
#define E0_B       7.18f
#define E1_K    (-106.40f)
#define E1_B     110.90f
#define E2_K     180.85f
#define E2_B       8.79f
#define E3_K    (-275.50f)
#define E3_B     145.50f

#define ST_MAX_PTS   64U

static uint16_t s_raw[4][ST_MAX_PTS];
static uint16_t s_prc[4][ST_MAX_PTS];
static uint8_t  s_n4[4];
static float    s_base[4];
static float    s_wrk[ST_MAX_PTS];
static float    s_u[4];
static float    s_pk[4];
static uint8_t  s_ok[4];
static float    s_x_mm;
static float    s_y_mm;

static void median3(float *a, uint8_t n)
{
    uint8_t i;
    float t0, t1, t2;
    float tmp[ST_MAX_PTS];

    if (n < 3U) { return; }
    tmp[0] = a[0];
    for (i = 1U; (uint8_t)(i + 1U) < n; i++) {
        float s;
        t0 = a[i - 1U]; t1 = a[i]; t2 = a[i + 1U];
        if (t0 > t1) { s = t0; t0 = t1; t1 = s; }
        if (t1 > t2) { s = t1; t1 = t2; t2 = s; }
        if (t0 > t1) { s = t0; t0 = t1; t1 = s; }
        tmp[i] = t1;
    }
    tmp[n - 1U] = a[n - 1U];
    for (i = 0U; i < n; i++) { a[i] = tmp[i]; }
}

static void savgol5(float *a, uint8_t n)
{
    uint8_t i;
    float tmp[ST_MAX_PTS];

    if (n < 5U) { return; }
    tmp[0] = a[0];
    tmp[1] = a[1];
    for (i = 2U; (uint8_t)(i + 2U) < n; i++) {
        tmp[i] = (-3.0f * a[i - 2U] + 12.0f * a[i - 1U] + 17.0f * a[i]
                  + 12.0f * a[i + 1U] - 3.0f * a[i + 2U]) / 35.0f;
    }
    tmp[n - 2U] = a[n - 2U];
    tmp[n - 1U] = a[n - 1U];
    for (i = 0U; i < n; i++) { a[i] = tmp[i]; }
}

static void edge_proc(uint8_t e)
{
    uint8_t i;
    uint8_t n = s_n4[e];
    uint8_t hi = 0U, lo, end;
    float   mx = 0.0f, thr, num = 0.0f, den = 0.0f, gmin = 1.0e9f;

    s_ok[e] = 0U;
    s_u[e]  = 0.0f;
    s_pk[e] = 0.0f;
    if ((n < 5U) || (n > ST_MAX_PTS)) { return; }

    /* ① 去全局最小值 */
    for (i = 0U; i < n; i++) { if ((float)s_raw[e][i] < gmin) { gmin = (float)s_raw[e][i]; } }
    for (i = 0U; i < n; i++) {
        float v = (float)s_raw[e][i] - gmin;
        s_wrk[i] = (v > 0.0f) ? v : 0.0f;
    }

    /* ② 中值 + Savitzky-Golay 5 点平滑 */
    median3(s_wrk, n);
    savgol5(s_wrk, n);

    /* ③ 输出曲线 */
    for (i = 0U; i < n; i++) {
        s_prc[e][i] = (uint16_t)((s_wrk[i] > 4095.0f) ? 4095.0f : s_wrk[i]);
    }

    /* ④ 峰值 */
    for (i = 0U; i < n; i++) { if (s_wrk[i] > mx) { mx = s_wrk[i]; } }
    s_pk[e] = mx;
    if (mx < 20.0f) { return; }                  /* 确实没有光 */

    /* ⑤ 峰位置 + 半高宽区间（退化时取峰两侧各 2 点） */
    for (i = 0U; i < n; i++) { if (s_wrk[i] >= mx) { hi = i; break; } }
    thr = mx * 0.5f;
    lo = hi; end = hi;
    while ((lo > 0U) && (s_wrk[lo - 1U] > thr)) { lo--; }
    while (((uint8_t)(end + 1U) < n) && (s_wrk[end + 1U] > thr)) { end++; }
    if (lo == end) {
        lo = (hi >= 2U) ? (uint8_t)(hi - 2U) : 0U;
        end = (uint8_t)((hi + 2U) < n ? (hi + 2U) : (n - 1U));
    }

    /* ⑥ 加权重心 */
    for (i = lo; i <= end; i++) {
        num += s_wrk[i] * (float)i;
        den += s_wrk[i];
    }
    if (den < 1.0f) { den = 1.0f; num = (float)hi; }

    s_u[e] = (num / den) / (float)(n - 1U);
    if (s_u[e] < 0.0f) { s_u[e] = 0.0f; }
    if (s_u[e] > 1.0f) { s_u[e] = 1.0f; }
    s_ok[e] = 1U;
}

void st_init(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++) { s_base[i] = 0.0f; s_n4[i] = 0U; s_ok[i] = 0U; }
    s_x_mm = 0.0f;
    s_y_mm = 0.0f;
}

float st_x_mm(void) { return s_x_mm; }
float st_y_mm(void) { return s_y_mm; }
uint8_t st_ok(uint8_t e) { return (e < 4U) ? s_ok[e] : 0U; }
float st_peak(uint8_t e) { return (e < 4U) ? s_pk[e] : 0.0f; }
uint8_t st_n(uint8_t e)  { return (e < 4U) ? s_n4[e] : 0U; }

void st_dump_edges(void)
{
    uint8_t  k;
    uint16_t i;
    float    x0, x2, y1, y3, x, y;

    adc64_read_edges_all(s_raw[0], s_raw[1], s_raw[2], s_raw[3], s_n4);

    for (k = 0U; k < 4U; k++) { edge_proc(k); }

    x0 = E0_K * s_u[0] + E0_B;
    x2 = E2_K * s_u[2] + E2_B;
    y1 = E1_K * s_u[1] + E1_B;
    y3 = E3_K * s_u[3] + E3_B;

    if (s_ok[0] && s_ok[2]) {
        float w0 = s_pk[0] * s_pk[0];
        float w2 = s_pk[2] * s_pk[2];
        x = (x0 * w0 + x2 * w2) / (w0 + w2);
    } else if (s_ok[0]) {
        x = x0;
    } else if (s_ok[2]) {
        x = x2;
    } else {
        x = s_x_mm;
    }

    if (s_ok[1] && s_ok[3]) {
        float w1 = s_pk[1] * s_pk[1];
        float w3 = s_pk[3] * s_pk[3];
        y = (y1 * w1 + y3 * w3) / (w1 + w3);
    } else if (s_ok[1]) {
        y = y1;
    } else if (s_ok[3]) {
        y = y3;
    } else {
        y = s_y_mm;
    }

    if (x < 0.0f)   { x = 0.0f; }
    if (x > 200.0f) { x = 200.0f; }
    if (y < 0.0f)   { y = 0.0f; }
    if (y > 100.0f) { y = 100.0f; }
    s_x_mm = x;
    s_y_mm = y;

    for (k = 0U; k < 4U; k++) {
        uart_puts("E");
        uart_put_u32((uint32_t)k);
        uart_puts(":");
        uart_put_u32((uint32_t)s_n4[k]);
        for (i = 0U; i < (uint16_t)s_n4[k]; i++) {
            uart_putc(' ');
            uart_put_u32((uint32_t)s_prc[k][i]);
        }
        uart_puts("\r\n");
    }

    uart_puts("POS ");
    uart_put_i32((int32_t)(x * 10.0f));
    uart_putc(' ');
    uart_put_i32((int32_t)(y * 10.0f));
    uart_puts("\r\n");
}
