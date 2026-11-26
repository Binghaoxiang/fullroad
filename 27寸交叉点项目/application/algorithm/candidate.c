// candidate.c
//
// Iteration 01: 单边多峰候选层。
// 只输出候选，不判真假；产品默认 K=3，调试可用 K=4。
// 旧 centroids_find() 路径保持不变，当前产品行为不变。
#include "candidate.h"
#include "board_map.h"
#include "config.h"

#include <math.h>
#include <string.h>

#define ADC_MAX          4095.0f
#define MAX_PEAKS        64
#define MAX_GROUPS       64

static float    s_norm[MAX_SENSORS_PER_EDGE];
static uint16_t s_hist[256];
static uint16_t s_peak_idx[MAX_PEAKS];
static float    s_peak_amp[MAX_PEAKS];

static uint16_t s_group_first[MAX_GROUPS];
static uint16_t s_group_last[MAX_GROUPS];
static float    s_group_amp[MAX_GROUPS];

static EdgeCandidate s_cand_all[MAX_GROUPS];
static uint8_t s_wide_mode[EDGE_COUNT];
static uint8_t s_wide_cnt[EDGE_COUNT];

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float estimate_noise(const uint16_t *signal, uint16_t n)
{
    uint32_t i, half, acc;
    uint16_t bin;

    memset(s_hist, 0, sizeof(s_hist));
    for (i = 0; i < n; i++) {
        uint16_t v = signal[i];
        if (v > 4095u) v = 4095u;
        s_hist[v >> 4]++;
    }

    half = n / 2u;
    acc = 0;
    bin = 0;
    for (i = 0; i < 256u; i++) {
        acc += s_hist[i];
        if (acc >= half) {
            bin = (uint16_t)i;
            break;
        }
    }
    return ((float)bin * 16.0f + 8.0f) / ADC_MAX;
}

static uint16_t valley_index(uint16_t a, uint16_t b)
{
    uint16_t i, best = a;
    float v = s_norm[a];
    for (i = a + 1; i <= b; i++) {
        if (s_norm[i] < v) {
            v = s_norm[i];
            best = i;
        }
    }
    return best;
}

static uint8_t near_seg_boundary(uint8_t edge, uint16_t idx)
{
    uint8_t seg, total = board_edge_seg_total(edge);
    for (seg = 1; seg < total; seg++) {
        int32_t boundary = (int32_t)board_seg_prefix(edge, seg);
        int32_t d = (int32_t)idx - boundary;
        if (d < 0) d = -d;
        if (d <= CAND_SEG_MERGE_TOL) return 1;
    }
    return 0;
}

static void add_peak(uint16_t *np, uint16_t idx, float amp)
{
    if (*np < MAX_PEAKS) {
        s_peak_idx[*np] = idx;
        s_peak_amp[*np] = amp;
        (*np)++;
    }
}

static void find_peaks(uint16_t n, float thr, uint16_t *np)
{
    uint16_t i;
    *np = 0;
    if (n < 2) return;

    if (s_norm[0] >= s_norm[1] && s_norm[0] >= thr)
        add_peak(np, 0, s_norm[0]);
    for (i = 1; i + 1 < n; i++) {
        if (s_norm[i] >= s_norm[i - 1] && s_norm[i] > s_norm[i + 1] &&
            s_norm[i] >= thr)
            add_peak(np, i, s_norm[i]);
    }
    if (s_norm[n - 1] > s_norm[n - 2] && s_norm[n - 1] >= thr)
        add_peak(np, n - 1, s_norm[n - 1]);
}

static float peak_prominence(uint16_t n, uint16_t idx)
{
    float hi = s_norm[idx];
    float left_min = hi;
    float right_min = hi;
    float lb, rb, base;
    int32_t j;

    j = (int32_t)idx - 1;
    while (j >= 0 && s_norm[j] <= hi) {
        if (s_norm[j] < left_min) left_min = s_norm[j];
        j--;
    }
    lb = (j < 0) ? 0.0f : left_min;

    j = (int32_t)idx + 1;
    while (j < (int32_t)n && s_norm[j] <= hi) {
        if (s_norm[j] < right_min) right_min = s_norm[j];
        j++;
    }
    rb = (j >= (int32_t)n) ? 0.0f : right_min;

    base = (lb > rb) ? lb : rb;
    return hi - base;
}

static void group_peaks(uint8_t edge, uint16_t np, uint16_t *ng)
{
    uint16_t p, g;
    *ng = 0;
    if (np == 0) return;

    s_group_first[0] = s_peak_idx[0];
    s_group_last[0]  = s_peak_idx[0];
    s_group_amp[0]   = s_peak_amp[0];
    *ng = 1;

    for (p = 1; p < np; p++) {
        g = (uint16_t)(*ng - 1);
        {
            uint16_t i1 = s_group_last[g];
            uint16_t i2 = s_peak_idx[p];
            uint16_t vi = valley_index(i1, i2);
            float valley = s_norm[vi];
            float min_amp = (s_group_amp[g] < s_peak_amp[p]) ?
                            s_group_amp[g] : s_peak_amp[p];
            uint8_t merge = 0;

            if ((i2 - i1) <= CAND_MERGE_DIST) merge = 1;
            if ((i2 - i1) <= CAND_RATIO_MERGE_SPAN &&
                valley >= CAND_MERGE_RATIO * min_amp) merge = 1;
            if ((i2 - i1) <= CAND_SEG_MERGE_SPAN &&
                (near_seg_boundary(edge, i1) || near_seg_boundary(edge, i2)))
                merge = 1;

            if (merge) {
                s_group_last[g] = i2;
                if (s_peak_amp[p] > s_group_amp[g]) {
                    s_group_amp[g] = s_peak_amp[p];
                }
            } else if (*ng < MAX_GROUPS) {
                s_group_first[*ng] = i2;
                s_group_last[*ng]  = i2;
                s_group_amp[*ng]   = s_peak_amp[p];
                (*ng)++;
            }
        }
    }
}

static void candidate_from_group(uint8_t edge, uint16_t n, uint16_t ng, uint8_t g,
                                 const uint16_t *signal,
                                 float noise, EdgeCandidate *c)
{
    uint16_t lo, hi, i, pk_idx, group_lo, group_hi;
    float base, sumw = 0.0f, sumiw = 0.0f;
    float peak = 0.0f, width, half, snr, q;

    if (g == 0) {
        lo = (s_group_first[g] > CAND_GROUP_PAD) ?
             (uint16_t)(s_group_first[g] - CAND_GROUP_PAD) : 0;
    } else {
        lo = valley_index(s_group_last[g - 1], s_group_first[g]);
    }

    if (g + 1 < ng) {
        hi = valley_index(s_group_last[g], s_group_first[g + 1]);
    } else {
        hi = s_group_last[g];
        if (hi + CAND_GROUP_PAD < n) hi = (uint16_t)(hi + CAND_GROUP_PAD);
        else hi = (uint16_t)(n - 1);
    }

    if (lo > s_group_first[g]) lo = s_group_first[g];
    if (hi < s_group_last[g]) hi = s_group_last[g];
    if (hi >= n) hi = (uint16_t)(n - 1);

    group_lo = lo;
    group_hi = hi;
    pk_idx = lo;
    peak = s_norm[lo];
    for (i = lo + 1; i <= hi; i++) {
        if (s_norm[i] > peak) {
            peak = s_norm[i];
            pk_idx = i;
        }
    }
    {
        uint16_t span = (uint16_t)(s_group_last[g] - s_group_first[g]);
        if (span >= CAND_WIDE_ON_SPAN) {
            if (!s_wide_mode[edge]) {
                if (++s_wide_cnt[edge] >= CAND_WIDE_CONFIRM) {
                    s_wide_mode[edge] = 1;
                    s_wide_cnt[edge] = 0;
                }
            } else {
                s_wide_cnt[edge] = 0;
            }
        } else if (span <= CAND_WIDE_OFF_SPAN) {
            if (s_wide_mode[edge]) {
                if (++s_wide_cnt[edge] >= CAND_WIDE_CONFIRM) {
                    s_wide_mode[edge] = 0;
                    s_wide_cnt[edge] = 0;
                }
            } else {
                s_wide_cnt[edge] = 0;
            }
        } else {
            s_wide_cnt[edge] = 0;
        }
    }
    if (s_wide_mode[edge]) {
        lo = group_lo;
        hi = group_hi;
    } else {
        if (pk_idx > CAND_CENTROID_HALF)
            lo = (uint16_t)(pk_idx - CAND_CENTROID_HALF);
        else
            lo = 0;
        if (pk_idx + CAND_CENTROID_HALF < hi)
            hi = (uint16_t)(pk_idx + CAND_CENTROID_HALF);
        if (hi >= n) hi = (uint16_t)(n - 1);
        if (lo < s_group_first[g]) lo = s_group_first[g];
    }

    base = s_norm[lo];
    for (i = lo + 1; i <= hi; i++)
        if (s_norm[i] < base) base = s_norm[i];

    for (i = lo; i <= hi; i++) {
        float w = s_norm[i] - base;
        if (w < 0.0f) w = 0.0f;
        sumw += w;
        sumiw += (float)i * w;
    }

    if (sumw <= 1e-9f) {
        c->valid = 0;
        return;
    }

    half = base + 0.5f * (peak - base);
    {
        uint16_t l = pk_idx, r = pk_idx;
        while (l > lo && s_norm[l] >= half) l--;
        while (r < hi && s_norm[r] >= half) r++;
        width = (float)(r - l);
    }
    if (width < 1.0f) width = 1.0f;

    snr = peak / ((noise > 0.0005f) ? noise : 0.0005f);
    q = peak / CAND_PEAK_REF;
    if (q > 1.0f) q = 1.0f;
    q *= snr / (snr + CAND_SNR_REF);
    {
        float dw = (width - CAND_WIDTH_TYP) / CAND_WIDTH_SIGMA;
        q *= expf(-0.5f * dw * dw);
    }
    if (width < CAND_MIN_WIDTH || width > CAND_MAX_WIDTH) q *= 0.5f;
    if (signal[pk_idx] >= 4095u) q *= 0.9f;
    q = clampf(q, 0.0f, 1.0f);

    c->pos = sumiw / sumw;
    c->peak = peak;
    c->width = width;
    c->energy = sumw;
    c->snr = snr;
    c->quality = q;
    c->saturated = (signal[pk_idx] >= 4095u) ? 1u : 0u;
    c->valid = 1;
}

void candidates_find_edge_k(uint8_t edge, const uint16_t *signal,
                            uint16_t len, uint8_t k, EdgeCandidates *out)
{
    uint16_t i, n, np, ng, g;
    float noise, thr;

    out->n = 0;
    n = (len > MAX_SENSORS_PER_EDGE) ? MAX_SENSORS_PER_EDGE : len;
    if (n < 3) return;

    for (i = 0; i < n; i++) {
        s_norm[i] = (float)signal[i] / ADC_MAX;
    }

    noise = estimate_noise(signal, n);
    thr = CAND_ABS_THRESHOLD;
    if (noise * 2.0f > thr) thr = noise * 2.0f;

    find_peaks(n, thr, &np);

    /* prominence filter (stable in index order) */
    {
        uint16_t w = 0;
        for (i = 0; i < np; i++) {
            if (peak_prominence(n, s_peak_idx[i]) >= CAND_PROMINENCE) {
                s_peak_idx[w] = s_peak_idx[i];
                s_peak_amp[w] = s_peak_amp[i];
                w++;
            }
        }
        np = w;
    }

    group_peaks(edge, np, &ng);
    if (ng == 0) return;

    /* build candidates */
    {
        uint16_t nc = 0;
        for (g = 0; g < ng; g++) {
            EdgeCandidate c;
            memset(&c, 0, sizeof(c));
            candidate_from_group(edge, n, ng, g, signal, noise, &c);
            if (!c.valid) continue;
            /* 产品级光存在前置门禁：暗态偶发单点不应进入几何组合。
               最终仍需要水平+垂直两类内外视线连续两帧一致才输出。 */
            if (c.peak < CAND_PRODUCT_MIN_PEAK ||
                c.snr < CAND_PRODUCT_MIN_SNR ||
                c.quality < CAND_PRODUCT_MIN_QUALITY ||
                c.width < CAND_PRODUCT_WIDTH_MIN ||
                c.width > CAND_PRODUCT_WIDTH_MAX)
                continue;
            s_cand_all[nc++] = c;
            if (nc >= MAX_GROUPS) break;
        }

        /* keep top K by quality, then sort by position */
        {
            uint8_t selected[MAX_CAND];
            uint8_t ns = 0;
            uint16_t j;

            if (k == 0) k = 1;
            if (k > MAX_CAND) k = MAX_CAND;
            for (j = 0; j < nc; j++) {
                if (ns < k) {
                    selected[ns++] = (uint8_t)j;
                } else {
                    uint8_t worst = 0;
                    uint8_t m;
                    for (m = 1; m < k; m++) {
                        if (s_cand_all[selected[m]].quality <
                            s_cand_all[selected[worst]].quality)
                            worst = m;
                    }
                    if (s_cand_all[j].quality >
                        s_cand_all[selected[worst]].quality)
                        selected[worst] = (uint8_t)j;
                }
            }

            /* sort selected by position */
            for (j = 1; j < ns; j++) {
                uint8_t key = selected[j];
                int32_t m = (int32_t)j - 1;
                while (m >= 0 &&
                       s_cand_all[selected[m]].pos > s_cand_all[key].pos) {
                    selected[m + 1] = selected[m];
                    m--;
                }
                selected[m + 1] = key;
            }

            for (j = 0; j < ns; j++) {
                out->c[j] = s_cand_all[selected[j]];
            }
            out->n = ns;
        }
    }
}

void candidates_find(uint16_t *raw[8], const uint16_t *edge_len,
                     EdgeCandidates out[8])
{
    candidates_find_k(raw, edge_len, PROD_K, out);
}

void candidates_find_k(uint16_t *raw[8], const uint16_t *edge_len,
                       uint8_t k, EdgeCandidates out[8])
{
    uint8_t e;
    for (e = 0; e < 8; e++) {
        candidates_find_edge_k(e, raw[e], edge_len[e], k, &out[e]);
    }
}

void candidates_find_edge(uint8_t edge, const uint16_t *signal,
                          uint16_t len, EdgeCandidates *out)
{
    candidates_find_edge_k(edge, signal, len, PROD_K, out);
}
