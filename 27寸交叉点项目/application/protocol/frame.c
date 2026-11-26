// frame.c
#include "frame.h"
#include "bsp_uart.h"
#include <string.h>
#include "config.h"
#include "runtime_params.h"
#include "can_proto.h"
#include <math.h>

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put_u16(uint8_t *buf, int *idx, uint16_t v)
{
    buf[(*idx)++] = (uint8_t)(v & 0xFFu);
    buf[(*idx)++] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *buf, int *idx, uint32_t v)
{
    memcpy(&buf[*idx], &v, 4);
    *idx += 4;
}

void frame_send_position(Position *p)
{
    static uint8_t buf[FRAME_POS_LEN];  // 改成static
    uint8_t checksum = 0;

    buf[0] = FRAME_HEADER_0;
    buf[1] = FRAME_HEADER_1;
    memcpy(&buf[2], &p->x, 4);
    memcpy(&buf[6], &p->y, 4);
    buf[10] = p->valid;
    for (int i = 2; i < 11; i++) checksum ^= buf[i];
    buf[11] = checksum;
    buf[12] = FRAME_FOOTER_0;
    buf[13] = FRAME_FOOTER_1;

    bsp_uart_send(buf, FRAME_POS_LEN);
}

void frame_send_raw(uint16_t *raw[8], const uint16_t *edge_len)
{
    static uint8_t buf[FRAME_RAW_LEN];
    uint8_t checksum = 0;
    int idx = 2;

    buf[0] = FRAME_HEADER_0;
    buf[1] = FRAME_HEADER_1;

    // 顺序: UP → DOWN → LEFT → RIGHT → IN_UP → IN_DOWN → IN_LEFT → IN_RIGHT
    uint8_t order[8] = {
        EDGE_UP, EDGE_DOWN, EDGE_LEFT, EDGE_RIGHT,
        EDGE_IN_UP, EDGE_IN_DOWN, EDGE_IN_LEFT, EDGE_IN_RIGHT
    };
    for (int j = 0; j < 8; j++) {
        int e = order[j];
        for (uint16_t i = 0; i < edge_len[e]; i++) {
            buf[idx++] = (uint8_t)(raw[e][i] & 0xFF);
            buf[idx++] = (uint8_t)((raw[e][i] >> 8) & 0xFF);
        }
    }

    for (int i = 2; i < idx; i++) checksum ^= buf[i];
    buf[idx++] = checksum;
    buf[idx++] = FRAME_FOOTER_0;
    buf[idx++] = FRAME_FOOTER_1;

    bsp_uart_send(buf, idx);
}

// 单帧下发：原始波形(8边) + 8边质心 + 坐标 + 交点调试。供上位机一次性同步显示波形、质心数值、光斑坐标
void frame_send_full(uint16_t *raw[8], const uint16_t *edge_len, Centroids *c, Position *p, const PositionDebug *dbg)
{
    static uint8_t buf[FRAME_FULL_LEN];
    uint8_t checksum = 0;
    int idx = 2;

    buf[0] = FRAME_HEADER_0;
    buf[1] = FRAME_HEADER_1;

    // 1) 原始波形，顺序与 frame_send_raw 一致
    uint8_t order[8] = {
        EDGE_UP, EDGE_DOWN, EDGE_LEFT, EDGE_RIGHT,
        EDGE_IN_UP, EDGE_IN_DOWN, EDGE_IN_LEFT, EDGE_IN_RIGHT
    };
    for (int j = 0; j < 8; j++) {
        int e = order[j];
        for (uint16_t i = 0; i < edge_len[e]; i++) {
            buf[idx++] = (uint8_t)(raw[e][i] & 0xFF);
            buf[idx++] = (uint8_t)((raw[e][i] >> 8) & 0xFF);
        }
    }

    // 2) 8 边质心（顺序同上）
    Centroid *cs[8] = {
        &c->up, &c->down, &c->left, &c->right,
        &c->in_up, &c->in_down, &c->in_left, &c->in_right
    };
    for (int e = 0; e < 8; e++) {
        memcpy(&buf[idx], &cs[e]->position, 4); idx += 4;
        memcpy(&buf[idx], &cs[e]->peak, 4);     idx += 4;
        buf[idx++] = cs[e]->valid;
    }

    // 3) 坐标
    memcpy(&buf[idx], &p->x, 4); idx += 4;
    memcpy(&buf[idx], &p->y, 4); idx += 4;
    buf[idx++] = p->valid;

    // 4) 交点调试数据；旧全帧布局中的融合字段仅保留零占位。
    if (dbg) {
        const float zero = 0.0f;
        memcpy(&buf[idx], &dbg->x_line, 4);  idx += 4;
        memcpy(&buf[idx], &dbg->y_line, 4);  idx += 4;
        /* 旧全帧格式保留两个零占位，避免改变其固定帧长度。 */
        memcpy(&buf[idx], &zero, 4); idx += 4;
        memcpy(&buf[idx], &zero, 4); idx += 4;
        buf[idx++] = dbg->have_x;
        buf[idx++] = dbg->have_y;
    }

    for (int i = 2; i < idx; i++) checksum ^= buf[i];
    buf[idx++] = checksum;
    buf[idx++] = FRAME_FOOTER_0;
    buf[idx++] = FRAME_FOOTER_1;

    bsp_uart_send(buf, idx);
}
// 候选调试帧：原始波形 + n_cand[8] + 每候选 8B；
// 仅供调试模式使用，产品仍走 14B 帧/全帧方式传输。
void frame_send_candidates(uint16_t *raw[8], const uint16_t *edge_len,
                           const EdgeCandidates cand[8],
                           const SelectionResult *sel)
{
    static uint8_t buf[FRAME_CAND_MAX_LEN];
    uint8_t checksum = 0;
    int idx = 2;
    uint8_t order[8] = {
        EDGE_UP, EDGE_DOWN, EDGE_LEFT, EDGE_RIGHT,
        EDGE_IN_UP, EDGE_IN_DOWN, EDGE_IN_LEFT, EDGE_IN_RIGHT
    };

    buf[0] = FRAME_HEADER_0;
    buf[1] = FRAME_HEADER_1;

    for (int j = 0; j < 8; j++) {
        int e = order[j];
        for (uint16_t i = 0; i < edge_len[e]; i++) {
            buf[idx++] = (uint8_t)(raw[e][i] & 0xFF);
            buf[idx++] = (uint8_t)((raw[e][i] >> 8) & 0xFF);
        }
    }

    int n_off = idx;
    for (int j = 0; j < 8; j++) {
        int e = order[j];
        uint8_t n = cand[e].n;
        if (n > MAX_CAND) n = MAX_CAND;
        buf[idx++] = n;
    }

    for (int j = 0; j < 8; j++) {
        int e = order[j];
        uint8_t n = buf[n_off + j];
        for (uint8_t k = 0; k < n; k++) {
            const EdgeCandidate *c = &cand[e].c[k];
            float pos = c->pos;
            float pkf = c->peak * 10000.0f;
            uint16_t peak;
            uint8_t width = (uint8_t)(c->width + 0.5f);
            uint8_t q = (uint8_t)(c->quality * 127.0f);
            if (pkf > 65535.0f) pkf = 65535.0f;
            peak = (uint16_t)pkf;
            if (width == 0) width = 1;
            if (q > 127) q = 127;
            if (c->saturated) q |= 0x80;
            memcpy(&buf[idx], &pos, 4); idx += 4;
            memcpy(&buf[idx], &peak, 2); idx += 2;
            buf[idx++] = width;
            buf[idx++] = q;
        }
    }

    // 选择结果：sel_idx[8] + mask/n/hq/mode + x/y/residual/confidence
    // + x_line/y_line/line_valid + cross_lines/source/conf/diff
    if (sel) {
        for (int j = 0; j < 8; j++) buf[idx++] = sel->sel_idx[j];
        buf[idx++] = sel->support_mask;
        buf[idx++] = sel->n_support;
        buf[idx++] = sel->hq_support;
        buf[idx++] = sel->mode;
        memcpy(&buf[idx], &sel->x, 4); idx += 4;
        memcpy(&buf[idx], &sel->y, 4); idx += 4;
        memcpy(&buf[idx], &sel->residual, 4); idx += 4;
        memcpy(&buf[idx], &sel->confidence, 4); idx += 4;
        memcpy(&buf[idx], &sel->x_line, 4); idx += 4;
        memcpy(&buf[idx], &sel->y_line, 4); idx += 4;
        buf[idx++] = sel->line_valid;
        buf[idx++] = sel->cross_lines;
        buf[idx++] = sel->cross_source;
        memcpy(&buf[idx], &sel->cross_conf, 4); idx += 4;
        memcpy(&buf[idx], &sel->cross_diff, 4); idx += 4;
    } else {
        for (int j = 0; j < 8; j++) buf[idx++] = 0xFF;
        buf[idx++] = 0; buf[idx++] = 0; buf[idx++] = 0; buf[idx++] = 0;
        float zero = 0.0f;
        for (int j = 0; j < 6; j++) { memcpy(&buf[idx], &zero, 4); idx += 4; }
        buf[idx++] = 0;
        buf[idx++] = 0;
        buf[idx++] = 0;
        for (int j = 0; j < 2; j++) { memcpy(&buf[idx], &zero, 4); idx += 4; }
    }

    for (int i = 2; i < idx; i++) checksum ^= buf[i];
    buf[idx++] = checksum;
    buf[idx++] = FRAME_FOOTER_0;
    buf[idx++] = FRAME_FOOTER_1;

    bsp_uart_send(buf, idx);
}

void frame_send_diagnostic(uint16_t *processed[8], const uint16_t *edge_len,
                           Centroids *c,
                           Position *p, const PositionDebug *dbg,
                           const EdgeCandidates candidates[8],
                           const SelectionResult *selection,
                           uint32_t sequence, uint32_t tick_ms)
{
    static uint8_t buf[FRAME_DIAG_MAX_LEN];
    uint8_t order[EDGE_COUNT] = {
        EDGE_UP, EDGE_DOWN, EDGE_LEFT, EDGE_RIGHT,
        EDGE_IN_UP, EDGE_IN_DOWN, EDGE_IN_LEFT, EDGE_IN_RIGHT
    };
    Centroid *cs[EDGE_COUNT] = {
        &c->up, &c->down, &c->left, &c->right,
        &c->in_up, &c->in_down, &c->in_left, &c->in_right
    };
    uint16_t points = 0;
    int idx = 0;

    for (uint8_t j = 0; j < EDGE_COUNT; j++) points += edge_len[order[j]];
    uint16_t payload_len = (uint16_t)(points * 2u +
                           FRAME_DIAG_CENTROID_BYTES +
                           FRAME_DIAG_MAPPED_BYTES +
                           FRAME_DIAG_POSITION_BYTES +
                           FRAME_DIAG_DEBUG_BYTES +
                           FRAME_DIAG_CAND_BYTES +
                           FRAME_DIAG_PARAM_BYTES);

    buf[idx++] = FRAME_DIAG_HEADER_0;
    buf[idx++] = FRAME_DIAG_HEADER_1;
    buf[idx++] = FRAME_DIAG_VERSION;
    buf[idx++] = FRAME_DIAG_TYPE_SNAPSHOT;
    put_u16(buf, &idx, payload_len);
    put_u32(buf, &idx, sequence);
    put_u32(buf, &idx, tick_ms);
    put_u16(buf, &idx, FRAME_DIAG_FLAGS);
    buf[idx++] = EDGE_COUNT;
#if MCU_IS_MASTER
    /* The formerly reserved byte reports edges received in this acquisition
       cycle.  It distinguishes an actual zero waveform from missing CAN data. */
    buf[idx++] = can_proto_get_cycle_edge_mask();
#else
    buf[idx++] = 0;
#endif
    for (uint8_t j = 0; j < EDGE_COUNT; j++) put_u16(buf, &idx, edge_len[order[j]]);

    // Board-processed values used by the positioning pipeline and UI.
    for (uint8_t j = 0; j < EDGE_COUNT; j++) {
        uint8_t e = order[j];
        for (uint16_t i = 0; i < edge_len[e]; i++) put_u16(buf, &idx, processed[e][i]);
    }

    for (uint8_t e = 0; e < EDGE_COUNT; e++) {
        memcpy(&buf[idx], &cs[e]->position, 4); idx += 4;
        memcpy(&buf[idx], &cs[e]->peak, 4); idx += 4;
        buf[idx++] = cs[e]->valid;
    }
    /* Exact per-edge millimetre coordinate consumed by vertical_axis_fuse().
       The UI must display this value instead of rebuilding an obsolete
       independent calibration curve from raw nodes.  Invalid centroids use
       NaN, preserving the distinction from a legitimate zero coordinate. */
    for (uint8_t e = 0; e < EDGE_COUNT; e++) {
        float mapped = cs[e]->valid
                     ? runtime_params_map_edge(e, cs[e]->position)
                     : NAN;
        memcpy(&buf[idx], &mapped, 4); idx += 4;
    }
    memcpy(&buf[idx], &p->x, 4); idx += 4;
    memcpy(&buf[idx], &p->y, 4); idx += 4;
    buf[idx++] = p->valid;

    if (dbg) {
        memcpy(&buf[idx], &dbg->x_line, 4); idx += 4;
        memcpy(&buf[idx], &dbg->y_line, 4); idx += 4;
        buf[idx++] = dbg->have_x;
        buf[idx++] = dbg->have_y;
        memcpy(&buf[idx], &dbg->x_cross_out, 4); idx += 4;
        memcpy(&buf[idx], &dbg->y_cross_out, 4); idx += 4;
        buf[idx++] = dbg->cross_out_valid;
        buf[idx++] = dbg->cross_source;
        buf[idx++] = dbg->cross_lines;
        memcpy(&buf[idx], &dbg->cross_conf, 4); idx += 4;
        memcpy(&buf[idx], &dbg->cross_diff, 4); idx += 4;
        for (uint8_t side = 0; side < 4; side++) {
            memcpy(&buf[idx], &dbg->line_x1[side], 4); idx += 4;
            memcpy(&buf[idx], &dbg->line_y1[side], 4); idx += 4;
            memcpy(&buf[idx], &dbg->line_x2[side], 4); idx += 4;
            memcpy(&buf[idx], &dbg->line_y2[side], 4); idx += 4;
        }
        buf[idx++] = dbg->line_valid_mask;
    } else {
        memset(&buf[idx], 0, FRAME_DIAG_DEBUG_BYTES);
        idx += FRAME_DIAG_DEBUG_BYTES;
    }

    /* All candidate detection and selection is firmware-owned.  The UI only
       renders this result and never searches the waveform itself. */
    for (uint8_t e = 0; e < EDGE_COUNT; e++) buf[idx++] = candidates[e].n;
    for (uint8_t e = 0; e < EDGE_COUNT; e++) {
        for (uint8_t k = 0; k < MAX_CAND; k++) {
            const EdgeCandidate *ce = &candidates[e].c[k];
            float zero = 0.0f;
            const float *pos = (k < candidates[e].n) ? &ce->pos : &zero;
            const float *peak = (k < candidates[e].n) ? &ce->peak : &zero;
            const float *width = (k < candidates[e].n) ? &ce->width : &zero;
            const float *quality = (k < candidates[e].n) ? &ce->quality : &zero;
            memcpy(&buf[idx], pos, 4); idx += 4;
            memcpy(&buf[idx], peak, 4); idx += 4;
            memcpy(&buf[idx], width, 4); idx += 4;
            memcpy(&buf[idx], quality, 4); idx += 4;
            buf[idx++] = (k < candidates[e].n) ? ce->valid : 0;
        }
    }
    if (selection) {
        for (uint8_t e = 0; e < EDGE_COUNT; e++) buf[idx++] = selection->sel_idx[e];
        buf[idx++] = selection->mode;
        buf[idx++] = selection->support_mask;
        memcpy(&buf[idx], &selection->confidence, 4); idx += 4;
        memcpy(&buf[idx], &selection->score, 4); idx += 4;
        memcpy(&buf[idx], &selection->residual, 4); idx += 4;
        memcpy(&buf[idx], &selection->x, 4); idx += 4;
        memcpy(&buf[idx], &selection->y, 4); idx += 4;
    } else {
        memset(&buf[idx], 0, EDGE_COUNT + 2 + 4 * 5);
        for (uint8_t e = 0; e < EDGE_COUNT; e++) buf[idx + e] = 0xFF;
        idx += EDGE_COUNT + 2 + 4 * 5;
    }

    runtime_params_encode(&buf[idx]);
    idx += FRAME_DIAG_PARAM_BYTES;

    uint16_t crc = crc16_ccitt(&buf[2], (uint16_t)(idx - 2));
    put_u16(buf, &idx, crc);
    buf[idx++] = FRAME_FOOTER_0;
    buf[idx++] = FRAME_FOOTER_1;
    bsp_uart_send(buf, (uint16_t)idx);
}
