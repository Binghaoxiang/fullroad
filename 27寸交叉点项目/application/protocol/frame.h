// frame.h — 新增声明
#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include "position.h"
#include "centroid.h"
#include "candidate.h"
#include "select.h"
#include "config.h"

#define FRAME_HEADER_0  0xAA
#define FRAME_HEADER_1  0x55
#define FRAME_FOOTER_0  0xFE
#define FRAME_FOOTER_1  0xFD

// Self-describing diagnostic stream consumed by UI/unified_ui.py.
#define FRAME_DIAG_HEADER_0  0xAA
#define FRAME_DIAG_HEADER_1  0x56
#define FRAME_DIAG_VERSION   4
#define FRAME_DIAG_TYPE_SNAPSHOT 1
#define FRAME_DIAG_FIXED_HEADER_LEN (2 + 1 + 1 + 2 + 4 + 4 + 2 + 1 + 1 + EDGE_COUNT * 2)
#define FRAME_DIAG_FLAG_BEFORE    0x0001u
#define FRAME_DIAG_FLAG_PROCESSED 0x0002u
#define FRAME_DIAG_FLAG_CENTROID  0x0004u
#define FRAME_DIAG_FLAG_POSITION  0x0008u
#define FRAME_DIAG_FLAG_DEBUG     0x0010u
#define FRAME_DIAG_FLAG_CANDIDATE 0x0020u
#define FRAME_DIAG_FLAG_PARAMS    0x0040u
#define FRAME_DIAG_FLAGS     0x007Eu
#define FRAME_DIAG_CENTROID_BYTES (EDGE_COUNT * 9)
#define FRAME_DIAG_MAPPED_BYTES   (EDGE_COUNT * 4)
#define FRAME_DIAG_POSITION_BYTES 9
#define FRAME_DIAG_DEBUG_BYTES    94
#define FRAME_DIAG_CAND_ENTRY_BYTES 17
#define FRAME_DIAG_CAND_BYTES (EDGE_COUNT + EDGE_COUNT * MAX_CAND * FRAME_DIAG_CAND_ENTRY_BYTES + EDGE_COUNT + 2 + 4 * 5)
#define FRAME_DIAG_PARAM_BYTES 1168
#define FRAME_DIAG_MAX_PAYLOAD (FRAME_RAW_BYTES_MAX + FRAME_DIAG_CENTROID_BYTES + FRAME_DIAG_MAPPED_BYTES + FRAME_DIAG_POSITION_BYTES + FRAME_DIAG_DEBUG_BYTES + FRAME_DIAG_CAND_BYTES + FRAME_DIAG_PARAM_BYTES)
#define FRAME_DIAG_MAX_LEN (FRAME_DIAG_FIXED_HEADER_LEN + FRAME_DIAG_MAX_PAYLOAD + 4)

#define FRAME_POS_LEN   14
// 缓冲按最坏情况(全 35cm 板)分配，实际发送长度 = Σ各边实际点数*2 + 结果段，帧内不含长度字段
#define FRAME_RAW_LEN   (2 + FRAME_RAW_BYTES_MAX + 3)

// 全帧 = 原始波形(8边 uint16) + 8边质心(每条: position float + peak float + valid u8) + 坐标(x float + y float + valid u8) + 交点调试(18)
#define FRAME_RAW_BYTES      FRAME_RAW_BYTES_MAX
#define FRAME_RESULT_BYTES   (EDGE_COUNT * 9 + 9 + 18)
#define FRAME_FULL_DATA_LEN  (FRAME_RAW_BYTES + FRAME_RESULT_BYTES)
#define FRAME_FULL_LEN       (2 + FRAME_FULL_DATA_LEN + 1 + 2)

// 候选调试帧：原始波形 + n_cand[8] + 每候选 {pos f32, peak u16, width u8, qflags u8}
#define FRAME_CAND_ENTRY_LEN 8
#define FRAME_CAND_MAX_ENTRIES (EDGE_COUNT * MAX_CAND)
#define FRAME_CAND_SELECT_LEN (EDGE_COUNT + 4 + 16 + 8 + 1 + 2 + 8)   // + cross_lines/source/conf/diff
#define FRAME_CAND_MAX_DATA_LEN (FRAME_RAW_BYTES + EDGE_COUNT + FRAME_CAND_MAX_ENTRIES * FRAME_CAND_ENTRY_LEN + FRAME_CAND_SELECT_LEN)
#define FRAME_CAND_MAX_LEN (2 + FRAME_CAND_MAX_DATA_LEN + 1 + 2)

void frame_send_position(Position *p);
void frame_send_raw(uint16_t *raw[8], const uint16_t *edge_len);
void frame_send_full(uint16_t *raw[8], const uint16_t *edge_len, Centroids *c, Position *p, const PositionDebug *dbg);
void frame_send_candidates(uint16_t *raw[8], const uint16_t *edge_len,
                           const EdgeCandidates cand[8],
                           const SelectionResult *sel);
void frame_send_diagnostic(uint16_t *processed[8], const uint16_t *edge_len,
                           Centroids *c,
                           Position *p, const PositionDebug *dbg,
                           const EdgeCandidates candidates[8],
                           const SelectionResult *selection,
                           uint32_t sequence, uint32_t tick_ms);

#endif
