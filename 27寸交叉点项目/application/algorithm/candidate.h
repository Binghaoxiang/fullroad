// candidate.h
#ifndef CANDIDATE_H
#define CANDIDATE_H

#include <stdint.h>
#include "config.h"

typedef struct {
    float pos;          // 亚像素位置（传感器索引）
    float peak;         // 本候选局部峰值，不是全局峰
    float width;        // FWHM / 二阶矩
    float energy;       // 局部基线以上能量
    float snr;          // 峰值 / 噪声
    float quality;      // 候选质量（0~1），供后续视线/投票使用
    uint8_t saturated;  // 是否削顶
    uint8_t valid;
} EdgeCandidate;

typedef struct {
    uint8_t n;
    EdgeCandidate c[MAX_CAND];
} EdgeCandidates;

// 8 边候选提取；raw[e] 指向该边原始数据，edge_len[e] 为数据槽数
void candidates_find(uint16_t *raw[8], const uint16_t *edge_len,
                     EdgeCandidates out[8]);

// 8 边候选提取（指定 K；DEBUG_K=4 用于最坏情况）
void candidates_find_k(uint16_t *raw[8], const uint16_t *edge_len,
                       uint8_t k, EdgeCandidates out[8]);

// 单边候选提取（供离线/单边调试使用）
void candidates_find_edge(uint8_t edge, const uint16_t *signal,
                          uint16_t len, EdgeCandidates *out);
void candidates_find_edge_k(uint8_t edge, const uint16_t *signal,
                            uint16_t len, uint8_t k, EdgeCandidates *out);

#endif
