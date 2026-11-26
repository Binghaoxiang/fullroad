// board_map.c — 逐段板型表(内容来自 config.h 的 *_SEG_BOARD)
#include "board_map.h"
#include "config.h"

#define MAX_SEG 4

static const uint8_t seg_board[EDGE_COUNT][MAX_SEG] = {
    { UP_SEG_BOARD },
    { DOWN_SEG_BOARD },
    { LEFT_SEG_BOARD },
    { RIGHT_SEG_BOARD },
    { IN_UP_SEG_BOARD },
    { IN_DOWN_SEG_BOARD },
    { IN_LEFT_SEG_BOARD },
    { IN_RIGHT_SEG_BOARD },
};

static const uint8_t seg_total[EDGE_COUNT] = {
    UP_SEGMENTS, DOWN_SEGMENTS, LEFT_SEGMENTS, RIGHT_SEGMENTS,
    IN_UP_SEGMENTS, IN_DOWN_SEGMENTS, IN_LEFT_SEGMENTS, IN_RIGHT_SEGMENTS
};

uint8_t board_edge_seg_total(uint8_t edge)
{
    return (edge < EDGE_COUNT) ? seg_total[edge] : 0;
}

uint16_t board_seg_sensors(uint8_t edge, uint8_t seg)
{
    if (edge >= EDGE_COUNT || seg >= board_edge_seg_total(edge)) return 0;
    return (seg_board[edge][seg] == BOARD_35CM) ? SEG_SENSORS_35CM : SEG_SENSORS_30CM;
}

uint16_t board_seg_data_count(uint8_t edge, uint8_t seg)
{
    return (uint16_t)(board_seg_sensors(edge, seg) * SENSOR_UNIT);
}

uint16_t board_seg_prefix(uint8_t edge, uint8_t seg)
{
    uint16_t n = 0;
    for (uint8_t s = 0; s < seg; s++) {
        n = (uint16_t)(n + board_seg_data_count(edge, s));
    }
    return n;
}

uint16_t board_edge_sensors(uint8_t edge)
{
    uint16_t n = 0;
    for (uint8_t s = 0; s < board_edge_seg_total(edge); s++) {
        n = (uint16_t)(n + board_seg_sensors(edge, s));
    }
    return n;
}

uint16_t board_edge_data_count(uint8_t edge)
{
    return (uint16_t)(board_edge_sensors(edge) * SENSOR_UNIT);
}
