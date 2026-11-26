// board_map.h — 逐段板型表访问（混合 30/35cm 板解析用）
#ifndef BOARD_MAP_H
#define BOARD_MAP_H

#include <stdint.h>

// 某边段数
uint8_t  board_edge_seg_total(uint8_t edge);
// 单段物理传感器数(50=30cm 板, 58=35cm 板)
uint16_t board_seg_sensors(uint8_t edge, uint8_t seg);
// 单段数据槽数(含 SENSOR_UNIT, 两路模式翻倍)
uint16_t board_seg_data_count(uint8_t edge, uint8_t seg);
// 某段之前所有段的数据槽数之和(段在边缓冲内的起始偏移)
uint16_t board_seg_prefix(uint8_t edge, uint8_t seg);
// 某边总物理传感器数 / 数据槽数
uint16_t board_edge_sensors(uint8_t edge);
uint16_t board_edge_data_count(uint8_t edge);

#endif
