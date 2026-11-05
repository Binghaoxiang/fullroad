/**
  ******************************************************************************
  * @file    sensor.h
  * @brief   采集层：两块 ADC64（长边 / 短边）各扫 4x16，扣暗电流后求和，
  *          得到归一化差分 S = (长边 − 短边)/(长边 + 短边) 与总光强 P。
  ******************************************************************************
  */
#ifndef __SENSOR_H
#define __SENSOR_H

#include "config.h"
#include "adc64.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t cell[ADC64_BOARD_COUNT][ADC64_ROWS][ADC64_COLS];  /* 扣本底后的码值 */
    uint32_t total;              /* 两块板之和 */
    uint32_t sum_a;              /* 板 0：长边 */
    uint32_t sum_b;              /* 板 1：短边 */
    int32_t  s_permille;         /* (A-B)/(A+B) x1000 */
    uint8_t  valid;              /* 1 = 总光强超过阈值 */
} sensor_meas_t;

void     sensor_init(void);
void     sensor_cal_dark(void);                  /* 全遮光时标定本底 */
void     sensor_measure(sensor_meas_t *m);
uint16_t sensor_dark(uint8_t board, uint8_t row, uint8_t col);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H */
