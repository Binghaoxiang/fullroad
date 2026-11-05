/**
  ******************************************************************************
  * @file    sensor.c
  * @brief   双板采集实现：扫描 + 暗电流扣除 + 掩码求和
  ******************************************************************************
  */
#include "sensor.h"
#include "board.h"

static uint16_t s_dark[ADC64_BOARD_COUNT][ADC64_ROWS][ADC64_COLS];

void sensor_init(void)
{
    uint8_t b;
    uint8_t r;
    uint8_t c;

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        for (r = 0U; r < ADC64_ROWS; r++) {
            for (c = 0U; c < ADC64_COLS; c++) {
                s_dark[b][r][c] = 0U;
            }
        }
    }
    adc64_init();
}

/* 遮光标定：EN 全关扫 4 次取平均，作为每点本底 */
void sensor_cal_dark(void)
{
    adc64_frame_t f;
    uint8_t b;
    uint8_t r;
    uint8_t c;
    uint8_t k;

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        for (r = 0U; r < ADC64_ROWS; r++) {
            for (c = 0U; c < ADC64_COLS; c++) {
                s_dark[b][r][c] = 0U;
            }
        }
    }
    for (k = 0U; k < 4U; k++) {
        adc64_scan_dark(&f);
        for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
            for (r = 0U; r < ADC64_ROWS; r++) {
                for (c = 0U; c < ADC64_COLS; c++) {
                    s_dark[b][r][c] = (uint16_t)(s_dark[b][r][c]
                                     + (uint16_t)(f.cell[b][r][c] / 4U));
                }
            }
        }
    }
}

void sensor_measure(sensor_meas_t *m)
{
    adc64_frame_t f;
    uint64_t mask[ADC64_BOARD_COUNT];
    uint32_t sum[ADC64_BOARD_COUNT];
    uint32_t total = 0U;
    uint8_t b;
    uint8_t r;
    uint8_t c;

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        mask[b] = adc64_mask(b);
        sum[b]  = 0U;
    }

    adc64_scan(&f);

    for (b = 0U; b < ADC64_BOARD_COUNT; b++) {
        for (r = 0U; r < ADC64_ROWS; r++) {
            for (c = 0U; c < ADC64_COLS; c++) {
                uint16_t v = f.cell[b][r][c];
                uint16_t e = (v > s_dark[b][r][c]) ? (uint16_t)(v - s_dark[b][r][c]) : 0U;
                uint32_t bit = (uint32_t)r * ADC64_COLS + (uint32_t)c;

                m->cell[b][r][c] = e;
                if (((mask[b] >> bit) & 1ULL) != 0ULL) {
                    sum[b] += (uint32_t)e;
                    total  += (uint32_t)e;
                }
            }
        }
    }

    m->total  = total;
    m->sum_a  = sum[0];
    m->sum_b  = (ADC64_BOARD_COUNT > 1U) ? sum[1] : 0U;
    m->s_permille = 0;
    if ((m->sum_a + m->sum_b) > 0U) {
        m->s_permille = (int32_t)(((int32_t)m->sum_a - (int32_t)m->sum_b) * 1000)
                      / (int32_t)(m->sum_a + m->sum_b);
    }
    m->valid = (total >= ADC64_MIN_TOTAL) ? 1U : 0U;
}

uint16_t sensor_dark(uint8_t board, uint8_t row, uint8_t col)
{
    if ((board >= ADC64_BOARD_COUNT) || (row >= ADC64_ROWS) || (col >= ADC64_COLS)) {
        return 0U;
    }
    return s_dark[board][row][col];
}
