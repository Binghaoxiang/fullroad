// bsp_sensor.h
#ifndef BSP_SENSOR_H
#define BSP_SENSOR_H

#include <stdint.h>

#define SENSOR_MODE_RAW      0
#define SENSOR_MODE_FILTERED 1
#define SENSOR_MODE_BOTH     2

void bsp_sensor_init(void);
void bsp_sensor_calibrate(void);
void bsp_sensor_acquire(uint16_t *buf, uint16_t count, uint8_t mode);
/*
 * Diagnostic acquisition. `before` receives the per-sensor demodulated
 * amplitude before spatial smoothing/baseline subtraction/noise gating;
 * `processed` receives the normal board output used by the product path.
 * Either pointer may be NULL.
 */
void bsp_sensor_acquire_pair(uint16_t *processed, uint16_t *before,
                             uint16_t count, uint8_t mode);
uint32_t bsp_sensor_measure_laser(uint16_t sensor_idx);
uint32_t bsp_sensor_get_laser_freq(void);
uint16_t bsp_sensor_get_laser_peak_idx(void);
void bsp_sensor_enable_default_long(void);
void bsp_sensor_align_pulse(uint16_t sensor_idx, uint32_t f_hz);
// 诊断: 全通道 DC 扫描 (TEST_FIXED_SENSOR=1 时才编译/调用)
void bsp_sensor_debug_dump(void);

#endif
