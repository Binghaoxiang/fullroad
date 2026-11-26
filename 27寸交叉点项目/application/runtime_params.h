#ifndef RUNTIME_PARAMS_H
#define RUNTIME_PARAMS_H

#include <stdint.h>
#include "centroid.h"
#include "candidate.h"

#define CALIB_MAX_POINTS 16
#define CALIB_CAPTURE_MAX_FRAMES 32

/* Parameters that are safe and useful to tune while the product is running.
 * UI writes these values; every algorithm still executes on the MCU. */
typedef struct {
    uint8_t centroid_filter_enable;
    uint8_t position_filter_enable;
    uint8_t reserved[2];
    float centroid_threshold;
    float position_outlier_mm; /* 协议兼容保留；纯交点算法不使用 */
    float cross_min_conf;
    float position_jump_mm;
    float edge_min[8];
    float edge_max[8];
    uint8_t calibration_samples[8];
    uint8_t calibration_valid_mask;
    uint8_t calibration_point_count[8];
    uint8_t calibration_last_samples[8];
    uint8_t calibration_state;       /* 0=空闲 1=采集 2=已记录 3=已应用 4=失败 */
    uint8_t calibration_pending;
    uint8_t calibration_requested;
    float calibration_rms[8];        /* 留一节点交叉验证误差(mm) */
    float calibration_raw[8][CALIB_MAX_POINTS];
    float calibration_mm[8][CALIB_MAX_POINTS];
} RuntimeParams;

void runtime_params_init(void);
const RuntimeParams *runtime_params_get(void);
void runtime_params_poll(void);
void runtime_params_encode(uint8_t *out);
/* Calibration must observe the exact same filtered centroid estimator used by
 * product geometry and the UI.  Calibrating candidate peaks and later mapping
 * filtered centroids creates a deterministic per-edge offset. */
void runtime_params_calibration_update(const Centroids *centroids);
float runtime_params_map_edge(uint8_t edge, float sensor_position);
uint8_t runtime_params_take_algorithm_reset(void);

#endif
