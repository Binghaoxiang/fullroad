#include "runtime_params.h"
#include "config.h"
#include "bsp_uart.h"
#include "board_map.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stddef.h>

#define CMD_HEADER_0 0x55u
#define CMD_HEADER_1 0xAAu
#define CMD_VERSION  2u
#define CMD_SET_PARAMS 1u
#define CMD_CALIB_CAPTURE 2u
#define CMD_CALIB_CLEAR 3u
#define CMD_CALIB_APPLY 4u
#define CMD_CALIB_IMPORT_BEGIN 5u
#define CMD_CALIB_IMPORT_EDGE  6u
#define CMD_CALIB_IMPORT_COMMIT 7u
#define CMD_SET_LEN 28u
#define CMD_CAPTURE_LEN 17u
#define CMD_IMPORT_EDGE_MAX_LEN (10u + 8u * CALIB_MAX_POINTS)

#define CALIB_FLASH_ADDRESS 0x0801F800u
#define CALIB_FLASH_PAGE    63u
#define CALIB_FLASH_MAGIC   0x354C4143u /* "CAL5" */
#define CALIB_FLASH_VERSION 5u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint8_t valid_mask;
    uint8_t state;
    uint8_t point_count[8];
    uint8_t reserved[2];
    float rms[8];
    float raw[8][CALIB_MAX_POINTS];
    float mm[8][CALIB_MAX_POINTS];
    uint32_t crc32;
} CalibrationFlashRecord;

typedef char calibration_record_fits_page[
    (sizeof(CalibrationFlashRecord) <= FLASH_PAGE_SIZE &&
     (sizeof(CalibrationFlashRecord) & 7u) == 0u) ? 1 : -1];

typedef struct {
    float values[CALIB_CAPTURE_MAX_FRAMES];
    uint8_t n;
} CaptureEdge;

static RuntimeParams p;
static uint8_t rx[CMD_IMPORT_EDGE_MAX_LEN];
static uint8_t rx_n;
static float capture_x, capture_y;
static CaptureEdge capture_edge[8];
/* A full grid observes the same X node at several Y positions (and vice
   versa).  Combine those firmware-side repetitions instead of allowing the
   last grid point to overwrite all earlier observations of that node. */
static uint8_t calibration_node_hits[8][CALIB_MAX_POINTS];
/* Let the One-Euro centroid filters settle after the operator moves the
   calibration laser.  These frames are deliberately not counted as samples. */
static uint8_t calibration_settle_pending;
/* UI disaster-recovery import is staged here.  Active calibration and its
   Flash record are untouched until all eight edges arrive and pass the same
   firmware-side validation used by a normal Apply operation. */
static uint8_t calibration_import_mask;
static uint8_t calibration_import_count[8];
static float calibration_import_raw[8][CALIB_MAX_POINTS];
static float calibration_import_mm[8][CALIB_MAX_POINTS];
#if MCU_IS_MASTER
static CalibrationFlashRecord flash_record;
#endif
static uint8_t algorithm_reset_pending;

/* Joint calibration is rebuilt from the persisted per-edge nodes at boot or
 * Apply time.  It deliberately is not stored in Flash, so existing CAL5
 * records (including the current 21-position calibration) remain compatible.
 *
 * Each axis has one physical coordinate.  The four detector strips share one
 * scale and one residual shape; only their installation offsets are allowed
 * to differ.  A Huber fit prevents one bad node from bending an entire edge.
 */
typedef struct {
    uint8_t valid;
    uint8_t node_count;
    float scale;
    float offset[8];
    float aligned_node[CALIB_MAX_POINTS];
    float target_node[CALIB_MAX_POINTS];
} JointAxisCalibration;

static JointAxisCalibration joint_axis[2];
static float joint_slope_samples[4u * CALIB_MAX_POINTS * (CALIB_MAX_POINTS - 1u) / 2u];
static const uint8_t joint_axis_edges[2][4] = {
    { EDGE_UP, EDGE_DOWN, EDGE_IN_UP, EDGE_IN_DOWN },
    { EDGE_LEFT, EDGE_RIGHT, EDGE_IN_LEFT, EDGE_IN_RIGHT }
};

#define JOINT_CALIB_HUBER_MM       3.0f
#define JOINT_CALIB_MIN_SCALE      0.70f
#define JOINT_CALIB_MAX_SCALE      1.30f
#define JOINT_CALIB_TARGET_EPS_MM  0.25f
#define JOINT_CALIB_MIN_PAIR_MM    20.0f

static uint8_t calibration_apply_internal(uint8_t persist);

static float pcb_nominal_length(uint16_t sensors)
{
    return (sensors == SEG_SENSORS_35CM) ? PCB_35_NOMINAL_LENGTH_MM
                                         : PCB_30_NOMINAL_LENGTH_MM;
}

static float pcb_first_detector(uint16_t sensors)
{
    return (sensors == SEG_SENSORS_35CM) ? PCB_35_FIRST_DETECTOR_MM
                                         : PCB_30_FIRST_DETECTOR_MM;
}

/* 未标定时的分段物理映射。
 *
 * 不能把整条机械边均匀除以二极管数量：外侧竖边机械高度402 mm，
 * 实物却仍只有一块35 cm PCB。那样会把58颗点错误拉伸14.9%。
 * 这里按每块PCB自己的首颗位置/节距映射，再把整组PCB放到膜边中心。
 * 标定有效后，runtime_params_map_edge()仍优先使用实测分段节点。 */
static float default_map_edge(uint8_t edge, float sensor_position)
{
    uint8_t segs, seg;
    uint16_t prefix = 0;
    float assembly_len = 0.0f;
    float before_len = 0.0f;
    float center;
    float forward;
    uint8_t reverse;

    if (edge >= 8) return 0.0f;
    segs = board_edge_seg_total(edge);
    if (segs == 0u) return 0.0f;

    for (seg = 0; seg < segs; seg++)
        assembly_len += pcb_nominal_length(board_seg_sensors(edge, seg));

    for (seg = 0; seg + 1u < segs; seg++) {
        uint16_t n = board_seg_sensors(edge, seg);
        if (sensor_position < (float)(prefix + n)) break;
        prefix = (uint16_t)(prefix + n);
        before_len += pcb_nominal_length(n);
    }
    if (seg >= segs) seg = (uint8_t)(segs - 1u);

    center = (edge == EDGE_UP || edge == EDGE_DOWN ||
              edge == EDGE_IN_UP || edge == EDGE_IN_DOWN)
           ? 0.5f * SCREEN_W : 0.5f * SCREEN_H;
    forward = center - 0.5f * assembly_len + before_len +
              pcb_first_detector(board_seg_sensors(edge, seg)) +
              (sensor_position - (float)prefix) * DETECTOR_PITCH_MM;

    reverse = (edge == EDGE_RIGHT || edge == EDGE_IN_RIGHT) ? 1u : 0u;
    return reverse ? (2.0f * center - forward) : forward;
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

#if MCU_IS_MASTER
static uint32_t crc32_ieee(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}
#endif

static void set_default_edge_ranges(void)
{
    uint8_t edge;
    for (edge = 0; edge < 8; edge++) {
        uint16_t count = board_edge_sensors(edge);
        p.edge_min[edge] = default_map_edge(edge, 0.0f);
        p.edge_max[edge] = default_map_edge(
            edge, count > 0u ? (float)(count - 1u) : 0.0f);
    }
}

static uint8_t calibration_flash_erase(void)
{
#if MCU_IS_MASTER
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0xFFFFFFFFu;
    HAL_StatusTypeDef status;

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = CALIB_FLASH_PAGE;
    erase.NbPages = 1;
    if (HAL_FLASH_Unlock() != HAL_OK) return 0;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    status = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
    return (status == HAL_OK && page_error == 0xFFFFFFFFu) ? 1u : 0u;
#else
    return 1u;
#endif
}

static uint8_t calibration_flash_save(void)
{
#if MCU_IS_MASTER
    HAL_StatusTypeDef status = HAL_OK;
    const uint8_t *src;
    uint32_t offset;

    memset(&flash_record, 0, sizeof(flash_record));
    flash_record.magic = CALIB_FLASH_MAGIC;
    flash_record.version = CALIB_FLASH_VERSION;
    flash_record.length = (uint16_t)sizeof(flash_record);
    flash_record.valid_mask = p.calibration_valid_mask;
    flash_record.state = p.calibration_state;
    memcpy(flash_record.point_count, p.calibration_point_count,
           sizeof(flash_record.point_count));
    memcpy(flash_record.rms, p.calibration_rms, sizeof(flash_record.rms));
    memcpy(flash_record.raw, p.calibration_raw, sizeof(flash_record.raw));
    memcpy(flash_record.mm, p.calibration_mm, sizeof(flash_record.mm));
    flash_record.crc32 = crc32_ieee((const uint8_t *)&flash_record,
                                    (uint32_t)offsetof(CalibrationFlashRecord, crc32));

    if (!calibration_flash_erase()) return 0;
    if (HAL_FLASH_Unlock() != HAL_OK) return 0;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    src = (const uint8_t *)&flash_record;
    for (offset = 0; offset < sizeof(flash_record); offset += 8u) {
        uint64_t word;
        memcpy(&word, &src[offset], sizeof(word));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   CALIB_FLASH_ADDRESS + offset, word);
        if (status != HAL_OK) break;
    }
    HAL_FLASH_Lock();
    if (status != HAL_OK) return 0;
    return memcmp((const void *)CALIB_FLASH_ADDRESS,
                  &flash_record, sizeof(flash_record)) == 0 ? 1u : 0u;
#else
    return 1u;
#endif
}

static uint8_t calibration_flash_load(void)
{
#if MCU_IS_MASTER
    uint8_t edge;
    memcpy(&flash_record, (const void *)CALIB_FLASH_ADDRESS, sizeof(flash_record));
    if (flash_record.magic != CALIB_FLASH_MAGIC ||
        flash_record.version != CALIB_FLASH_VERSION ||
        flash_record.length != sizeof(flash_record) ||
        flash_record.crc32 != crc32_ieee((const uint8_t *)&flash_record,
                                         (uint32_t)offsetof(CalibrationFlashRecord, crc32)))
        return 0;
    for (edge = 0; edge < 8; edge++) {
        if (flash_record.point_count[edge] > CALIB_MAX_POINTS) return 0;
    }
    memcpy(p.calibration_point_count, flash_record.point_count,
           sizeof(p.calibration_point_count));
    memcpy(p.calibration_samples, flash_record.point_count,
           sizeof(p.calibration_samples));
    memcpy(p.calibration_rms, flash_record.rms, sizeof(p.calibration_rms));
    memcpy(p.calibration_raw, flash_record.raw, sizeof(p.calibration_raw));
    memcpy(p.calibration_mm, flash_record.mm, sizeof(p.calibration_mm));
    p.calibration_valid_mask = flash_record.valid_mask;
    return calibration_apply_internal(0u) != 0u;
#else
    return 0u;
#endif
}

static float clampf(float v, float lo, float hi, float fallback)
{
    if (!isfinite(v)) return fallback;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float edge_target(uint8_t edge)
{
    if (edge == EDGE_UP || edge == EDGE_DOWN ||
        edge == EDGE_IN_UP || edge == EDGE_IN_DOWN)
        return capture_x;
    return capture_y;
}

static uint8_t calibration_min_points(uint8_t edge)
{
    return (edge == EDGE_UP || edge == EDGE_DOWN ||
            edge == EDGE_IN_UP || edge == EDGE_IN_DOWN)
               ? CALIB_MIN_POINTS_LONG : CALIB_MIN_POINTS_SHORT;
}

static void sort_float(float *v, uint8_t n)
{
    uint8_t i;
    for (i = 1; i < n; i++) {
        float key = v[i];
        int j = (int)i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }
}

static float median_small(float *v, uint8_t n)
{
    if (n == 0u) return NAN;
    sort_float(v, n);
    return (n & 1u) ? v[n / 2u]
                    : 0.5f * (v[n / 2u - 1u] + v[n / 2u]);
}

static int8_t find_target_node(uint8_t edge, float target)
{
    uint8_t i, n = p.calibration_point_count[edge];
    int8_t best = -1;
    float best_err = JOINT_CALIB_TARGET_EPS_MM;
    for (i = 0u; i < n; i++) {
        float err = fabsf(p.calibration_mm[edge][i] - target);
        if (err <= best_err) {
            best_err = err;
            best = (int8_t)i;
        }
    }
    return best;
}

/* Fixed-effects robust regression:
 *     target_mm = shared_scale * nominal_physical_mm + edge_offset
 * Using default_map_edge() as the independent variable preserves the known
 * 30+35 / 35+30 PCB seam geometry while jointly constraining all four strips.
 */
static uint8_t joint_axis_build(uint8_t axis)
{
    JointAxisCalibration *m = &joint_axis[axis];
    float weight[4][CALIB_MAX_POINTS];
    uint8_t slot, i, iteration;

    memset(m, 0, sizeof(*m));
    memset(weight, 0, sizeof(weight));
    for (slot = 0u; slot < 4u; slot++) {
        uint8_t edge = joint_axis_edges[axis][slot];
        uint8_t n = p.calibration_point_count[edge];
        if (n < calibration_min_points(edge)) return 0u;
        for (i = 0u; i < n; i++) weight[slot][i] = 1.0f;
    }

    m->scale = 1.0f;
    for (iteration = 0u; iteration < 5u; iteration++) {
        float numerator = 0.0f, denominator = 0.0f;

        /* Eliminate the per-edge intercept by centering each strip. */
        for (slot = 0u; slot < 4u; slot++) {
            uint8_t edge = joint_axis_edges[axis][slot];
            uint8_t n = p.calibration_point_count[edge];
            float sw = 0.0f, sq = 0.0f, sy = 0.0f;
            for (i = 0u; i < n; i++) {
                float w = weight[slot][i];
                float q = default_map_edge(edge, p.calibration_raw[edge][i]);
                sw += w; sq += w * q; sy += w * p.calibration_mm[edge][i];
            }
            if (sw <= 1e-6f) return 0u;
            sq /= sw; sy /= sw;
            for (i = 0u; i < n; i++) {
                float w = weight[slot][i];
                float q = default_map_edge(edge, p.calibration_raw[edge][i]) - sq;
                float y = p.calibration_mm[edge][i] - sy;
                numerator += w * q * y;
                denominator += w * q * q;
            }
        }
        if (denominator <= 1e-6f) return 0u;
        m->scale = numerator / denominator;
        if (!isfinite(m->scale) || m->scale < JOINT_CALIB_MIN_SCALE ||
            m->scale > JOINT_CALIB_MAX_SCALE) return 0u;

        for (slot = 0u; slot < 4u; slot++) {
            uint8_t edge = joint_axis_edges[axis][slot];
            uint8_t n = p.calibration_point_count[edge];
            float sw = 0.0f, residual_sum = 0.0f;
            for (i = 0u; i < n; i++) {
                float w = weight[slot][i];
                float q = default_map_edge(edge, p.calibration_raw[edge][i]);
                sw += w;
                residual_sum += w * (p.calibration_mm[edge][i] - m->scale * q);
            }
            if (sw <= 1e-6f) return 0u;
            m->offset[edge] = residual_sum / sw;
        }

        /* Reweight only after obtaining the first complete solution. */
        for (slot = 0u; slot < 4u; slot++) {
            uint8_t edge = joint_axis_edges[axis][slot];
            uint8_t n = p.calibration_point_count[edge];
            for (i = 0u; i < n; i++) {
                float q = default_map_edge(edge, p.calibration_raw[edge][i]);
                float residual = fabsf(m->scale * q + m->offset[edge] -
                                       p.calibration_mm[edge][i]);
                weight[slot][i] = (residual <= JOINT_CALIB_HUBER_MM ||
                                   residual <= 1e-6f)
                                ? 1.0f : JOINT_CALIB_HUBER_MM / residual;
            }
        }
    }

    /* Use the first strip's target list only as the list of commanded grid
       coordinates.  At each coordinate the median of all four aligned strips
       creates the shared residual curve. */
    {
        uint8_t reference = joint_axis_edges[axis][0];
        uint8_t n = p.calibration_point_count[reference];
        float targets[CALIB_MAX_POINTS];
        for (i = 0u; i < n; i++) targets[i] = p.calibration_mm[reference][i];
        sort_float(targets, n);
        for (i = 0u; i < n; i++) {
            float aligned[4];
            uint8_t found = 0u;
            for (slot = 0u; slot < 4u; slot++) {
                uint8_t edge = joint_axis_edges[axis][slot];
                int8_t node = find_target_node(edge, targets[i]);
                if (node >= 0) {
                    float q = default_map_edge(edge,
                                  p.calibration_raw[edge][(uint8_t)node]);
                    aligned[found++] = m->scale * q + m->offset[edge];
                }
            }
            if (found < 3u) return 0u;
            m->aligned_node[i] = median_small(aligned, found);
            m->target_node[i] = targets[i];
            if (i > 0u &&
                m->aligned_node[i] - m->aligned_node[i - 1u] < 1.0f)
                return 0u;
        }
        m->node_count = n;
    }
    m->valid = 1u;
    return 1u;
}

/* The short axis currently has only three commanded Y positions per strip.
 * With so few nodes, least squares can still be bent by one bad capture (the
 * present IN_RIGHT record contains exactly that failure).  Estimate the
 * common short-edge scale from the median of all valid pairwise slopes, then
 * estimate every strip's installation offset by another median.  One bad
 * node on one strip therefore cannot move the fitted Y coordinate.
 *
 * The seven-node X model is intentionally left on the smoother Huber fit. */
static uint8_t joint_short_axis_build(void)
{
    JointAxisCalibration *m = &joint_axis[1];
    uint16_t slope_n = 0u;
    uint8_t slot, i;

    memset(m, 0, sizeof(*m));
    for (slot = 0u; slot < 4u; slot++) {
        uint8_t edge = joint_axis_edges[1][slot];
        uint8_t n = p.calibration_point_count[edge];
        if (n < calibration_min_points(edge)) return 0u;
        for (i = 0u; i + 1u < n; i++) {
            uint8_t j;
            float qi = default_map_edge(edge, p.calibration_raw[edge][i]);
            for (j = (uint8_t)(i + 1u); j < n; j++) {
                float qj = default_map_edge(edge, p.calibration_raw[edge][j]);
                float dq = qj - qi;
                float dy = p.calibration_mm[edge][j] - p.calibration_mm[edge][i];
                float slope;
                if (fabsf(dq) < JOINT_CALIB_MIN_PAIR_MM) continue;
                slope = dy / dq;
                if (isfinite(slope) && slope >= JOINT_CALIB_MIN_SCALE &&
                    slope <= JOINT_CALIB_MAX_SCALE)
                    joint_slope_samples[slope_n++] = slope;
            }
        }
    }
    if (slope_n < 4u || slope_n > 255u) return 0u;
    sort_float(joint_slope_samples, (uint8_t)slope_n);
    m->scale = (slope_n & 1u)
             ? joint_slope_samples[slope_n / 2u]
             : 0.5f * (joint_slope_samples[slope_n / 2u - 1u] +
                       joint_slope_samples[slope_n / 2u]);

    for (slot = 0u; slot < 4u; slot++) {
        uint8_t edge = joint_axis_edges[1][slot];
        uint8_t n = p.calibration_point_count[edge];
        float residual[CALIB_MAX_POINTS];
        for (i = 0u; i < n; i++) {
            float q = default_map_edge(edge, p.calibration_raw[edge][i]);
            residual[i] = p.calibration_mm[edge][i] - m->scale * q;
        }
        m->offset[edge] = median_small(residual, n);
    }

    {
        uint8_t reference = joint_axis_edges[1][0];
        uint8_t n = p.calibration_point_count[reference];
        float targets[CALIB_MAX_POINTS];
        for (i = 0u; i < n; i++) targets[i] = p.calibration_mm[reference][i];
        sort_float(targets, n);
        for (i = 0u; i < n; i++) {
            float aligned[4];
            uint8_t found = 0u;
            for (slot = 0u; slot < 4u; slot++) {
                uint8_t edge = joint_axis_edges[1][slot];
                int8_t node = find_target_node(edge, targets[i]);
                if (node >= 0) {
                    float q = default_map_edge(edge,
                                  p.calibration_raw[edge][(uint8_t)node]);
                    aligned[found++] = m->scale * q + m->offset[edge];
                }
            }
            if (found < 3u) return 0u;
            m->aligned_node[i] = median_small(aligned, found);
            m->target_node[i] = targets[i];
            if (i > 0u &&
                m->aligned_node[i] - m->aligned_node[i - 1u] < 1.0f)
                return 0u;
        }
        m->node_count = n;
    }
    m->valid = 1u;
    return 1u;
}

static void joint_calibration_build(void)
{
    memset(joint_axis, 0, sizeof(joint_axis));
    if (p.calibration_valid_mask != 0xFFu) return;
    joint_axis_build(0u);
    joint_short_axis_build();
}

static float joint_axis_map(uint8_t axis, uint8_t edge, float sensor_position)
{
    const JointAxisCalibration *m = &joint_axis[axis];
    float q, z;
    uint8_t i, n;
    if (!m->valid) return NAN;
    n = m->node_count;
    if (n < 2u) return NAN;
    q = default_map_edge(edge, sensor_position);
    z = m->scale * q + m->offset[edge];
    if (z <= m->aligned_node[0]) i = 0u;
    else if (z >= m->aligned_node[n - 1u]) i = (uint8_t)(n - 2u);
    else {
        for (i = 0u; i + 1u < n; i++)
            if (z <= m->aligned_node[i + 1u]) break;
        if (i + 1u >= n) i = (uint8_t)(n - 2u);
    }
    {
        float dz = m->aligned_node[i + 1u] - m->aligned_node[i];
        if (fabsf(dz) < 0.05f) return NAN;
        return m->target_node[i] +
               (z - m->aligned_node[i]) *
               (m->target_node[i + 1u] - m->target_node[i]) / dz;
    }
}

static void sort_nodes(uint8_t edge)
{
    uint8_t i, j, n = p.calibration_point_count[edge];
    for (i = 1; i < n; i++) {
        float raw = p.calibration_raw[edge][i];
        float mm = p.calibration_mm[edge][i];
        uint8_t hits = calibration_node_hits[edge][i];
        j = i;
        while (j > 0 && p.calibration_raw[edge][j - 1] > raw) {
            p.calibration_raw[edge][j] = p.calibration_raw[edge][j - 1];
            p.calibration_mm[edge][j] = p.calibration_mm[edge][j - 1];
            calibration_node_hits[edge][j] = calibration_node_hits[edge][j - 1];
            j--;
        }
        p.calibration_raw[edge][j] = raw;
        p.calibration_mm[edge][j] = mm;
        calibration_node_hits[edge][j] = hits;
    }
}

static float map_nodes_excluding(uint8_t edge, float sensor_position, int8_t skip)
{
    uint8_t ids[CALIB_MAX_POINTS], n = 0, i;
    uint8_t total = p.calibration_point_count[edge];
    for (i = 0; i < total; i++) if ((int8_t)i != skip) ids[n++] = i;
    if (n < 2) return NAN;

    if (sensor_position <= p.calibration_raw[edge][ids[0]]) i = 0;
    else if (sensor_position >= p.calibration_raw[edge][ids[n - 1]]) i = (uint8_t)(n - 2);
    else {
        for (i = 0; i + 1 < n; i++)
            if (sensor_position <= p.calibration_raw[edge][ids[i + 1]]) break;
        if (i + 1 >= n) i = (uint8_t)(n - 2);
    }
    {
        uint8_t a = ids[i], b = ids[i + 1];
        float dx = p.calibration_raw[edge][b] - p.calibration_raw[edge][a];
        if (fabsf(dx) < 0.05f) return NAN;
        return p.calibration_mm[edge][a] +
               (sensor_position - p.calibration_raw[edge][a]) *
               (p.calibration_mm[edge][b] - p.calibration_mm[edge][a]) / dx;
    }
}

float runtime_params_map_edge(uint8_t edge, float sensor_position)
{
    if (edge >= 8) return 0.0f;
    /* Coordinate calibration is one coupled eight-edge system.  Never mix a
       calibrated curve on one detector edge with the mechanical default on
       another: that creates an artificial inner/outer displacement even for
       a perpendicular cross.  A partial mask remains available for UI
       diagnosis, but becomes active only after all eight edges pass. */
    if (p.calibration_valid_mask == 0xFFu &&
        p.calibration_point_count[edge] >= 2) {
        uint8_t axis = (edge == EDGE_UP || edge == EDGE_DOWN ||
                        edge == EDGE_IN_UP || edge == EDGE_IN_DOWN) ? 0u : 1u;
        float joint = joint_axis_map(axis, edge, sensor_position);
        if (isfinite(joint)) return joint;
        float v = map_nodes_excluding(edge, sensor_position, -1);
        if (isfinite(v)) return v;
    }
    return default_map_edge(edge, sensor_position);
}

static void reset_calibration(void)
{
    set_default_edge_ranges();
    memset(p.calibration_samples, 0, sizeof(p.calibration_samples));
    memset(p.calibration_point_count, 0, sizeof(p.calibration_point_count));
    memset(p.calibration_last_samples, 0, sizeof(p.calibration_last_samples));
    memset(p.calibration_rms, 0, sizeof(p.calibration_rms));
    memset(p.calibration_raw, 0, sizeof(p.calibration_raw));
    memset(p.calibration_mm, 0, sizeof(p.calibration_mm));
    memset(capture_edge, 0, sizeof(capture_edge));
    memset(calibration_node_hits, 0, sizeof(calibration_node_hits));
    p.calibration_valid_mask = 0;
    p.calibration_state = 0;
    p.calibration_pending = 0;
    p.calibration_requested = 0;
    calibration_settle_pending = 0;
    memset(joint_axis, 0, sizeof(joint_axis));
}

void runtime_params_init(void)
{
    memset(&p, 0, sizeof(p));
    p.centroid_filter_enable = CENTROID_FILTER_ENABLE ? 1u : 0u;
    p.position_filter_enable = 1u;
    p.centroid_threshold = CENTROID_THRESHOLD;
    p.position_outlier_mm = POSITION_OUTLIER_MM;
    p.cross_min_conf = POSITION_CROSS_MIN_CONF;
    p.position_jump_mm = KALMAN_JUMP_MM;
    reset_calibration();
    calibration_flash_load();
    algorithm_reset_pending = 0;
    calibration_import_mask = 0;
    rx_n = 0;
}

const RuntimeParams *runtime_params_get(void) { return &p; }

void runtime_params_encode(uint8_t *out)
{
    uint16_t idx = 0;
    out[idx++] = p.centroid_filter_enable;
    out[idx++] = p.position_filter_enable;
    out[idx++] = p.calibration_state;
    out[idx++] = p.calibration_valid_mask;
    memcpy(&out[idx], &p.centroid_threshold, 4); idx += 4;
    memcpy(&out[idx], &p.position_outlier_mm, 4); idx += 4;
    memcpy(&out[idx], &p.cross_min_conf, 4); idx += 4;
    memcpy(&out[idx], &p.position_jump_mm, 4); idx += 4;
    memcpy(&out[idx], p.edge_min, 8 * 4); idx += 8 * 4;
    memcpy(&out[idx], p.edge_max, 8 * 4); idx += 8 * 4;
    memcpy(&out[idx], p.calibration_samples, 8); idx += 8;
    memcpy(&out[idx], p.calibration_point_count, 8); idx += 8;
    memcpy(&out[idx], p.calibration_last_samples, 8); idx += 8;
    out[idx++] = p.calibration_pending;
    out[idx++] = p.calibration_requested;
    out[idx++] = 0;
    out[idx++] = 0;
    memcpy(&out[idx], p.calibration_rms, 8 * 4); idx += 8 * 4;
    memcpy(&out[idx], p.calibration_raw, 8 * CALIB_MAX_POINTS * 4); idx += 8 * CALIB_MAX_POINTS * 4;
    memcpy(&out[idx], p.calibration_mm, 8 * CALIB_MAX_POINTS * 4);
}

static void finish_capture(void)
{
    uint8_t edge;
    for (edge = 0; edge < 8; edge++) {
        CaptureEdge *ce = &capture_edge[edge];
        uint8_t n = ce->n;
        p.calibration_last_samples[edge] = n;
        if (n < CALIB_CAPTURE_MIN_VALID_FRAMES) continue;
        sort_float(ce->values, n);
        {
            float raw = ce->values[n / 2];
            float target = edge_target(edge);
            uint8_t count = p.calibration_point_count[edge];
            uint8_t replace = 0xFF, i;
            for (i = 0; i < count; i++) {
                if (fabsf(p.calibration_mm[edge][i] - target) <= CALIB_TARGET_REPLACE_MM) {
                    replace = i;
                    break;
                }
            }
            if (replace == 0xFF && count < CALIB_MAX_POINTS) replace = count++;
            if (replace == 0xFF) continue;
            if (calibration_node_hits[edge][replace] > 0u) {
                uint8_t hits = calibration_node_hits[edge][replace];
                p.calibration_raw[edge][replace] =
                    (p.calibration_raw[edge][replace] * (float)hits + raw) /
                    (float)(hits + 1u);
                if (hits < 254u) calibration_node_hits[edge][replace] = hits + 1u;
            } else {
                p.calibration_raw[edge][replace] = raw;
                calibration_node_hits[edge][replace] = 1u;
            }
            p.calibration_mm[edge][replace] = target;
            p.calibration_point_count[edge] = count;
            p.calibration_samples[edge] = count;
            sort_nodes(edge);
        }
    }
    p.calibration_pending = 0;
    p.calibration_state = 2;
}

void runtime_params_calibration_update(const Centroids *centroids)
{
    const Centroid *all[8] = {
        &centroids->up, &centroids->down, &centroids->left, &centroids->right,
        &centroids->in_up, &centroids->in_down,
        &centroids->in_left, &centroids->in_right
    };
    uint8_t edge;
    if (!p.calibration_pending) return;
    if (calibration_settle_pending) {
        calibration_settle_pending--;
        return;
    }
    for (edge = 0; edge < 8; edge++) {
        const Centroid *c = all[edge];
        CaptureEdge *ce = &capture_edge[edge];
        if (ce->n >= CALIB_CAPTURE_MAX_FRAMES) continue;
        if (c->valid && isfinite(c->position) && c->peak >= CALIB_MIN_PEAK)
            ce->values[ce->n++] = c->position;
    }
    if (p.calibration_pending > 0) p.calibration_pending--;
    if (p.calibration_pending == 0) finish_capture();
}

static uint8_t calibration_apply_internal(uint8_t persist)
{
    uint8_t edge, valid_mask = 0;
    set_default_edge_ranges();
    for (edge = 0; edge < 8; edge++) {
        uint8_t n = p.calibration_point_count[edge], i;
        float err2 = 0.0f;
        uint8_t ok = (n >= calibration_min_points(edge));
        if (ok) {
            int8_t sign = 0;
            for (i = 1; i < n; i++) {
                float dr = p.calibration_raw[edge][i] - p.calibration_raw[edge][i - 1];
                float dm = p.calibration_mm[edge][i] - p.calibration_mm[edge][i - 1];
                int8_t s = (dm > 0.0f) ? 1 : (dm < 0.0f ? -1 : 0);
                if (dr < CALIB_MIN_RAW_SPACING || s == 0 || (sign != 0 && s != sign)) {
                    ok = 0;
                    break;
                }
                sign = s;
            }
        }
        if (ok) {
            for (i = 0; i < n; i++) {
                float predicted = map_nodes_excluding(edge, p.calibration_raw[edge][i], (int8_t)i);
                float err = predicted - p.calibration_mm[edge][i];
                if (!isfinite(predicted)) { ok = 0; break; }
                err2 += err * err;
            }
        }
        if (ok) {
            p.calibration_rms[edge] = sqrtf(err2 / (float)n);
            /* During commissioning RMS is diagnostic, not an application
               gate.  Complete monotonic finite nodes are safe to map; UI
               exposes RMS and cross-edge spread so alignment can be improved
               without making the first observable result impossible. */
            valid_mask |= (uint8_t)(1u << edge);
            p.edge_min[edge] = map_nodes_excluding(edge, 0.0f, -1);
            {
                uint16_t count = board_edge_sensors(edge);
                p.edge_max[edge] = map_nodes_excluding(
                    edge, count > 0u ? (float)(count - 1u) : 0.0f, -1);
            }
        }
        if (!ok) p.calibration_rms[edge] = -1.0f;
#if CALIB_ACCEPT_CURRENT_FOR_TEST
        /* Bench-only override: preserve the measured nodes and diagnostics,
           but allow a complete data set to participate in the coupled map. */
        if (n >= calibration_min_points(edge))
            valid_mask |= (uint8_t)(1u << edge);
#endif
    }
    p.calibration_valid_mask = valid_mask;
    joint_calibration_build();
    p.calibration_state = (valid_mask == 0xFFu) ? 3u : 4u;
    if (persist && valid_mask == 0xFFu && !calibration_flash_save())
        p.calibration_state = 5u;
    if (persist) algorithm_reset_pending = 1u;
    return valid_mask;
}

uint8_t runtime_params_take_algorithm_reset(void)
{
    uint8_t pending = algorithm_reset_pending;
    algorithm_reset_pending = 0;
    return pending;
}

static void apply_set(const uint8_t *cmd)
{
    RuntimeParams next = p;
    next.centroid_filter_enable = cmd[4] ? 1u : 0u;
    next.position_filter_enable = cmd[5] ? 1u : 0u;
    memcpy(&next.centroid_threshold, &cmd[8], 4);
    memcpy(&next.position_outlier_mm, &cmd[12], 4);
    memcpy(&next.cross_min_conf, &cmd[16], 4);
    memcpy(&next.position_jump_mm, &cmd[20], 4);
    next.centroid_threshold = clampf(next.centroid_threshold, 0.0001f, 0.5f, p.centroid_threshold);
    next.position_outlier_mm = clampf(next.position_outlier_mm, 1.0f, 200.0f, p.position_outlier_mm);
    next.cross_min_conf = clampf(next.cross_min_conf, 0.0f, 1.0f, p.cross_min_conf);
    next.position_jump_mm = clampf(next.position_jump_mm, 1.0f, 500.0f, p.position_jump_mm);
    p = next;
}

void runtime_params_poll(void)
{
    uint8_t b;
    while (bsp_uart_read_byte(&b)) {
        uint8_t needed;
        if (rx_n == 0 && b != CMD_HEADER_0) continue;
        if (rx_n == 1 && b != CMD_HEADER_1) { rx_n = (b == CMD_HEADER_0) ? 1u : 0u; continue; }
        rx[rx_n++] = b;
        if (rx_n < 4) continue;
        if (rx[3] < CMD_SET_PARAMS || rx[3] > CMD_CALIB_IMPORT_COMMIT) {
            rx_n = 0;
            continue;
        }
        if (rx[3] == CMD_CALIB_IMPORT_EDGE) {
            uint8_t count;
            if (rx_n < 6u) continue;
            count = rx[5];
            if (rx[4] >= 8u || count < 2u || count > CALIB_MAX_POINTS) {
                rx_n = 0;
                continue;
            }
            needed = (uint8_t)(10u + 8u * count);
        } else {
            needed = (rx[3] == CMD_SET_PARAMS) ? CMD_SET_LEN :
                     (rx[3] == CMD_CALIB_CAPTURE) ? CMD_CAPTURE_LEN : 8u;
        }
        if (rx_n < needed) continue;
        if (rx[2] == CMD_VERSION &&
            crc16_ccitt(&rx[2], (uint16_t)(needed - 6)) ==
                (uint16_t)(rx[needed - 4] | ((uint16_t)rx[needed - 3] << 8)) &&
            rx[needed - 2] == 0xFEu && rx[needed - 1] == 0xFDu) {
            if (rx[3] == CMD_SET_PARAMS) apply_set(rx);
            else if (rx[3] == CMD_CALIB_CAPTURE) {
                uint8_t requested;
                memcpy(&capture_x, &rx[4], 4);
                memcpy(&capture_y, &rx[8], 4);
                requested = rx[12] ? rx[12] : 1u;
                if (requested > CALIB_CAPTURE_MAX_FRAMES) requested = CALIB_CAPTURE_MAX_FRAMES;
                if (isfinite(capture_x) && isfinite(capture_y) &&
                    capture_x >= 0.0f && capture_x <= SCREEN_W &&
                    capture_y >= 0.0f && capture_y <= SCREEN_H) {
                    memset(capture_edge, 0, sizeof(capture_edge));
                    memset(p.calibration_last_samples, 0, sizeof(p.calibration_last_samples));
                    p.calibration_requested = requested;
                    p.calibration_pending = requested;
                    calibration_settle_pending = CALIB_SETTLE_FRAMES;
                    p.calibration_state = 1;
                }
            } else if (rx[3] == CMD_CALIB_CLEAR) {
                reset_calibration();
                if (!calibration_flash_erase()) p.calibration_state = 6u;
                algorithm_reset_pending = 1u;
            } else if (rx[3] == CMD_CALIB_APPLY) {
                calibration_apply_internal(1u);
            } else if (rx[3] == CMD_CALIB_IMPORT_BEGIN) {
                calibration_import_mask = 0u;
                memset(calibration_import_count, 0, sizeof(calibration_import_count));
                memset(calibration_import_raw, 0, sizeof(calibration_import_raw));
                memset(calibration_import_mm, 0, sizeof(calibration_import_mm));
            } else if (rx[3] == CMD_CALIB_IMPORT_EDGE) {
                uint8_t edge = rx[4];
                uint8_t count = rx[5];
                memcpy(calibration_import_raw[edge], &rx[6], (size_t)count * 4u);
                memcpy(calibration_import_mm[edge], &rx[6u + (uint16_t)count * 4u],
                       (size_t)count * 4u);
                calibration_import_count[edge] = count;
                calibration_import_mask |= (uint8_t)(1u << edge);
            } else if (rx[3] == CMD_CALIB_IMPORT_COMMIT &&
                       calibration_import_mask == 0xFFu) {
                uint8_t edge;
                reset_calibration();
                for (edge = 0; edge < 8u; edge++) {
                    uint8_t count = calibration_import_count[edge];
                    p.calibration_point_count[edge] = count;
                    p.calibration_samples[edge] = count;
                    memcpy(p.calibration_raw[edge], calibration_import_raw[edge],
                           (size_t)count * 4u);
                    memcpy(p.calibration_mm[edge], calibration_import_mm[edge],
                           (size_t)count * 4u);
                    sort_nodes(edge);
                }
                p.calibration_state = 2u;
                /* Never replace the recovery Flash page with a partial or
                   invalid import.  First validate all edges in RAM, then
                   persist only the complete 0xFF result. */
                calibration_apply_internal(0u);
                if (p.calibration_valid_mask == 0xFFu) {
                    if (!calibration_flash_save()) p.calibration_state = 5u;
                    algorithm_reset_pending = 1u;
                } else {
                    p.calibration_state = 4u;
                }
                calibration_import_mask = 0u;
            }
        }
        rx_n = 0;
    }
}
