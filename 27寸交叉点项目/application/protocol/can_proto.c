#include "can_proto.h"
#include "bsp_can.h"
#include "board_map.h"
#include <string.h>
#include "stm32g4xx_hal.h"
#include <stdio.h>

#define MAX_SEG 4

// 返回 <= n 的最大合法分帧传感器数（对应 FDCAN 合法字节数）
// FDCAN DLC 合法字节数: 0..8, 12, 16, 20, 24, 32, 48, 64
static uint16_t valid_chunk(uint16_t n)
{
    uint16_t bytes = n * 2;
    if (bytes >= 64) return 32;
    if (bytes >= 48) return 24;
    if (bytes >= 32) return 16;
    if (bytes >= 24) return 12;
    if (bytes >= 20) return 10;
    if (bytes >= 16) return 8;
    if (bytes >= 12) return 6;
    if (bytes >= 8)  return 4;
    if (bytes >= 6)  return 3;
    if (bytes >= 4)  return 2;
    if (bytes >= 2)  return 1;
    return 0;
}

extern UART_HandleTypeDef huart2;
#if MCU_IS_MASTER

// 第 frame 帧在段内的起始数据下标（按发送端相同的分帧规则推算，与到达顺序无关）
// count = 该段数据槽数(含 SENSOR_UNIT)，混合板时逐段不同
static uint16_t frame_start(uint8_t frame, uint16_t count)
{
    uint16_t off = 0;
    while (frame--) {
        uint16_t rem = (count > off) ? (count - off) : 0;
        if (rem > SENSORS_PER_FRAME) rem = SENSORS_PER_FRAME;
        off += valid_chunk(rem);
    }
    return off;
}

// 本段一共分几帧
static uint8_t frame_total(uint16_t count)
{
    uint16_t off = 0;
    uint8_t n = 0;
    while (off < count && n < 8) {
        uint16_t rem = count - off;
        if (rem > SENSORS_PER_FRAME) rem = SENSORS_PER_FRAME;
        off += valid_chunk(rem);
        n++;
    }
    return n;
}

/* Store edge data densely.  The old [8][512] layout reserved 8 KiB per
 * layer although this product has only 648 points.  Diagnostic mode needs
 * two layers, so the sparse layout needlessly consumed 16 KiB of 32 KiB RAM. */
static uint16_t raw[FRAME_RAW_BYTES_MAX / sizeof(uint16_t)];
static volatile uint8_t seg_frames[EDGE_COUNT][MAX_SEG];
static volatile uint8_t edge_ready[EDGE_COUNT];
static uint8_t  edge_seen = 0;   /* 曾收到过帧的边(BIT 掩码) */
static uint8_t  seg_seen[EDGE_COUNT][MAX_SEG];   /* 曾收到过帧的段(按段等待用) */
static volatile uint8_t cycle_edge_seen = 0; /* 本轮同步后实际收到的边 */
static volatile uint8_t cycle_edge_complete = 0; /* 本轮所有段/分帧完整的边 */
static volatile uint8_t cycle_accepting = 0;
/* 可由调试器在线读取，通信问题不再与无光状态混为一谈。 */
volatile uint32_t can_proto_rx_edge_frames[EDGE_COUNT];
volatile uint32_t can_proto_bad_edge_frames[EDGE_COUNT];
volatile uint32_t can_proto_late_frames;
volatile uint32_t can_proto_sync_failures;

static void reset_cycle(void)
{
    for (uint8_t e = 0; e < EDGE_COUNT; e++) {
        edge_ready[e] = 0;
        for (uint8_t s = 0; s < MAX_SEG; s++) seg_frames[e][s] = 0;
    }
    cycle_edge_seen = 0;
    cycle_edge_complete = 0;
}

static uint16_t edge_base(uint8_t edge)
{
    uint16_t base = 0;
    for (uint8_t e = 0; e < edge && e < EDGE_COUNT; e++) {
        base = (uint16_t)(base + board_edge_data_count(e));
    }
    return base;
}

static void check_edge_complete(uint8_t edge)
{
    uint8_t all = 1;
    for (uint8_t s = 0; s < board_edge_seg_total(edge); s++) {
        uint8_t mask = (uint8_t)((1u << frame_total(board_seg_data_count(edge, s))) - 1u);
        if (seg_frames[edge][s] != mask) { all = 0; break; }
    }
    if (all) {
        edge_ready[edge] = 1;
        cycle_edge_complete |= (uint8_t)(1u << edge);
    }
}

static void can_rx_handler(uint32_t can_id, uint8_t *data, uint8_t len)
{
    if (can_id == CAN_ID_SYNC) return;   // 忽略同步帧

    uint8_t edge  = CAN_ID_GET_EDGE(can_id);
    uint8_t seg   = CAN_ID_GET_SEG(can_id);
    uint8_t frame = CAN_ID_GET_FRAME(can_id);
    if (edge >= EDGE_COUNT || seg >= board_edge_seg_total(edge)) return;
    can_proto_rx_edge_frames[edge]++;
    if (!cycle_accepting) { can_proto_late_frames++; return; }
    if (can_id & 0x400u) { can_proto_bad_edge_frames[edge]++; return; }

    uint16_t scount = board_seg_data_count(edge, seg);   // 该段数据槽数(混合板逐段不同)
    if (frame >= frame_total(scount)) { can_proto_bad_edge_frames[edge]++; return; }
    uint16_t start = frame_start(frame, scount);
    uint16_t remaining = scount - start;
    if (remaining > SENSORS_PER_FRAME) remaining = SENSORS_PER_FRAME;
    uint16_t expected_bytes = 2u * valid_chunk(remaining);
    if (len != expected_bytes) { can_proto_bad_edge_frames[edge]++; return; }

    uint16_t base = 0;
    for (uint8_t s = 0; s < seg; s++) base = (uint16_t)(base + board_seg_data_count(edge, s));

    // 按帧号定位（与到达顺序无关），不会跨周期整块对调
    uint16_t offset = (uint16_t)(base + start);
    if (offset + len / 2 > base + scount) return;

    edge_seen |= (uint8_t)(1u << edge);
    cycle_edge_seen |= (uint8_t)(1u << edge);
    seg_seen[edge][seg] = 1;

    uint16_t storage_offset = (uint16_t)(edge_base(edge) + offset);
    memcpy(&raw[storage_offset], data, len);
    seg_frames[edge][seg] |= (uint8_t)(1u << frame);

    uint8_t mask = (uint8_t)((1u << frame_total(scount)) - 1u);
    if (seg_frames[edge][seg] == mask) {
        check_edge_complete(edge);
    }
}

void can_proto_init(void)
{
    reset_cycle();
    memset(raw, 0, sizeof(raw));
    memset(seg_seen, 0, sizeof(seg_seen));
    edge_seen = 0;
    cycle_edge_seen = 0;
    cycle_edge_complete = 0;
    cycle_accepting = 0;
    for (uint8_t e = 0; e < EDGE_COUNT; e++) {
        can_proto_rx_edge_frames[e] = 0;
        can_proto_bad_edge_frames[e] = 0;
    }
    can_proto_late_frames = 0;
    can_proto_sync_failures = 0;
    bsp_can_init();
    bsp_can_register_rx_callback(can_rx_handler);
}

// 主机广播同步帧：从机收到后开始采集并上传（时序对齐的关键）
void can_proto_send_sync(void)
{
    uint8_t sync_payload = CAN_SYNC_MAGIC;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    reset_cycle();
    cycle_accepting = 1;
    if (!primask) __enable_irq();

    if (!bsp_can_send(CAN_ID_SYNC, &sync_payload, sizeof(sync_payload)))
        can_proto_sync_failures++;
}

// 主机本地采集完成后，标记自己那一段已就绪
void can_proto_mark_local_ready(void)
{
    if (MY_SEGMENT >= board_edge_seg_total(MY_EDGE)) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    edge_seen |= (uint8_t)(1u << MY_EDGE);
    cycle_edge_seen |= (uint8_t)(1u << MY_EDGE);
    seg_seen[MY_EDGE][MY_SEGMENT] = 1;
    seg_frames[MY_EDGE][MY_SEGMENT] = (uint8_t)((1u << frame_total(MY_SENSOR_COUNT)) - 1u);
    check_edge_complete(MY_EDGE);
    if (!primask) __enable_irq();
}

uint8_t can_proto_wait_all_edges(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (1) {
        bsp_can_service();
        uint8_t all = 1;
#if CAN_WAIT_MODE == 2
        /* 按段等待: 只要求"见过面"的段收到完整帧; 未接的段不等待 */
        for (int i = 0; i < EDGE_COUNT && all; i++) {
            for (int s = 0; s < board_edge_seg_total(i); s++) {
                if (!seg_seen[i][s]) continue;
                uint8_t mask = (uint8_t)((1u << frame_total(board_seg_data_count(i, s))) - 1u);
                if (seg_frames[i][s] != mask) { all = 0; break; }
            }
        }
        /* Do not return immediately after the local acquisition.  Remote
           boards acquire in parallel and need a short discovery/drain window;
           without it their first frames can remain in/overflow the RX FIFO,
           so some connected edges are never added to seg_seen. */
        if (HAL_GetTick() - t0 < CAN_DISCOVERY_GRACE_MS) all = 0;
#elif CAN_WAIT_MODE == 1
        for (int i = 0; i < EDGE_COUNT; i++)
            if ((edge_seen & (1u << i)) && !edge_ready[i]) { all = 0; break; }
#else
        for (int i = 0; i < EDGE_COUNT; i++)
            if (!edge_ready[i]) { all = 0; break; }
#endif
        if (all || HAL_GetTick() - t0 > timeout_ms) {
            /* 关闭本轮接收后再由主循环取快照，迟到数据不得改写已输出状态。
               旧协议没有轮次号，无法区分跨到下一轮的迟到帧。 */
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            cycle_accepting = 0;
            if (!primask) __enable_irq();
            return all;
        }
    }
}

uint16_t* can_proto_get_edge_data(uint8_t edge)
{
    return (edge < EDGE_COUNT) ? &raw[edge_base(edge)] : raw;
}

uint32_t can_proto_get_seen_mask(void)
{
    uint32_t m = 0;
    for (int i = 0; i < EDGE_COUNT; i++)
        for (int s = 0; s < board_edge_seg_total(i); s++)
            if (seg_seen[i][s]) m |= (1u << (i * 4 + s));
    return m;
}

uint8_t can_proto_get_cycle_edge_mask(void)
{
    /* UI and snapshot consumers need complete edges, not merely one CAN
       fragment.  This prevents a partial/stale strip from looking valid. */
    return cycle_edge_complete;
}

#endif

#if !MCU_IS_MASTER

static volatile uint32_t sync_count = 0;
static uint32_t sync_consumed = 0;

static void sync_rx_handler(uint32_t can_id, uint8_t *data, uint8_t len)
{
    if (can_id != CAN_ID_SYNC || len < 1 || data[0] != CAN_SYNC_MAGIC) return;
    __DMB();
    sync_count++;
}

void can_proto_init(void)
{
    sync_count = 0;
    sync_consumed = 0;
    bsp_can_init();
    bsp_can_register_rx_callback(sync_rx_handler);
}

// 等主机的同步帧（超时应大于主机一轮周期）
uint8_t can_proto_wait_sync(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (sync_count == sync_consumed) {
        bsp_can_service();
        if (HAL_GetTick() - t0 > timeout_ms) return 0;
    }

    /* 同步可能在上一轮采集/发送期间已经到达。不要在等待入口清标志，
       而是消费最新计数和对应轮次，避免整轮采集被跳过。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    sync_consumed = sync_count;
    if (!primask) __enable_irq();
    return 1;
}

// 上传本段数据：分帧背靠背发送（不插延时！），等 TX FIFO 空位带超时保护
static void send_processed(uint16_t *buf, uint16_t count)
{
    uint16_t offset = 0;
    uint8_t frame = 0;
    while (offset < count && frame < MAX_FRAME) {
        uint16_t chunk = count - offset;
        if (chunk > SENSORS_PER_FRAME) chunk = SENSORS_PER_FRAME;
        chunk = valid_chunk(chunk);
        if (chunk == 0) break;
        uint8_t byte_count = chunk * sizeof(uint16_t);

        uint32_t t0 = HAL_GetTick();
        uint8_t queued = 0;
        while (HAL_GetTick() - t0 <= SLAVE_TX_TIMEOUT_MS) {
            if (bsp_can_tx_free() &&
                bsp_can_send(CAN_DATA_ID(frame),
                             (uint8_t *)&buf[offset], byte_count)) {
                queued = 1;
                break;
            }
        }
        if (!queued) return;
        offset += chunk;
        frame++;
    }
}

void can_proto_send_data(uint16_t *buf, uint16_t count)
{
    send_processed(buf, count);
}

#endif
