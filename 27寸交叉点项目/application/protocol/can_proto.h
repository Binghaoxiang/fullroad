#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdint.h>
#include "config.h"

// ==================== CAN ID 分配（11 位标准帧）====================
//  同步帧 : 0x7FF（最高段，独占）
//  同步载荷: [0x5A]，保持已部署从板的一字节同步格式
//  数据帧 : [保留:1位][边:3位][段:3位][帧号:4位]
//           bit 10   = 0，仅发送处理后的数据
//           bit 9..7 = 边 (0..7)  最多 8 条边
//           bit 6..4 = 段 (0..7)  每条边最多 8 段
//           bit 3..0 = 帧号(0..15) 每段最多 16 帧（32 点/帧 → 512 点）
//  数据位于 0x000..0x3FF。0x7FF 保留为同步帧。
#define CAN_ID_SYNC              0x7FFu
#define CAN_SYNC_MAGIC           0x5Au
#define CAN_DATA_ID(frame) ((((uint32_t)(MY_EDGE)) << 7) | (((uint32_t)(MY_SEGMENT)) << 4) | ((uint32_t)(frame) & 0x0Fu))
#define CAN_ID_GET_EDGE(id)      (((id) >> 7) & 0x07u)
#define CAN_ID_GET_SEG(id)       (((id) >> 4) & 0x07u)
#define CAN_ID_GET_FRAME(id)     ((id) & 0x0Fu)

#define SENSORS_PER_FRAME  32
#define MAX_FRAME          16

#if MCU_IS_MASTER

void can_proto_init(void);
void can_proto_send_sync(void);
uint8_t can_proto_wait_all_edges(uint32_t timeout_ms);
uint16_t* can_proto_get_edge_data(uint8_t edge);
void can_proto_mark_local_ready(void);
uint32_t can_proto_get_seen_mask(void);
uint8_t can_proto_get_cycle_edge_mask(void);

#else

void can_proto_init(void);
uint8_t can_proto_wait_sync(uint32_t timeout_ms);
void can_proto_send_data(uint16_t *buf, uint16_t count);

#endif

#endif
