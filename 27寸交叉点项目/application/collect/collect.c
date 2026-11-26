// collect.c — 从机：等同步帧 → 采集 → 上传（时序对齐，v2 采集核心）
#include "collect.h"
#include "config.h"

#if !MCU_IS_MASTER

#include "bsp_sensor.h"
#include "mirror.h"
#include "can_proto.h"

static uint16_t buf[HW_SENSOR_MAX];

void collect_init(void)
{
    // 传感器/背景校准已在 main 的 bsp_sensor_init 里做过，这里只初始化 CAN
    can_proto_init();
}

void collect_run(void)
{
    // 等主机的同步帧：收到才开始采集，保证各从机时间对齐
    if (!can_proto_wait_sync(SYNC_TIMEOUT_MS)) {
        return;   // 没等到同步（主机没上电/总线异常），跳过本轮
    }

    bsp_sensor_acquire(buf, MY_SENSOR_COUNT, SENSOR_MODE);
#if MY_MIRROR
    mirror_apply(buf, MY_SENSOR_COUNT);
#endif
    can_proto_send_data(buf, MY_SENSOR_COUNT);
}

#else

void collect_init(void) {}
void collect_run(void) {}

#endif
