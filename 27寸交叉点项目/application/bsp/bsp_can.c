#include "bsp_can.h"
#include "fdcan.h"
#include "config.h"
#include "stm32g4xx_hal.h"
#include <string.h>

FDCAN_TxHeaderTypeDef tx_header;
FDCAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[64];
volatile uint8_t rx_flag;

static can_rx_callback_t can_rx_callback = NULL;
static volatile uint8_t can_busoff_req = 0;
static volatile uint32_t can_rx_fifo_full_count = 0;
static volatile uint32_t can_rx_fifo_lost_count = 0;
volatile uint32_t can_busoff_recovery_count = 0;
static uint32_t can_last_recovery_tick = 0;

#define CAN_RX_NOTIFICATIONS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                              FDCAN_IT_RX_FIFO0_FULL | \
                              FDCAN_IT_RX_FIFO0_MESSAGE_LOST | \
                              FDCAN_IT_BUS_OFF)

extern FDCAN_HandleTypeDef hfdcan1;

static void can_restart(void)
{
    /* 已关闭总线时仍可能有3个旧分帧堵在发送队列；取消后再恢复，
       否则恢复后会重新发送过期帧，且等待空位的路径永远进不了恢复函数。 */
    HAL_FDCAN_AbortTxRequest(&hfdcan1,
        FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, CAN_RX_NOTIFICATIONS, 0);
}

/* 仅在主循环调用，不依赖发送空位或已收到同步。 */
void bsp_can_service(void)
{
    if (hfdcan1.State != HAL_FDCAN_STATE_BUSY) return;
    if (!can_busoff_req && !(hfdcan1.Instance->PSR & FDCAN_PSR_BO)) return;
    /* BUS_OFF 中断在进入和离开状态时都会出现，只在仍为bus-off时恢复。 */
    if (!(hfdcan1.Instance->PSR & FDCAN_PSR_BO)) {
        can_busoff_req = 0;
        return;
    }
    uint32_t now = HAL_GetTick();
    if (now - can_last_recovery_tick < 100u) return;
    can_last_recovery_tick = now;
    can_busoff_req = 0;
    can_busoff_recovery_count++;
    can_restart();
}


void bsp_can_init(void)
{
    rx_flag = 0;
    can_busoff_req = 0;
    can_rx_fifo_full_count = 0;
    can_rx_fifo_lost_count = 0;
    can_busoff_recovery_count = 0;
    can_last_recovery_tick = HAL_GetTick() - 100u;
    tx_header.Identifier = 0;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
#if CAN_SPEED_MODE == 1
    tx_header.BitRateSwitch = FDCAN_BRS_ON;      /* 数据段切 2Mbps */
#else
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
#endif
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    HAL_FDCAN_Start(&hfdcan1);

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    HAL_FDCAN_ActivateNotification(&hfdcan1, CAN_RX_NOTIFICATIONS, 0);
}

void bsp_can_register_rx_callback(can_rx_callback_t callback)
{
    can_rx_callback = callback;
}

uint8_t bsp_can_send(uint32_t id, uint8_t *data, uint8_t len)
{
    bsp_can_service();
    if (hfdcan1.Instance->PSR & FDCAN_PSR_BO) return 0;

    tx_header.Identifier = id;

    switch (len) {
        case 0:  tx_header.DataLength = FDCAN_DLC_BYTES_0;  break;
        case 1:  tx_header.DataLength = FDCAN_DLC_BYTES_1;  break;
        case 2:  tx_header.DataLength = FDCAN_DLC_BYTES_2;  break;
        case 3:  tx_header.DataLength = FDCAN_DLC_BYTES_3;  break;
        case 4:  tx_header.DataLength = FDCAN_DLC_BYTES_4;  break;
        case 5:  tx_header.DataLength = FDCAN_DLC_BYTES_5;  break;
        case 6:  tx_header.DataLength = FDCAN_DLC_BYTES_6;  break;
        case 7:  tx_header.DataLength = FDCAN_DLC_BYTES_7;  break;
        case 8:  tx_header.DataLength = FDCAN_DLC_BYTES_8;  break;
        case 12: tx_header.DataLength = FDCAN_DLC_BYTES_12; break;
        case 16: tx_header.DataLength = FDCAN_DLC_BYTES_16; break;
        case 20: tx_header.DataLength = FDCAN_DLC_BYTES_20; break;
        case 24: tx_header.DataLength = FDCAN_DLC_BYTES_24; break;
        case 32: tx_header.DataLength = FDCAN_DLC_BYTES_32; break;
        case 48: tx_header.DataLength = FDCAN_DLC_BYTES_48; break;
        case 64: tx_header.DataLength = FDCAN_DLC_BYTES_64; break;
        default: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, data) == HAL_OK;
}

// TX FIFO 是否有空位（从机发送前检查，避免丢帧）
uint8_t bsp_can_tx_free(void)
{
    bsp_can_service();
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0;
}

uint8_t bsp_can_wait_rx(uint32_t id, uint32_t timeout_ms)
{
    rx_flag = 0;
    uint32_t t0 = HAL_GetTick();
    while (!rx_flag) {
        if (HAL_GetTick() - t0 > timeout_ms) return 0;
    }
    return 1;
}

uint8_t bsp_can_get_rx_length(void)
{
    uint32_t dlc_code = (rx_header.DataLength >> 16) & 0x0F;
    switch (dlc_code) {
        case 0:  return 0;
        case 1:  return 1;
        case 2:  return 2;
        case 3:  return 3;
        case 4:  return 4;
        case 5:  return 5;
        case 6:  return 6;
        case 7:  return 7;
        case 8:  return 8;
        case 9:  return 12;
        case 10: return 16;
        case 11: return 20;
        case 12: return 24;
        case 13: return 32;
        case 14: return 48;
        case 15: return 64;
        default: return 0;
    }
}

uint32_t bsp_can_get_rx_fifo_full_count(void)
{
    return can_rx_fifo_full_count;
}

uint32_t bsp_can_get_rx_fifo_lost_count(void)
{
    return can_rx_fifo_lost_count;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan->Instance == FDCAN1 && (ErrorStatusITs & FDCAN_IT_BUS_OFF)) {
        can_busoff_req = 1;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) return;

    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)
        can_rx_fifo_full_count++;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
        can_rx_fifo_lost_count++;

    /* CAN 数据是多块板同时突发上传。一次中断必须排空 FIFO，不能只取
       第一帧，否则低优先级边的后续分帧会残留甚至溢出。 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &rx_header, rx_data) != HAL_OK)
            break;

        if (can_rx_callback) {
            uint8_t len = bsp_can_get_rx_length();
            can_rx_callback(rx_header.Identifier, rx_data, len);
        }
        rx_flag = 1;
    }
}
