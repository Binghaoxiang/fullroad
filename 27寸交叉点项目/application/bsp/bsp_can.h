#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef void (*can_rx_callback_t)(uint32_t can_id, uint8_t *data, uint8_t len);

extern FDCAN_TxHeaderTypeDef tx_header;
extern FDCAN_RxHeaderTypeDef rx_header;
extern uint8_t rx_data[64];
extern volatile uint8_t rx_flag;

void bsp_can_init(void);
void bsp_can_service(void);
uint8_t bsp_can_send(uint32_t id, uint8_t *data, uint8_t len);
uint8_t bsp_can_tx_free(void);
uint8_t bsp_can_wait_rx(uint32_t id, uint32_t timeout_ms);
uint8_t bsp_can_get_rx_length(void);
void bsp_can_register_rx_callback(can_rx_callback_t callback);
uint32_t bsp_can_get_rx_fifo_full_count(void);
uint32_t bsp_can_get_rx_fifo_lost_count(void);

#endif
