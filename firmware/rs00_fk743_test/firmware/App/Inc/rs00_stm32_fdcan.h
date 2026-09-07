#ifndef RS00_STM32_FDCAN_H
#define RS00_STM32_FDCAN_H

#include "rs00_protocol.h"
#include "stm32h7xx_hal.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*rs00_rx_callback_t)(const rs00_frame_t *frame);
typedef void (*mini_rx_callback_t)(uint16_t standard_id, const uint8_t *data,
                                   uint8_t length);

typedef enum {
    RS00_FDCAN_OK = 0,
    RS00_FDCAN_INVALID_ARGUMENT,
    RS00_FDCAN_NOT_STARTED,
    RS00_FDCAN_TX_FIFO_FULL,
    RS00_FDCAN_HAL_ERROR
} rs00_fdcan_result_t;

bool rs00_fdcan_start(FDCAN_HandleTypeDef *hfdcan,
                      rs00_rx_callback_t rx_callback);
bool mini_fdcan_start(FDCAN_HandleTypeDef *hfdcan,
                      mini_rx_callback_t rx_callback);
bool rs00_fdcan_send(const rs00_frame_t *frame);
rs00_fdcan_result_t rs00_fdcan_send_detailed(const rs00_frame_t *frame);
bool rs00_fdcan_send_standard(uint16_t standard_id, const uint8_t *data,
                              uint8_t length);
void rs00_fdcan_on_rx_fifo0(FDCAN_HandleTypeDef *hfdcan);
void rs00_fdcan_on_tx_fifo_empty(FDCAN_HandleTypeDef *hfdcan);
void rs00_fdcan_debug_task(void);
bool rs00_fdcan_is_bus_off(void);
uint32_t rs00_fdcan_get_rx_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif
