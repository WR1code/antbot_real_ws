#ifndef RS00_APP_H
#define RS00_APP_H

#include "rs00_protocol.h"
#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RS00_COMMAND_NONE = 0,
    RS00_COMMAND_QUERY = 1,
    RS00_COMMAND_READ_VBUS = 2,
    RS00_COMMAND_START_SAFE_SPEED_TEST = 3,
    RS00_COMMAND_STOP = 4,
    RS00_COMMAND_STOP_AND_CLEAR_FAULT = 5
} rs00_command_t;

typedef struct {
    volatile bool device_seen;
    volatile bool feedback_valid;
    volatile bool vbus_valid;
    volatile uint32_t last_rx_tick_ms;
    volatile float vbus_v;
    volatile rs00_feedback_t feedback;
    volatile uint8_t uid[8];
} rs00_app_status_t;

extern volatile rs00_command_t g_rs00_command;
extern rs00_app_status_t g_rs00_status;

bool rs00_app_init(FDCAN_HandleTypeDef *hfdcan);
void rs00_app_poll(void);
void rs00_app_on_rx(const rs00_frame_t *frame);
void rs00_app_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif
