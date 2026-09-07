#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#ifndef CHASSIS_DEBUG_ENABLE
#define CHASSIS_DEBUG_ENABLE 1
#endif

#ifndef CHASSIS_UART_ACK_ENABLE
#define CHASSIS_UART_ACK_ENABLE 1
#endif

#ifndef CHASSIS_DEBUG_TEXT_ENABLE
#define CHASSIS_DEBUG_TEXT_ENABLE 0
#endif

#define CHASSIS_DEBUG_MAGIC   0x43484442UL /* "CHDB" */
#define CHASSIS_DEBUG_VERSION 0x00010000UL

typedef enum {
    CHASSIS_STATE_BOOT = 0,
    CHASSIS_STATE_DISABLED,
    CHASSIS_STATE_WAIT_ENABLE,
    CHASSIS_STATE_WAIT_HOMING,
    CHASSIS_STATE_IDLE,
    CHASSIS_STATE_STEERING,
    CHASSIS_STATE_DRIVE,
    CHASSIS_STATE_TIMEOUT_STOP,
    CHASSIS_STATE_FAULT,
    CHASSIS_STATE_EMERGENCY_STOP
} ChassisDebugState;

typedef enum {
    CHASSIS_REJECT_NONE = 0,
    CHASSIS_REJECT_DISABLED,
    CHASSIS_REJECT_NOT_HOMED,
    CHASSIS_REJECT_STEERING_FAULT,
    CHASSIS_REJECT_NONZERO_ANGULAR_Z,
    CHASSIS_REJECT_COMM_TIMEOUT,
    CHASSIS_REJECT_INVALID_COMMAND,
    CHASSIS_REJECT_CAN_FAULT
} ChassisRejectReason;

typedef enum {
    CHASSIS_FAULT_NONE = 0U,
    CHASSIS_FAULT_STEERING = (1U << 0),
    CHASSIS_FAULT_DRIVE = (1U << 1),
    CHASSIS_FAULT_CAN = (1U << 2),
    CHASSIS_FAULT_CAN_BUS_OFF = (1U << 3),
    CHASSIS_FAULT_UART = (1U << 4)
} ChassisFaultFlags;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t system_tick;

    uint32_t uart_rx_byte_count;
    uint32_t uart_rx_event_count;
    uint32_t uart_last_rx_length;
    uint32_t uart_last_rx_tick;
    uint32_t uart_candidate_frame_count;
    uint32_t uart_valid_frame_count;
    uint32_t uart_header_error_count;
    uint32_t uart_length_error_count;
    uint32_t uart_crc_error_count;
    uint32_t uart_sequence_error_count;
    uint32_t uart_overflow_count;
    uint32_t uart_resync_count;
    uint32_t uart_timeout_count;
    uint32_t uart_ack_tx_count;
    uint32_t uart_ack_drop_count;

    uint32_t command_sequence;
    int32_t command_vx_raw;
    int32_t command_vy_raw;
    int32_t command_wz_raw;
    float command_vx_mps;
    float command_vy_mps;
    float command_wz_rps;
    uint32_t last_valid_command_tick;

    uint32_t chassis_state;
    uint32_t previous_chassis_state;
    uint32_t chassis_state_change_count;
    uint32_t last_state_change_tick;
    uint32_t chassis_fault_flags;
    uint32_t chassis_reject_reason;

    uint32_t steering_enabled;
    uint32_t steering_homed;
    uint32_t steering_ready;
    uint32_t steering_fault;
    uint32_t steering_enable_request_count;

    uint32_t can_tx_submit_ok_count;
    uint32_t can_tx_submit_error_count;
    uint32_t can_tx_complete_count;
    uint32_t can_rx_frame_count;
    uint32_t can_rx_rs00_count;
    uint32_t can_rx_mini_count;
    uint32_t can_unknown_id_count;

    uint32_t fdcan_bus_off;
    uint32_t fdcan_error_passive;
    uint32_t fdcan_warning;
    uint32_t fdcan_last_error_code;
    uint32_t fdcan_tx_error_count;
    uint32_t fdcan_rx_error_count;

    uint32_t last_can_tx_id;
    uint32_t last_can_tx_dlc;
    uint32_t last_can_rx_id;
    uint32_t last_can_rx_dlc;
    uint32_t last_can_tx_tick;
    uint32_t last_can_rx_tick;
    uint32_t last_can_tx_success;
    uint32_t last_can_rx_valid;
    uint8_t last_can_tx_data[8];
    uint8_t last_can_rx_data[8];

    uint32_t debug_hook_counter;
    uint32_t debug_hook_last_reason;
} ChassisDebugSnapshot;

extern volatile ChassisDebugSnapshot g_chassis_debug;
extern volatile uint8_t uart_last_raw_frame[12];

void ChassisDebug_Init(void);
void ChassisDebug_TaskTick(uint32_t tick);
void ChassisDebug_RecordUartRx(uint32_t length, uint32_t tick);
void ChassisDebug_RecordRawFrame(const uint8_t frame[12]);
void ChassisDebug_RecordCommand(uint8_t sequence, int16_t vx, int16_t vy,
                                int16_t wz, uint32_t tick);
void ChassisDebug_SetState(ChassisDebugState state, uint32_t tick);
void ChassisDebug_SetReject(ChassisRejectReason reason);
void ChassisDebug_SetFault(uint32_t fault_flags);
void ChassisDebug_ClearFaults(void);
void ChassisDebug_SetSteering(bool enabled, bool homed, bool ready,
                              bool fault);
void ChassisDebug_RecordCanTx(uint32_t id, const uint8_t *data,
                              uint8_t length, bool submitted, uint32_t tick);
void ChassisDebug_RecordCanRx(uint32_t id, const uint8_t *data,
                              uint8_t length, bool rs00, bool mini,
                              uint32_t tick);
void ChassisDebug_RecordFdcanStatus(bool bus_off, bool error_passive,
                                   bool warning, uint32_t last_error,
                                   uint32_t tx_errors, uint32_t rx_errors);

/* Stable, non-inlined breakpoint locations. */
void DebugHook_UartRx(void);
void DebugHook_UartFrameValid(void);
void DebugHook_UartFrameRejected(uint32_t reason);
void DebugHook_ChassisStateChanged(uint32_t old_state, uint32_t new_state);
void DebugHook_CanTxBeforeSubmit(uint32_t id, const uint8_t *data);
void DebugHook_CanTxSubmitted(uint32_t id, const uint8_t *data);
void DebugHook_CanTxFailed(uint32_t id, uint32_t error);
void DebugHook_CanRxReceived(uint32_t id, const uint8_t *data);
void DebugHook_Fault(uint32_t fault_flags);
void DebugHook_TimeoutStop(void);

#endif
