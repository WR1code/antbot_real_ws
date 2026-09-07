#ifndef CHASSIS_UART_ACK_H
#define CHASSIS_UART_ACK_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_ACK_FRAME_SIZE 52U
#define CHASSIS_ACK_SYNC_0     0xA5U
#define CHASSIS_ACK_SYNC_1     0x5AU
#define CHASSIS_ACK_VERSION    0x03U
#define CHASSIS_ACK_POSITION_INVALID INT16_MIN

typedef enum {
    ACK_OK = 0,
    ACK_BAD_HEADER,
    ACK_BAD_LENGTH,
    ACK_BAD_CRC,
    ACK_UNSUPPORTED_COMMAND,
    ACK_REJECTED_DISABLED,
    ACK_REJECTED_NOT_HOMED,
    ACK_REJECTED_STEERING_FAULT,
    ACK_REJECTED_ANGULAR_Z,
    ACK_ACCEPTED_WAIT_STEERING,
    ACK_ACCEPTED_DRIVING,
    ACK_TIMEOUT_STOP,
    ACK_CAN_TX_ERROR,
    ACK_CAN_BUS_OFF,
    ACK_ACCEPTED_RESET
} ChassisAckStatus;

typedef struct {
    uint8_t sequence;
    ChassisAckStatus status;
    uint8_t chassis_state;
    uint8_t reject_reason;
    uint16_t fault_flags;
    uint8_t steering_flags;
    uint8_t can_flags;
    uint16_t uart_valid_count;
    uint16_t can_tx_count;
    uint16_t can_rx_count;
    uint32_t stm32_tick;
    int16_t steering_position_mrad[4];
    uint8_t control_id;
    uint8_t detail_type;
    int32_t detail_values[4];
    uint16_t detail_valid_mask;
} ChassisAck;

typedef enum {
    ACK_DETAIL_NONE = 0,
    ACK_DETAIL_STEERING_UID = 1,
    ACK_DETAIL_DRIVE_SPEED_ERPM = 2,
    ACK_DETAIL_DRIVE_CURRENT_10MA = 3,
    ACK_DETAIL_DRIVE_POSITION_CENTIDEG = 4,
    ACK_DETAIL_DRIVE_TEMPERATURE_C = 5,
    ACK_DETAIL_DRIVE_FAULT_BITS = 6,
    ACK_DETAIL_DRIVE_VALID_MASK = 7,
    ACK_DETAIL_STEERING_STATUS = 8,
    ACK_DETAIL_STEERING_FEEDBACK_AGE_MS = 9,
    ACK_DETAIL_SYSTEM_BOOT = 10,
    ACK_DETAIL_SYSTEM_RUNTIME = 11,
    ACK_DETAIL_FIRMWARE_IDENTITY = 12,
    ACK_DETAIL_CRASH_REGISTERS = 13,
    ACK_DETAIL_CRASH_FAULTS = 14,
    ACK_DETAIL_SYSTEM_EVENT = 15,
    ACK_DETAIL_SYSTEM_POWER = 16,
    ACK_DETAIL_DRIVE_FEEDBACK_AGE_MS = 17,
    ACK_DETAIL_DRIVE_SAFETY_FLAGS = 18,
    ACK_DETAIL_DRIVE_VOLTAGE_V = 19
} ChassisAckDetailType;

bool ChassisAck_Encode(const ChassisAck *ack,
                       uint8_t frame[CHASSIS_ACK_FRAME_SIZE]);
bool ChassisAck_Decode(const uint8_t frame[CHASSIS_ACK_FRAME_SIZE],
                       ChassisAck *ack);

#endif
