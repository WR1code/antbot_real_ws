#ifndef HOST_CMD_VEL_PROTOCOL_H
#define HOST_CMD_VEL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HOST_CMD_VEL_FRAME_SIZE       12U
#define HOST_CONTROL_FRAME_SIZE       16U
#define HOST_UART_MAX_FRAME_SIZE      HOST_CONTROL_FRAME_SIZE
#define HOST_CMD_VEL_SYNC_0           0xAAU
#define HOST_CMD_VEL_SYNC_1           0x55U
#define HOST_CMD_VEL_VERSION          0x01U
#define HOST_CONTROL_VERSION          0x02U
#define HOST_CONTROL_MAX_PAYLOAD      8U

typedef enum {
    HOST_CONTROL_QUERY_STATUS = 0x01,
    HOST_CONTROL_QUERY_SYSTEM_BOOT = 0x02,
    HOST_CONTROL_QUERY_SYSTEM_RUNTIME = 0x03,
    HOST_CONTROL_QUERY_FIRMWARE = 0x04,
    HOST_CONTROL_QUERY_CRASH_REGISTERS = 0x05,
    HOST_CONTROL_QUERY_CRASH_FAULTS = 0x06,
    HOST_CONTROL_QUERY_EVENT_LOG = 0x07,
    HOST_CONTROL_QUERY_POWER = 0x08,
    HOST_CONTROL_STEERING_ENABLE = 0x10,
    HOST_CONTROL_STEERING_SET_ALL = 0x11,
    HOST_CONTROL_STEERING_SET_EACH = 0x12,
    HOST_CONTROL_STEERING_DISABLE = 0x13,
    HOST_CONTROL_RS00_CLEAR_FAULT = 0x14,
    HOST_CONTROL_RS00_QUERY_UID = 0x15,
    HOST_CONTROL_RS00_READ_POSITION = 0x16,
    HOST_CONTROL_RS00_SET_CSP = 0x17,
    HOST_CONTROL_RS00_SET_LIMITS = 0x18,
    HOST_CONTROL_RS00_QUERY_FEEDBACK_AGE = 0x19,
    HOST_CONTROL_DRIVE_SET_ACCELERATION = 0x20,
    HOST_CONTROL_DRIVE_SET_DECELERATION = 0x21,
    HOST_CONTROL_DRIVE_SET_ALL = 0x22,
    HOST_CONTROL_DRIVE_STOP = 0x23,
    HOST_CONTROL_DRIVE_WITHDRAW_CURRENT = 0x24,
    HOST_CONTROL_DRIVE_QUERY_FEEDBACK = 0x25,
    HOST_CONTROL_EMERGENCY_STOP = 0x30,
    HOST_CONTROL_CLEAR_FAULT_RESTART = 0x31,
    HOST_CONTROL_SYSTEM_RESET = 0x32
} HostControlId;

typedef struct {
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t wz_mrad_s;
    uint8_t sequence;
    uint8_t version;
    HostControlId control_id;
    uint8_t payload_length;
    uint8_t payload[HOST_CONTROL_MAX_PAYLOAD];
} HostCmdVel;

typedef struct {
    uint8_t frame[HOST_UART_MAX_FRAME_SIZE];
    uint8_t last_frame[HOST_UART_MAX_FRAME_SIZE];
    size_t last_frame_size;
    size_t used;
    uint32_t candidate_frame_count;
    uint32_t valid_frame_count;
    uint32_t header_error_count;
    uint32_t crc_error_count;
    uint32_t version_error_count;
    uint32_t resync_count;
} HostCmdVelParser;

typedef enum {
    HOST_CMD_VEL_PARSE_INCOMPLETE = 0,
    HOST_CMD_VEL_PARSE_VALID,
    HOST_CMD_VEL_PARSE_BAD_HEADER,
    HOST_CMD_VEL_PARSE_BAD_VERSION,
    HOST_CMD_VEL_PARSE_BAD_CRC
} HostCmdVelParseResult;

void HostCmdVelParser_Init(HostCmdVelParser *parser);
bool HostCmdVelParser_PushByte(HostCmdVelParser *parser, uint8_t byte,
                               HostCmdVel *command);
HostCmdVelParseResult HostCmdVelParser_PushByteDetailed(
    HostCmdVelParser *parser, uint8_t byte, HostCmdVel *command);
uint16_t HostCmdVel_Crc16Ccitt(const uint8_t *data, size_t length);
bool HostCmdVel_Encode(const HostCmdVel *command,
                       uint8_t frame[HOST_CMD_VEL_FRAME_SIZE]);
bool HostControl_Encode(HostControlId control_id, uint8_t sequence,
                        const uint8_t *payload, uint8_t payload_length,
                        uint8_t frame[HOST_CONTROL_FRAME_SIZE]);

#endif
