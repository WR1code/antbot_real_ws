#include "host_cmd_vel_protocol.h"

#include <string.h>

static int16_t read_i16_le(const uint8_t *data)
{
    const uint16_t value =
        (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

static void write_i16_le(uint8_t *data, int16_t value)
{
    const uint16_t encoded = (uint16_t)value;
    data[0] = (uint8_t)encoded;
    data[1] = (uint8_t)(encoded >> 8U);
}

uint16_t HostCmdVel_Crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        unsigned bit;
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

void HostCmdVelParser_Init(HostCmdVelParser *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

static void resynchronize(HostCmdVelParser *parser)
{
    /*
     * Retain a trailing 0xAA so AA AA 55 can recover without losing the
     * second possible header byte.
     */
    if ((parser->used != 0U)
        && (parser->frame[parser->used - 1U] == HOST_CMD_VEL_SYNC_0)) {
        parser->frame[0] = HOST_CMD_VEL_SYNC_0;
        parser->used = 1U;
    } else {
        parser->used = 0U;
    }
    ++parser->resync_count;
}

HostCmdVelParseResult HostCmdVelParser_PushByteDetailed(
    HostCmdVelParser *parser, uint8_t byte, HostCmdVel *command)
{
    uint16_t expected_crc;
    uint16_t actual_crc;
    size_t expected_size;

    if ((parser == NULL) || (command == NULL)) {
        return HOST_CMD_VEL_PARSE_INCOMPLETE;
    }

    if (parser->used == 0U) {
        if (byte == HOST_CMD_VEL_SYNC_0) {
            parser->frame[0] = byte;
            parser->used = 1U;
        }
        return HOST_CMD_VEL_PARSE_INCOMPLETE;
    }
    if (parser->used == 1U) {
        if (byte == HOST_CMD_VEL_SYNC_1) {
            parser->frame[1] = byte;
            parser->used = 2U;
        } else if (byte != HOST_CMD_VEL_SYNC_0) {
            parser->used = 0U;
            ++parser->header_error_count;
            ++parser->resync_count;
            return HOST_CMD_VEL_PARSE_BAD_HEADER;
        }
        return HOST_CMD_VEL_PARSE_INCOMPLETE;
    }

    parser->frame[parser->used++] = byte;
    if (parser->used == 3U
        && (parser->frame[2] != HOST_CMD_VEL_VERSION)
        && (parser->frame[2] != HOST_CONTROL_VERSION)) {
        ++parser->version_error_count;
        resynchronize(parser);
        return HOST_CMD_VEL_PARSE_BAD_VERSION;
    }
    expected_size = parser->frame[2] == HOST_CONTROL_VERSION
                        ? HOST_CONTROL_FRAME_SIZE
                        : HOST_CMD_VEL_FRAME_SIZE;
    if (parser->used < expected_size) {
        return HOST_CMD_VEL_PARSE_INCOMPLETE;
    }

    memcpy(parser->last_frame, parser->frame, expected_size);
    parser->last_frame_size = expected_size;
    ++parser->candidate_frame_count;
    expected_crc =
        (uint16_t)parser->frame[expected_size - 2U]
        | ((uint16_t)parser->frame[expected_size - 1U] << 8U);
    actual_crc = HostCmdVel_Crc16Ccitt(parser->frame, expected_size - 2U);
    if (expected_crc != actual_crc) {
        ++parser->crc_error_count;
        resynchronize(parser);
        return HOST_CMD_VEL_PARSE_BAD_CRC;
    }
    command->sequence = parser->frame[3];
    command->version = parser->frame[2];
    command->control_id = (HostControlId)0;
    command->payload_length = 0U;
    memset(command->payload, 0, sizeof(command->payload));
    if (command->version == HOST_CMD_VEL_VERSION) {
        command->vx_mm_s = read_i16_le(&parser->frame[4]);
        command->vy_mm_s = read_i16_le(&parser->frame[6]);
        command->wz_mrad_s = read_i16_le(&parser->frame[8]);
    } else {
        command->control_id = (HostControlId)parser->frame[3];
        command->sequence = parser->frame[4];
        command->payload_length = parser->frame[5];
        if (command->payload_length > HOST_CONTROL_MAX_PAYLOAD) {
            resynchronize(parser);
            return HOST_CMD_VEL_PARSE_BAD_HEADER;
        }
        memcpy(command->payload, &parser->frame[6], HOST_CONTROL_MAX_PAYLOAD);
        command->vx_mm_s = 0;
        command->vy_mm_s = 0;
        command->wz_mrad_s = 0;
    }
    ++parser->valid_frame_count;
    parser->used = 0U;
    return HOST_CMD_VEL_PARSE_VALID;
}

bool HostCmdVelParser_PushByte(HostCmdVelParser *parser, uint8_t byte,
                               HostCmdVel *command)
{
    return HostCmdVelParser_PushByteDetailed(parser, byte, command)
           == HOST_CMD_VEL_PARSE_VALID;
}

bool HostCmdVel_Encode(const HostCmdVel *command,
                       uint8_t frame[HOST_CMD_VEL_FRAME_SIZE])
{
    uint16_t crc;

    if ((command == NULL) || (frame == NULL)) {
        return false;
    }
    frame[0] = HOST_CMD_VEL_SYNC_0;
    frame[1] = HOST_CMD_VEL_SYNC_1;
    frame[2] = HOST_CMD_VEL_VERSION;
    frame[3] = command->sequence;
    write_i16_le(&frame[4], command->vx_mm_s);
    write_i16_le(&frame[6], command->vy_mm_s);
    write_i16_le(&frame[8], command->wz_mrad_s);
    crc = HostCmdVel_Crc16Ccitt(frame, 10U);
    frame[10] = (uint8_t)crc;
    frame[11] = (uint8_t)(crc >> 8U);
    return true;
}

bool HostControl_Encode(HostControlId control_id, uint8_t sequence,
                        const uint8_t *payload, uint8_t payload_length,
                        uint8_t frame[HOST_CONTROL_FRAME_SIZE])
{
    uint16_t crc;
    if ((frame == NULL) || (payload_length > HOST_CONTROL_MAX_PAYLOAD)
        || ((payload_length != 0U) && (payload == NULL))) {
        return false;
    }
    memset(frame, 0, HOST_CONTROL_FRAME_SIZE);
    frame[0] = HOST_CMD_VEL_SYNC_0;
    frame[1] = HOST_CMD_VEL_SYNC_1;
    frame[2] = HOST_CONTROL_VERSION;
    frame[3] = (uint8_t)control_id;
    frame[4] = sequence;
    frame[5] = payload_length;
    if (payload_length != 0U) {
        memcpy(&frame[6], payload, payload_length);
    }
    crc = HostCmdVel_Crc16Ccitt(frame, HOST_CONTROL_FRAME_SIZE - 2U);
    frame[14] = (uint8_t)crc;
    frame[15] = (uint8_t)(crc >> 8U);
    return true;
}
