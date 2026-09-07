#include "host_cmd_vel_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static bool feed(HostCmdVelParser *parser, const uint8_t *data, size_t length,
                 HostCmdVel *decoded)
{
    bool received = false;
    size_t index;

    for (index = 0U; index < length; ++index) {
        if (HostCmdVelParser_PushByte(parser, data[index], decoded)) {
            received = true;
        }
    }
    return received;
}

int main(void)
{
    const HostCmdVel input = {
        .vx_mm_s = 123,
        .vy_mm_s = -45,
        .wz_mrad_s = 0,
        .sequence = 7U
    };
    HostCmdVel decoded;
    HostCmdVelParser parser;
    uint8_t frame[HOST_CMD_VEL_FRAME_SIZE];
    uint8_t damaged[HOST_CMD_VEL_FRAME_SIZE];
    uint8_t control_frame[HOST_CONTROL_FRAME_SIZE];
    const uint8_t control_payload[8] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    const uint8_t noise[] = {0x00U, 0xAAU, 0xAAU, 0x55U};

    HostCmdVelParser_Init(&parser);
    assert(HostCmdVel_Encode(&input, frame));
    assert(!feed(&parser, noise, sizeof(noise), &decoded));

    /*
     * The final AA 55 in noise is already a header; feed the payload and CRC
     * from the valid frame to verify stream resynchronization.
     */
    assert(feed(&parser, &frame[2], HOST_CMD_VEL_FRAME_SIZE - 2U, &decoded));
    assert(decoded.vx_mm_s == input.vx_mm_s);
    assert(decoded.vy_mm_s == input.vy_mm_s);
    assert(decoded.wz_mrad_s == input.wz_mrad_s);
    assert(decoded.sequence == input.sequence);
    assert(decoded.version == HOST_CMD_VEL_VERSION);

    assert(HostControl_Encode(HOST_CONTROL_STEERING_SET_EACH, 9U,
                              control_payload, sizeof(control_payload),
                              control_frame));
    assert(feed(&parser, control_frame, sizeof(control_frame), &decoded));
    assert(decoded.version == HOST_CONTROL_VERSION);
    assert(decoded.control_id == HOST_CONTROL_STEERING_SET_EACH);
    assert(decoded.sequence == 9U);
    assert(decoded.payload_length == 8U);
    assert(memcmp(decoded.payload, control_payload, 8U) == 0);

    assert(HostControl_Encode(HOST_CONTROL_SYSTEM_RESET, 10U,
                              (const uint8_t *)"RST!", 4U,
                              control_frame));
    assert(feed(&parser, control_frame, sizeof(control_frame), &decoded));
    assert(decoded.control_id == HOST_CONTROL_SYSTEM_RESET);
    assert(decoded.sequence == 10U);
    assert(decoded.payload_length == 4U);
    assert(memcmp(decoded.payload, "RST!", 4U) == 0);

    assert(HostControl_Encode(HOST_CONTROL_QUERY_EVENT_LOG, 11U,
                              (const uint8_t *)"\x1f", 1U,
                              control_frame));
    assert(feed(&parser, control_frame, sizeof(control_frame), &decoded));
    assert(decoded.control_id == HOST_CONTROL_QUERY_EVENT_LOG);
    assert(decoded.sequence == 11U);
    assert(decoded.payload_length == 1U);
    assert(decoded.payload[0] == 31U);

    memcpy(damaged, frame, sizeof(damaged));
    damaged[6] ^= 0x01U;
    assert(!feed(&parser, damaged, sizeof(damaged), &decoded));
    assert(parser.crc_error_count == 1U);
    assert(feed(&parser, frame, sizeof(frame), &decoded));
    assert(parser.valid_frame_count == 5U);

    puts("Host cmd_vel protocol tests passed");
    return 0;
}
