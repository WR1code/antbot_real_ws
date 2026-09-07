#include "mini_drive_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void expect(const MiniDriveFrame *frame, uint16_t id, uint8_t dlc,
                   const uint8_t *data)
{
    assert(frame->id == id);
    assert(frame->dlc == dlc);
    assert(memcmp(frame->data, data, dlc) == 0);
}

int main(void)
{
    MiniDriveFrame frame;
    MiniDriveFeedback feedback;
    const uint8_t heartbeat[] = {0x00U};
    const uint8_t speed_positive[] = {0x02U, 0x00U, 0x00U, 0x0FU, 0xA0U};
    const uint8_t speed_negative[] = {0x02U, 0xFFU, 0xFFU, 0xF0U, 0x60U};
    const uint8_t acceleration[] = {0x0AU, 0x00U, 0x00U, 0x07U, 0xD0U};
    const uint8_t deceleration[] = {0x10U, 0x00U, 0x00U, 0x03U, 0x20U};
    const uint8_t zero_current[] = {0x01U, 0x00U, 0x00U};
    const uint8_t voltage_query[] = {0x0FU, 0x04U};

    assert(MiniDrive_MakeHeartbeat(0x001U, &frame));
    expect(&frame, 0x001U, 1U, heartbeat);
    assert(MiniDrive_MakeSpeed(0x001U, 4000, &frame));
    expect(&frame, 0x001U, 5U, speed_positive);
    assert(MiniDrive_MakeSpeed(0x001U, -4000, &frame));
    expect(&frame, 0x001U, 5U, speed_negative);
    assert(MiniDrive_MakeAcceleration(0x001U, 2000, &frame));
    expect(&frame, 0x001U, 5U, acceleration);
    assert(MiniDrive_MakeDeceleration(0x001U, 800, &frame));
    expect(&frame, 0x001U, 5U, deceleration);
    assert(MiniDrive_MakeCurrent(0x001U, 0, &frame));
    expect(&frame, 0x001U, 3U, zero_current);
    assert(MiniDrive_MakeQuery(
        0x001U, MINI_DRIVE_FEEDBACK_VOLTAGE, &frame));
    expect(&frame, 0x001U, 2U, voltage_query);

    assert(!MiniDrive_MakeHeartbeat(0x800U, &frame));
    assert(!MiniDrive_MakeAcceleration(0x001U, -1, &frame));
    assert(!MiniDrive_MakeDeceleration(0x001U, -1, &frame));
    assert(!MiniDrive_MakeSpeed(0x001U, 0, NULL));

    {
        const uint8_t voltage_reply[] = {0x0FU, 0x04U, 0x00U, 0x18U};
        memset(&frame, 0, sizeof(frame));
        frame.id = 0x008U;
        frame.dlc = sizeof(voltage_reply);
        memcpy(frame.data, voltage_reply, sizeof(voltage_reply));
        assert(MiniDrive_DecodeFeedback(&frame, &feedback));
        assert(feedback.type == MINI_DRIVE_FEEDBACK_VOLTAGE);
        assert(feedback.value == 24);
    }
    {
        const uint8_t speed_reply[] =
            {0x0FU, 0x01U, 0xFFU, 0xFFU, 0xF0U, 0x60U};
        memset(&frame, 0, sizeof(frame));
        frame.id = 0x005U;
        frame.dlc = sizeof(speed_reply);
        memcpy(frame.data, speed_reply, sizeof(speed_reply));
        assert(MiniDrive_DecodeFeedback(&frame, &feedback));
        assert(feedback.id == 0x005U);
        assert(feedback.type == MINI_DRIVE_FEEDBACK_SPEED);
        assert(feedback.value == -4000);
    }
    {
        const uint8_t current_reply[] = {0x0FU, 0x05U, 0xFFU, 0x9CU};
        memset(&frame, 0, sizeof(frame));
        frame.id = 0x006U;
        frame.dlc = sizeof(current_reply);
        memcpy(frame.data, current_reply, sizeof(current_reply));
        assert(MiniDrive_DecodeFeedback(&frame, &feedback));
        assert(feedback.type == MINI_DRIVE_FEEDBACK_MOTOR_CURRENT);
        assert(feedback.value == -100);
    }
    {
        const uint8_t position_reply[] =
            {0x0FU, 0x08U, 0x00U, 0x00U, 0x8CU, 0xA0U};
        memset(&frame, 0, sizeof(frame));
        frame.id = 0x007U;
        frame.dlc = sizeof(position_reply);
        memcpy(frame.data, position_reply, sizeof(position_reply));
        assert(MiniDrive_DecodeFeedback(&frame, &feedback));
        assert(feedback.type == MINI_DRIVE_FEEDBACK_POSITION);
        assert(feedback.value == 36000);
        frame.dlc = 5U;
        assert(!MiniDrive_DecodeFeedback(&frame, &feedback));
    }

    puts("OIDelec MINI CAN protocol tests passed");
    return 0;
}
