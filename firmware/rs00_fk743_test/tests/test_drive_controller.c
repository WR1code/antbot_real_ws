#include "drive_config.h"
#include "drive_controller.h"
#include "mini_drive_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
} CapturedFrame;

static uint32_t s_tick;
static CapturedFrame s_frames[128];
static unsigned s_frame_count;

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

bool rs00_fdcan_send_standard(uint16_t standard_id, const uint8_t *data,
                              uint8_t length)
{
    CapturedFrame *frame;
    assert(s_frame_count < (sizeof(s_frames) / sizeof(s_frames[0])));
    frame = &s_frames[s_frame_count++];
    frame->id = standard_id;
    frame->length = length;
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, data, length);
    return true;
}

static void run_one_tx(void)
{
    DriveController_Task();
    s_tick += DRIVE_TX_INTERVAL_MS;
}

static void feed_safety_feedback(int32_t speed_erpm)
{
    unsigned wheel;
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        const uint16_t id = (uint16_t)(DRIVE_CAN_ID_FL + wheel);
        const uint32_t raw_speed = (uint32_t)speed_erpm;
        const uint8_t fault[] = {0x0FU, 0x00U, 0x00U, 0x00U};
        const uint8_t speed[] = {
            0x0FU, 0x01U, (uint8_t)(raw_speed >> 24),
            (uint8_t)(raw_speed >> 16), (uint8_t)(raw_speed >> 8),
            (uint8_t)raw_speed
        };
        const uint8_t voltage[] = {0x0FU, 0x04U, 0x00U, 0x18U};
        const uint8_t current[] = {0x0FU, 0x05U, 0x00U, 0x00U};
        const uint8_t temperature[] = {0x0FU, 0x07U, 0x00U, 0x19U};
        DriveController_OnCanFrame(id, fault, sizeof(fault));
        DriveController_OnCanFrame(id, speed, sizeof(speed));
        DriveController_OnCanFrame(id, voltage, sizeof(voltage));
        DriveController_OnCanFrame(id, current, sizeof(current));
        DriveController_OnCanFrame(id, temperature, sizeof(temperature));
    }
}

int main(void)
{
    unsigned index;
    float target;
    DriveMotorFeedback feedback;

    assert(DriveController_Init());
    for (index = 0U; index < 16U; ++index) {
        run_one_tx();
    }
    assert(DriveController_IsConfigured());
    assert(s_frame_count == 16U);

    {
        const uint8_t speed_reply[6] =
            {0x0FU, 0x01U, 0x00U, 0x00U, 0x03U, 0xE8U};
        DriveController_OnCanFrame(DRIVE_CAN_ID_FL, speed_reply,
                                   sizeof(speed_reply));
        assert(DriveController_GetFeedback(DRIVE_WHEEL_FL, &feedback));
        assert(feedback.speed_erpm == 1000);
        assert((feedback.valid_mask
                & (1UL << MINI_DRIVE_FEEDBACK_SPEED)) != 0U);
    }

    for (index = 0U; index < 4U; ++index) {
        assert(s_frames[index].id == (uint16_t)(index + 5U));
        assert(s_frames[index].length == 1U);
        assert(s_frames[index].data[0] == 0x00U);
        assert(s_frames[4U + index].data[0] == 0x0AU);
        assert(s_frames[8U + index].data[0] == 0x10U);
        assert(s_frames[12U + index].data[0] == 0x02U);
    }

    feed_safety_feedback(0);
    assert(DriveController_IsSafetyFeedbackReady());
    {
        const uint8_t hot[] = {0x0FU, 0x07U, 0x00U,
                               DRIVE_MAX_FEEDBACK_TEMPERATURE_C};
        const uint8_t normal[] = {0x0FU, 0x07U, 0x00U, 0x19U};
        DriveController_OnCanFrame(DRIVE_CAN_ID_FL, hot, sizeof(hot));
        assert((DriveController_GetSafetyFlags(DRIVE_WHEEL_FL)
                & DRIVE_SAFETY_OVERTEMPERATURE) != 0U);
        assert(!DriveController_IsHealthy());
        DriveController_OnCanFrame(DRIVE_CAN_ID_FL, normal, sizeof(normal));
        assert(DriveController_GetSafetyFlags(DRIVE_WHEEL_FL) == 0U);
        assert(DriveController_IsHealthy());
    }
    {
        const uint8_t fault[] = {0x0FU, 0x00U, 0x00U, 0x10U};
        const uint8_t clear[] = {0x0FU, 0x00U, 0x00U, 0x00U};
        DriveController_OnCanFrame(DRIVE_CAN_ID_FR, fault, sizeof(fault));
        assert((DriveController_GetSafetyFlags(DRIVE_WHEEL_FR)
                & DRIVE_SAFETY_DRIVER_FAULT) != 0U);
        DriveController_OnCanFrame(DRIVE_CAN_ID_FR, clear, sizeof(clear));
        assert(DriveController_GetSafetyFlags(DRIVE_WHEEL_FR) == 0U);
    }

    assert(DriveController_SetAllWheelSpeeds(0.1f, -0.1f, 0.1f, -0.1f));
    for (index = 0U; index < 4U; ++index) {
        run_one_tx();
    }
    for (index = 16U; index < 20U; ++index) {
        const bool positive = ((index - 16U) % 2U) == 0U;
        assert(s_frames[index].length == 5U);
        assert(s_frames[index].data[0] == 0x02U);
        assert(s_frames[index].data[1] == (positive ? 0x00U : 0xFFU));
        assert(s_frames[index].data[3] == (positive ? 0x00U : 0xFFU));
        assert(s_frames[index].data[4] == (positive ? 0xB6U : 0x4AU));
    }

    DriveController_StopAll();
    assert(!DriveController_AllWheelsStopped());
    feed_safety_feedback(0);
    s_tick += DRIVE_STOP_FEEDBACK_STABLE_MS;
    assert(DriveController_AllWheelsStopped());

    DriveController_EmergencyStop();
    assert(!DriveController_IsHealthy());
    for (index = 0U; index < 4U; ++index) {
        run_one_tx();
    }
    for (index = s_frame_count - 4U; index < s_frame_count; ++index) {
        assert(s_frames[index].length == 3U);
        assert(s_frames[index].data[0] == 0x01U);
        assert(s_frames[index].data[1] == 0x00U);
        assert(s_frames[index].data[2] == 0x00U);
    }

    assert(DriveController_GetTarget(DRIVE_WHEEL_FL, &target));
    assert(fabsf(target) < 0.0001f);
    assert(!DriveController_SetWheelSpeed(DRIVE_WHEEL_FL, 0.1f));
    assert(DriveController_GetTxErrorCount() == 0U);

    assert(DriveController_ClearEmergencyStopAndRestart());
    for (index = 0U; index < 16U; ++index) {
        run_one_tx();
    }
    {
        const unsigned start = s_frame_count;
        assert(DriveController_SetAccelerationErpmS(900));
        for (index = 0U; index < 12U; ++index) {
            run_one_tx();
        }
        for (index = start; index < start + 4U; ++index) {
            assert(s_frames[index].data[0] == 0x0AU);
            assert(s_frames[index].data[3] == 0x03U);
            assert(s_frames[index].data[4] == 0x84U);
        }
    }
    {
        const unsigned start = s_frame_count;
        assert(DriveController_SetDecelerationErpmS(1400));
        for (index = 0U; index < 8U; ++index) {
            run_one_tx();
        }
        for (index = start; index < start + 4U; ++index) {
            assert(s_frames[index].data[0] == 0x10U);
            assert(s_frames[index].data[3] == 0x05U);
            assert(s_frames[index].data[4] == 0x78U);
        }
    }

    puts("OIDelec MINI drive controller tests passed");
    return 0;
}
