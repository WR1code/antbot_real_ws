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
static bool s_can_send_ok = true;

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

bool rs00_fdcan_send_standard(uint16_t standard_id, const uint8_t *data,
                              uint8_t length)
{
    CapturedFrame *frame;
    if (!s_can_send_ok) {
        return false;
    }
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
        const uint16_t nominal_voltage =
            (DRIVE_MIN_FEEDBACK_VOLTAGE_V
             + DRIVE_MAX_FEEDBACK_VOLTAGE_V) / 2U;
        const uint8_t voltage[] = {
            0x0FU, 0x04U, (uint8_t)(nominal_voltage >> 8U),
            (uint8_t)nominal_voltage
        };
        const uint8_t current[] = {0x0FU, 0x05U, 0x00U, 0x00U};
        const uint8_t temperature[] = {0x0FU, 0x07U, 0x00U, 0x19U};
        DriveController_OnCanFrame(id, fault, sizeof(fault));
        DriveController_OnCanFrame(id, speed, sizeof(speed));
        DriveController_OnCanFrame(id, voltage, sizeof(voltage));
        DriveController_OnCanFrame(id, current, sizeof(current));
        DriveController_OnCanFrame(id, temperature, sizeof(temperature));
    }
}

static void feed_safety_feedback_without_voltage(int32_t speed_erpm)
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
        const uint8_t current[] = {0x0FU, 0x05U, 0x00U, 0x00U};
        const uint8_t temperature[] = {0x0FU, 0x07U, 0x00U, 0x19U};
        DriveController_OnCanFrame(id, fault, sizeof(fault));
        DriveController_OnCanFrame(id, speed, sizeof(speed));
        DriveController_OnCanFrame(id, current, sizeof(current));
        DriveController_OnCanFrame(id, temperature, sizeof(temperature));
    }
}

static void feed_voltage_only(void)
{
    unsigned wheel;
    const uint16_t nominal_voltage =
        (DRIVE_MIN_FEEDBACK_VOLTAGE_V
         + DRIVE_MAX_FEEDBACK_VOLTAGE_V) / 2U;
    const uint8_t voltage[] = {
        0x0FU, 0x04U, (uint8_t)(nominal_voltage >> 8U),
        (uint8_t)nominal_voltage
    };
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        DriveController_OnCanFrame(
            (uint16_t)(DRIVE_CAN_ID_FL + wheel), voltage, sizeof(voltage));
    }
}

int main(void)
{
    unsigned index;
    const unsigned startup_failed_sends =
        (DRIVE_TX_FAILURE_TIMEOUT_MS / DRIVE_TX_INTERVAL_MS) + 2U;
    float target;
    DriveMotorFeedback feedback;

    assert(DriveController_Init());
    s_can_send_ok = false;
    for (index = 0U; index < startup_failed_sends; ++index) {
        run_one_tx();
    }
    assert(DriveController_IsHealthy());
    assert(!DriveController_IsConfigured());
    s_can_send_ok = true;
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

    /* The production 1.50 m/s command is valid (~2728 erpm), while values
     * beyond the configured chassis boundary remain rejected. */
    assert(DriveController_SetAllWheelSpeeds(
        1.50f, -1.50f, 1.50f, -1.50f));
    assert(!DriveController_SetAllWheelSpeeds(
        1.501f, -1.501f, 1.501f, -1.501f));
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

    /* A repeated cmd_vel target must leave room for the due safety query. */
    {
        const unsigned start = s_frame_count;
        assert(DriveController_SetAllWheelSpeeds(
            0.1f, -0.1f, 0.1f, -0.1f));
        run_one_tx();
        assert(s_frame_count == start + 1U);
        assert(s_frames[start].data[0] == 0x0FU);
    }

    /* STOPPING_DRIVE repeatedly requests stop. Only the first request may
     * queue changed zero-speed targets; later calls must leave room for the
     * due safety query. */
    {
        const unsigned start = s_frame_count;
        for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
            DriveController_StopAll();
            run_one_tx();
            assert(s_frames[start + index].data[0] == 0x02U);
        }
        DriveController_StopAll();
        run_one_tx();
        assert(s_frames[start + DRIVE_WHEEL_COUNT].data[0] == 0x0FU);
    }
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
    assert(DriveController_GetTxErrorCount()
           == startup_failed_sends);

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

    feed_safety_feedback(0);
    s_tick += DRIVE_SPEED_FEEDBACK_TIMEOUT_MS + 1U;
    feed_voltage_only();
    assert((DriveController_GetSafetyFlags(DRIVE_WHEEL_FL)
            & DRIVE_SAFETY_STALE) != 0U);
    feed_safety_feedback(0);
    assert(DriveController_IsSafetyFeedbackReady());

    s_tick += DRIVE_TEMPERATURE_FEEDBACK_TIMEOUT_MS + 1U;
    feed_safety_feedback_without_voltage(0);
    assert(DriveController_IsSafetyFeedbackReady());
    s_tick += DRIVE_VOLTAGE_FEEDBACK_TIMEOUT_MS
              - DRIVE_TEMPERATURE_FEEDBACK_TIMEOUT_MS;
    feed_safety_feedback_without_voltage(0);
    assert((DriveController_GetSafetyFlags(DRIVE_WHEEL_FL)
            & DRIVE_SAFETY_STALE) != 0U);
    feed_safety_feedback(0);
    assert(DriveController_IsSafetyFeedbackReady());

    s_can_send_ok = false;
    for (index = 0U; index < startup_failed_sends; ++index) {
        run_one_tx();
    }
    assert(!DriveController_IsHealthy());
    s_can_send_ok = true;
    {
        const unsigned frame_count_before_recovery = s_frame_count;
        for (index = 0U;
             (index < 16U) && (s_frame_count == frame_count_before_recovery);
             ++index) {
            run_one_tx();
        }
        assert(s_frame_count > frame_count_before_recovery);
    }
    feed_safety_feedback(0);
    assert(DriveController_IsSafetyFeedbackReady());
    assert(DriveController_IsHealthy());

    puts("OIDelec MINI drive controller tests passed");
    return 0;
}
