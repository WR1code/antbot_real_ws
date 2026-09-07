#include "rs00_app.h"

#include "rs00_stm32_fdcan.h"

#include <string.h>

#define MOTOR_ID                 RS00_DEFAULT_MOTOR_ID
#define MASTER_ID                RS00_DEFAULT_MASTER_ID
#define STEP_INTERVAL_MS         20U
#define SPEED_TEST_DURATION_MS   2000U
#define SAFE_SPEED_RAD_S         0.2f
#define SAFE_CURRENT_LIMIT_A     0.5f
#define SAFE_ACCEL_RAD_S2        1.0f
#define CAN_TIMEOUT_100_MS       2000U

typedef enum {
    TEST_IDLE = 0,
    TEST_SET_MODE,
    TEST_SET_TIMEOUT,
    TEST_PRESET_CURRENT_LIMIT,
    TEST_PRESET_ACCEL,
    TEST_PRESET_ZERO_SPEED,
    TEST_ENABLE,
    TEST_RUNNING,
    TEST_STOP
} speed_test_state_t;

volatile rs00_command_t g_rs00_command = RS00_COMMAND_NONE;
rs00_app_status_t g_rs00_status;

static speed_test_state_t s_test_state;
static uint32_t s_next_step_tick;
static uint32_t s_test_stop_tick;

static bool tick_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void send_frame(rs00_frame_t frame)
{
    (void)rs00_fdcan_send(&frame);
}

static void send_stop(bool clear_fault)
{
    send_frame(rs00_make_stop(MASTER_ID, MOTOR_ID, clear_fault));
    s_test_state = TEST_IDLE;
}

bool rs00_app_init(FDCAN_HandleTypeDef *hfdcan)
{
    memset(&g_rs00_status, 0, sizeof(g_rs00_status));
    s_test_state = TEST_IDLE;
    g_rs00_command = RS00_COMMAND_NONE;

    if (!rs00_fdcan_start(hfdcan, rs00_app_on_rx)) {
        return false;
    }

    /* Safe default: request a stop before performing read-only discovery. */
    send_stop(false);
    HAL_Delay(10U);
    send_frame(rs00_make_query(MASTER_ID, MOTOR_ID));
    HAL_Delay(10U);
    send_frame(rs00_make_read_param(MASTER_ID, MOTOR_ID, RS00_PARAM_VBUS));
    return true;
}

void rs00_app_poll(void)
{
    const uint32_t now = HAL_GetTick();
    const rs00_command_t command = g_rs00_command;

    if (command != RS00_COMMAND_NONE) {
        g_rs00_command = RS00_COMMAND_NONE;
        switch (command) {
        case RS00_COMMAND_QUERY:
            send_frame(rs00_make_query(MASTER_ID, MOTOR_ID));
            break;
        case RS00_COMMAND_READ_VBUS:
            send_frame(rs00_make_read_param(
                MASTER_ID, MOTOR_ID, RS00_PARAM_VBUS));
            break;
        case RS00_COMMAND_START_SAFE_SPEED_TEST:
            send_stop(false);
            s_test_state = TEST_SET_MODE;
            s_next_step_tick = now + STEP_INTERVAL_MS;
            break;
        case RS00_COMMAND_STOP:
            send_stop(false);
            break;
        case RS00_COMMAND_STOP_AND_CLEAR_FAULT:
            send_stop(true);
            break;
        default:
            break;
        }
    }

    if ((s_test_state == TEST_IDLE) || !tick_reached(now, s_next_step_tick)) {
        return;
    }
    s_next_step_tick = now + STEP_INTERVAL_MS;

    switch (s_test_state) {
    case TEST_SET_MODE:
        send_frame(rs00_make_write_u8(
            MASTER_ID, MOTOR_ID, RS00_PARAM_RUN_MODE, RS00_RUN_SPEED));
        s_test_state = TEST_SET_TIMEOUT;
        break;
    case TEST_SET_TIMEOUT:
        send_frame(rs00_make_write_u32(
            MASTER_ID, MOTOR_ID, RS00_PARAM_CAN_TIMEOUT, CAN_TIMEOUT_100_MS));
        s_test_state = TEST_PRESET_CURRENT_LIMIT;
        break;
    case TEST_PRESET_CURRENT_LIMIT:
        send_frame(rs00_make_write_float(
            MASTER_ID, MOTOR_ID, RS00_PARAM_CURRENT_LIMIT,
            SAFE_CURRENT_LIMIT_A));
        s_test_state = TEST_PRESET_ACCEL;
        break;
    case TEST_PRESET_ACCEL:
        send_frame(rs00_make_write_float(
            MASTER_ID, MOTOR_ID, RS00_PARAM_SPEED_ACCEL,
            SAFE_ACCEL_RAD_S2));
        s_test_state = TEST_PRESET_ZERO_SPEED;
        break;
    case TEST_PRESET_ZERO_SPEED:
        send_frame(rs00_make_write_float(
            MASTER_ID, MOTOR_ID, RS00_PARAM_SPEED_REF, 0.0f));
        s_test_state = TEST_ENABLE;
        break;
    case TEST_ENABLE:
        send_frame(rs00_make_enable(MASTER_ID, MOTOR_ID));
        s_test_stop_tick = now + SPEED_TEST_DURATION_MS;
        s_test_state = TEST_RUNNING;
        break;
    case TEST_RUNNING:
        if (tick_reached(now, s_test_stop_tick)) {
            s_test_state = TEST_STOP;
        } else {
            /*
             * Refresh faster than the configured 100 ms CAN timeout. If the
             * MCU stops sending, the motor's timeout protection can take over.
             */
            send_frame(rs00_make_write_float(
                MASTER_ID, MOTOR_ID, RS00_PARAM_SPEED_REF,
                SAFE_SPEED_RAD_S));
        }
        break;
    case TEST_STOP:
        send_stop(false);
        break;
    default:
        s_test_state = TEST_IDLE;
        break;
    }
}

void rs00_app_on_rx(const rs00_frame_t *frame)
{
    const uint8_t type = rs00_get_type(frame->id);
    rs00_feedback_t feedback;
    float vbus;

    g_rs00_status.last_rx_tick_ms = HAL_GetTick();

    if ((type == RS00_TYPE_GET_DEVICE_ID)
        && (((frame->id >> 8) & 0xFFU) == MOTOR_ID)
        && ((frame->id & 0xFFU) == 0xFEU)) {
        size_t index;
        for (index = 0U; index < 8U; ++index) {
            g_rs00_status.uid[index] = frame->data[index];
        }
        g_rs00_status.device_seen = true;
    } else if (rs00_decode_feedback(frame, &feedback)
               && (feedback.motor_id == MOTOR_ID)
               && (feedback.master_id == MASTER_ID)) {
        g_rs00_status.feedback = feedback;
        g_rs00_status.feedback_valid = true;
    } else if (rs00_decode_read_float(frame, MOTOR_ID, MASTER_ID,
                                      RS00_PARAM_VBUS, &vbus)) {
        g_rs00_status.vbus_v = vbus;
        g_rs00_status.vbus_valid = true;
    }
}

void rs00_app_emergency_stop(void)
{
    send_stop(false);
}
