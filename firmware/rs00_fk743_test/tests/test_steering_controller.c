#include "steering_controller.h"
#include "steering_config.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_tick;
static uint32_t s_enable_count;
static uint32_t s_stop_count;
static bool s_enabled[STEERING_MOTOR_COUNT];
static bool s_suppress_feedback[STEERING_MOTOR_COUNT];
static float s_positions[STEERING_MOTOR_COUNT] = {0.10f, 0.20f, 0.30f, 0.40f};
static float s_loc_refs[STEERING_MOTOR_COUNT];
static uint8_t s_run_modes[STEERING_MOTOR_COUNT];
static uint32_t s_limit_speed_writes;
static uint32_t s_limit_current_writes;
static uint32_t s_pp_parameter_writes;
static float s_expected_limit_speed = STEERING_CSP_LIMIT_SPEED_RAD_S;
static float s_expected_limit_current = STEERING_CSP_LIMIT_CURRENT_A;
static uint32_t make_reply_id(uint8_t type, uint8_t status,
                              uint8_t motor_id, uint8_t destination)
{
    return ((uint32_t)type << 24) | ((uint32_t)status << 16)
           | ((uint32_t)motor_id << 8) | destination;
}

static void write_float_le(uint8_t output[4], float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    output[0] = (uint8_t)bits;
    output[1] = (uint8_t)(bits >> 8);
    output[2] = (uint8_t)(bits >> 16);
    output[3] = (uint8_t)(bits >> 24);
}

static void send_feedback(uint8_t motor_id)
{
    uint8_t data[8] = {
        0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA
    };
    const uint8_t state = s_enabled[motor_id - 1U] ? 2U : 0U;
    if (s_suppress_feedback[motor_id - 1U]) {
        return;
    }
    SteeringController_OnCanFrame(
        make_reply_id(RS00_TYPE_FEEDBACK, (uint8_t)(state << 6),
                      motor_id, STEERING_HOST_ID),
        data);
}

static void respond_to_tx(const FDCAN_TxHeaderTypeDef *header,
                          const uint8_t data[8])
{
    const uint8_t type = (uint8_t)((header->Identifier >> 24) & 0x1FU);
    const uint8_t motor_id = (uint8_t)header->Identifier;
    uint8_t response[8] = {0};

    if (type == RS00_TYPE_STOP) {
        ++s_stop_count;
        s_enabled[motor_id - 1U] = false;
        send_feedback(motor_id);
    } else if (type == RS00_TYPE_GET_DEVICE_ID) {
        size_t index;
        for (index = 0U; index < 8U; ++index) {
            response[index] = (uint8_t)(motor_id * 0x10U + index + 1U);
        }
        SteeringController_OnCanFrame(
            make_reply_id(RS00_TYPE_GET_DEVICE_ID, 0U,
                          motor_id, 0xFEU),
            response);
    } else if (type == RS00_TYPE_WRITE_PARAM) {
        const uint16_t parameter =
            (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        float value;
        memcpy(&value, &data[4], sizeof(value));
        if (parameter == RS00_PARAM_RUN_MODE) {
            s_run_modes[motor_id - 1U] = data[4];
        } else if (parameter == RS00_PARAM_LIMIT_SPD) {
            ++s_limit_speed_writes;
            assert(fabsf(value - s_expected_limit_speed) < 0.001f);
        } else if (parameter == RS00_PARAM_LIMIT_CUR) {
            ++s_limit_current_writes;
            assert(fabsf(value - s_expected_limit_current) < 0.001f);
        } else if ((parameter == RS00_PARAM_VEL_MAX)
                   || (parameter == RS00_PARAM_ACC_SET)) {
            ++s_pp_parameter_writes;
        } else if (parameter == RS00_PARAM_LOC_REF) {
            s_loc_refs[motor_id - 1U] = value;
        }
        send_feedback(motor_id);
    } else if (type == RS00_TYPE_READ_PARAM) {
        const uint16_t parameter =
            (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        response[0] = data[0];
        response[1] = data[1];
        if (parameter == RS00_PARAM_MECH_POS
            || parameter == RS00_PARAM_LOC_REF) {
            write_float_le(&response[4],
                           parameter == RS00_PARAM_MECH_POS
                               ? s_positions[motor_id - 1U]
                               : s_loc_refs[motor_id - 1U]);
        } else if (parameter == RS00_PARAM_RUN_MODE) {
            response[4] = s_run_modes[motor_id - 1U];
        } else if (parameter == RS00_PARAM_LIMIT_CUR) {
            write_float_le(&response[4], s_expected_limit_current);
        } else if (parameter == RS00_PARAM_LIMIT_SPD) {
            write_float_le(&response[4], s_expected_limit_speed);
        } else if (parameter == RS00_PARAM_CAN_TIMEOUT) {
            const uint32_t value = STEERING_CAN_TIMEOUT_COUNTS;
            response[4] = (uint8_t)value;
            response[5] = (uint8_t)(value >> 8);
            response[6] = (uint8_t)(value >> 16);
            response[7] = (uint8_t)(value >> 24);
        } else {
            assert(!"unexpected parameter read");
        }
        SteeringController_OnCanFrame(
            make_reply_id(RS00_TYPE_READ_PARAM, 0U,
                          motor_id, STEERING_HOST_ID),
            response);
    } else if (type == RS00_TYPE_ENABLE) {
        ++s_enable_count;
        s_enabled[motor_id - 1U] = true;
        /* Model a real sequential enable delay.  Four motors can exceed the
         * normal 200 ms READY watchdog before VERIFY_ALL is reached. */
        s_tick += 75U;
        send_feedback(motor_id);
    } else {
        assert(!"unexpected command");
    }
}

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

void HAL_Delay(uint32_t delay_ms)
{
    assert(!"controller must not call HAL_Delay");
    s_tick += delay_ms;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *filter)
{
    (void)hfdcan;
    (void)filter;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(
    FDCAN_HandleTypeDef *hfdcan, uint32_t non_matching_standard,
    uint32_t non_matching_extended, uint32_t reject_standard_remote,
    uint32_t reject_extended_remote)
{
    (void)hfdcan;
    (void)non_matching_standard;
    (void)non_matching_extended;
    (void)reject_standard_remote;
    (void)reject_extended_remote;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(
    FDCAN_HandleTypeDef *hfdcan, uint32_t active_it, uint32_t buffer_indexes)
{
    (void)hfdcan;
    (void)active_it;
    (void)buffer_indexes;
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return 8U;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    (void)hfdcan;
    assert(header->IdType == FDCAN_EXTENDED_ID);
    assert(header->TxFrameType == FDCAN_DATA_FRAME);
    assert(header->DataLength == FDCAN_DLC_BYTES_8);
    assert(header->BitRateSwitch == FDCAN_BRS_OFF);
    assert(header->FDFormat == FDCAN_CLASSIC_CAN);
    respond_to_tx(header, data);
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(
    FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo)
{
    (void)hfdcan;
    (void)rx_fifo;
    return 0U;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(
    FDCAN_HandleTypeDef *hfdcan, uint32_t rx_location,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)hfdcan;
    (void)rx_location;
    (void)header;
    (void)data;
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *status)
{
    (void)hfdcan;
    memset(status, 0, sizeof(*status));
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_ErrorCountersTypeDef *counters)
{
    (void)hfdcan;
    memset(counters, 0, sizeof(*counters));
    return HAL_OK;
}

static void run_until(SteeringState expected)
{
    unsigned iteration;
    for (iteration = 0U; iteration < 1000U; ++iteration) {
        SteeringController_Task();
        if (SteeringController_GetState() == expected) {
            return;
        }
        ++s_tick;
    }
    assert(!"state machine did not reach expected state");
}

int main(void)
{
    FDCAN_HandleTypeDef hfdcan = {0};
    SteeringMotor snapshot;

    SteeringController_Init(&hfdcan);
    assert(SteeringController_GetState() == STEERING_STATE_BOOT_WAIT);
    assert(s_enable_count == 0U);
    s_tick = STEERING_MOTOR_BOOT_DELAY_MS;
    run_until(STEERING_STATE_ARMED);
    assert(!SteeringController_IsReady());
    assert(s_enable_count == 0U);
    assert(s_stop_count == STEERING_MOTOR_COUNT);
    assert(s_limit_speed_writes == STEERING_MOTOR_COUNT);
    assert(s_limit_current_writes == STEERING_MOTOR_COUNT);
    assert(s_pp_parameter_writes == 0U);
    for (unsigned index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        assert(s_run_modes[index] == RS00_RUN_CSP);
        assert(fabsf(s_loc_refs[index] - s_positions[index]) < 0.001f);
    }
    assert(!SteeringController_SetAllAngles(0.1f, 0.2f, 0.3f, 0.4f));

    assert(SteeringController_GetMotorSnapshot(STEERING_MOTOR_RR, &snapshot));
    assert(snapshot.uid_received);
    assert(snapshot.initialized);
    assert(fabsf(snapshot.startup_position_rad - 0.4f) < 0.001f);

    assert(SteeringController_RequestEnable());
    run_until(STEERING_STATE_READY);
    assert(SteeringController_IsReady());
    assert(s_enable_count == STEERING_MOTOR_COUNT);

    /* A reply during stale confirmation must cancel the pending fault. */
    s_suppress_feedback[STEERING_MOTOR_RL] = true;
    s_tick += STEERING_FEEDBACK_TIMEOUT_MS + 1U;
    SteeringController_Task();
    assert(SteeringController_IsReady());
    s_tick += STEERING_FEEDBACK_CONFIRM_MS - 1U;
    s_suppress_feedback[STEERING_MOTOR_RL] = false;
    send_feedback(STEERING_MOTOR_ID_RL);
    SteeringController_Task();
    assert(SteeringController_IsReady());

    assert(SteeringController_SetAllAngles(0.11f, 0.21f, 0.31f, 0.41f));
    assert(!SteeringController_SetAllAngles(-0.01f, 0.2f, 0.3f, 0.4f));
    assert(SteeringController_GetState() == STEERING_STATE_FAULT);

    assert(SteeringController_ClearFaultAndRestart());
    s_tick += STEERING_MOTOR_BOOT_DELAY_MS;
    run_until(STEERING_STATE_ARMED);
    assert(SteeringController_RequestEnable());
    run_until(STEERING_STATE_READY);
    assert(!SteeringController_SetAllAngles(
        STEERING_MECH_MAX_RAD + 0.01f, 0.2f, 0.3f, 0.4f));
    assert(SteeringController_GetState() == STEERING_STATE_FAULT);

    s_expected_limit_speed = 0.75f;
    s_expected_limit_current = 1.5f;
    assert(SteeringController_SetCspLimitsAndRestart(
        s_expected_limit_speed, s_expected_limit_current));
    s_tick += STEERING_MOTOR_BOOT_DELAY_MS;
    run_until(STEERING_STATE_ARMED);
    assert(!SteeringController_SetCspLimitsAndRestart(0.0f, 1.0f));

    s_positions[0] = -0.01f;
    SteeringController_Init(&hfdcan);
    s_tick += STEERING_MOTOR_BOOT_DELAY_MS;
    run_until(STEERING_STATE_FAULT);
    assert(g_steering_debug_error == STEERING_ERROR_MECHANICAL_RANGE);
    s_positions[0] = STEERING_MECH_MAX_RAD + 0.01f;
    SteeringController_Init(&hfdcan);
    s_tick += STEERING_MOTOR_BOOT_DELAY_MS;
    run_until(STEERING_STATE_FAULT);
    assert(g_steering_debug_error == STEERING_ERROR_MECHANICAL_RANGE);
    puts("Steering controller state-machine test passed");
    return 0;
}
