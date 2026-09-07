#include "steering_controller.h"

#include "chassis_debug.h"
#include "rs00_stm32_fdcan.h"
#include "steering_config.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

_Static_assert(sizeof(float) == 4U, "RS00 requires 32-bit float");

volatile SteeringState g_steering_debug_state = STEERING_STATE_IDLE;
volatile SteeringError g_steering_debug_error = STEERING_ERROR_NONE;
volatile uint8_t g_steering_debug_fault_motor_id;
volatile uint32_t
    g_steering_debug_feedback_age_ms[STEERING_MOTOR_COUNT];
SteeringCanTrace g_steering_can_trace;

static const char *const s_motor_names[STEERING_MOTOR_COUNT] =
    {"FL", "FR", "RL", "RR"};
static const uint8_t s_motor_ids[STEERING_MOTOR_COUNT] = STEERING_MOTOR_IDS;
static const uint8_t s_expected_uids[STEERING_MOTOR_COUNT][8] =
    STEERING_EXPECTED_UIDS;
static const float s_direction[STEERING_MOTOR_COUNT] =
    STEERING_DIRECTION_SIGNS;
static const float s_zero_offset[STEERING_MOTOR_COUNT] =
    STEERING_ZERO_OFFSETS_RAD;
static const float s_min_position[STEERING_MOTOR_COUNT] =
    STEERING_MIN_POSITIONS_RAD;
static const float s_max_position[STEERING_MOTOR_COUNT] =
    STEERING_MAX_POSITIONS_RAD;

static SteeringMotor s_motors[STEERING_MOTOR_COUNT];
static SteeringState s_state;
static uint8_t s_motor_index;
static uint16_t s_parameter_index;
static uint32_t s_state_tick;
static uint32_t s_request_tick;
static uint32_t s_last_target_tick;
static uint32_t s_wait_sequence;
static bool s_ready;
static bool s_enable_requested;
static bool s_feedback_stale_pending[STEERING_MOTOR_COUNT];
static uint32_t s_feedback_stale_since[STEERING_MOTOR_COUNT];
static uint32_t s_feedback_stale_last_rx[STEERING_MOTOR_COUNT];
static float s_csp_limit_speed_rad_s = STEERING_CSP_LIMIT_SPEED_RAD_S;
static float s_csp_limit_current_a = STEERING_CSP_LIMIT_CURRENT_A;

static volatile uint32_t s_feedback_sequence[STEERING_MOTOR_COUNT];
static volatile uint32_t s_uid_sequence[STEERING_MOTOR_COUNT];
static volatile uint32_t s_parameter_sequence[STEERING_MOTOR_COUNT];
static volatile uint8_t s_parameter_result[STEERING_MOTOR_COUNT];
static volatile uint16_t s_parameter_reply_index[STEERING_MOTOR_COUNT];
static volatile uint8_t s_parameter_raw[STEERING_MOTOR_COUNT][4];

#if defined(__GNUC__)
__attribute__((weak))
#endif
void SteeringController_Log(SteeringLogLevel level, const char *message)
{
    (void)level;
    (void)message;
}

static void log_message(SteeringLogLevel level, const char *format, ...)
{
#if STEERING_LOG_LEVEL > 0
    char buffer[160];
    va_list args;

    if ((int)level > STEERING_LOG_LEVEL) {
        return;
    }
    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    SteeringController_Log(level, buffer);
#else
    (void)level;
    (void)format;
#endif
}

static uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
    return now - then;
}

static int motor_index_from_id(uint8_t motor_id)
{
    size_t index;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if (s_motor_ids[index] == motor_id) {
            return (int)index;
        }
    }
    return -1;
}

static uint16_t read_le_u16(const uint8_t data[2])
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t read_le_u32(const uint8_t data[4])
{
    return (uint32_t)data[0]
           | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16)
           | ((uint32_t)data[3] << 24);
}

static float raw_to_float(const uint8_t raw[4])
{
    const uint32_t bits = read_le_u32(raw);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool uid_is_valid(const uint8_t uid[8])
{
    size_t index;
    bool all_zero = true;
    bool all_ff = true;
    for (index = 0U; index < 8U; ++index) {
        all_zero = all_zero && (uid[index] == 0U);
        all_ff = all_ff && (uid[index] == 0xFFU);
    }
    return !all_zero && !all_ff;
}

static bool has_motor_fault(const SteeringMotor *motor)
{
    return (motor->fault != 0U);
}

static void set_state(SteeringState next)
{
    if (s_state != next) {
        s_state = next;
        g_steering_debug_state = next;
        s_state_tick = HAL_GetTick();
        log_message(STEERING_LOG_INFO, "steering state=%u motor=%s id=0x%02X",
                    (unsigned)next, s_motor_names[s_motor_index],
                    s_motor_ids[s_motor_index]);
    }
}

static SteeringError map_tx_error(rs00_fdcan_result_t result)
{
    if (result == RS00_FDCAN_TX_FIFO_FULL) {
        return STEERING_ERROR_TX_FIFO_FULL;
    }
    if ((result == RS00_FDCAN_INVALID_ARGUMENT)
        || (result == RS00_FDCAN_NOT_STARTED)) {
        return STEERING_ERROR_ARGUMENT;
    }
    return STEERING_ERROR_HAL_TX;
}

static void trace_tx(const rs00_frame_t *frame)
{
    size_t index;
    g_steering_can_trace.last_tx_id = frame->id;
    for (index = 0U; index < 8U; ++index) {
        g_steering_can_trace.last_tx_data[index] = frame->data[index];
    }
    ++g_steering_can_trace.tx_count;
    log_message(STEERING_LOG_DEBUG,
                "TX %08lX %02X %02X %02X %02X %02X %02X %02X %02X",
                (unsigned long)frame->id,
                frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
}

static bool send_frame(const rs00_frame_t *frame)
{
    const rs00_fdcan_result_t result = rs00_fdcan_send_detailed(frame);
    if (result != RS00_FDCAN_OK) {
        s_motors[s_motor_index].last_error = map_tx_error(result);
        return false;
    }
    trace_tx(frame);
    s_request_tick = HAL_GetTick();
    return true;
}

static void stop_frames_only(bool clear_fault)
{
    size_t index;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        const rs00_frame_t frame =
            rs00_make_stop(STEERING_HOST_ID, s_motor_ids[index], clear_fault);
        if (rs00_fdcan_send_detailed(&frame) == RS00_FDCAN_OK) {
            trace_tx(&frame);
        }
        s_motors[index].enabled = false;
    }
    s_ready = false;
}

static void enter_fault(SteeringError error)
{
    SteeringMotor *motor = &s_motors[s_motor_index];
    uint32_t now;
    uint32_t primask;
    size_t index;
    primask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        g_steering_debug_feedback_age_ms[index] =
            elapsed_ms(now, s_motors[index].last_rx_tick);
    }
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    motor->last_error = error;
    g_steering_debug_error = error;
    g_steering_debug_fault_motor_id = motor->motor_id;
    s_enable_requested = false;
    stop_frames_only(false);
    set_state(STEERING_STATE_FAULT);
    ChassisDebug_SetFault(CHASSIS_FAULT_STEERING);
    log_message(STEERING_LOG_ERROR, "FAULT motor=%s id=0x%02X error=%u",
                motor->name, motor->motor_id, (unsigned)error);
}

static bool retry_or_fault(SteeringState send_state, SteeringError error)
{
    SteeringMotor *motor = &s_motors[s_motor_index];
    if (motor->retry_count < STEERING_MAX_RETRIES) {
        ++motor->retry_count;
        motor->last_error = error;
        log_message(STEERING_LOG_WARN,
                    "retry motor=%s id=0x%02X count=%u error=%u",
                    motor->name, motor->motor_id, motor->retry_count,
                    (unsigned)error);
        set_state(send_state);
        return true;
    }
    enter_fault(error);
    return false;
}

static bool response_timed_out(void)
{
    return elapsed_ms(HAL_GetTick(), s_request_tick)
           >= STEERING_RESPONSE_TIMEOUT_MS;
}

static void advance_motor_or_state(SteeringState repeat_state,
                                   SteeringState next_state)
{
    s_motors[s_motor_index].retry_count = 0U;
    ++s_motor_index;
    if (s_motor_index < STEERING_MOTOR_COUNT) {
        set_state(repeat_state);
    } else {
        s_motor_index = 0U;
        set_state(next_state);
    }
}

static bool send_and_wait(const rs00_frame_t *frame,
                          uint32_t response_sequence,
                          SteeringState wait_state,
                          SteeringState retry_state)
{
    s_wait_sequence = response_sequence;
    if (!send_frame(frame)) {
        (void)retry_or_fault(retry_state,
                             s_motors[s_motor_index].last_error);
        return false;
    }
    set_state(wait_state);
    return true;
}

static bool wait_for_feedback(SteeringState retry_state)
{
    SteeringMotor *motor = &s_motors[s_motor_index];
    if (s_feedback_sequence[s_motor_index] != s_wait_sequence) {
        __DMB();
        if (has_motor_fault(motor)) {
            enter_fault(STEERING_ERROR_MOTOR_FAULT);
            return false;
        }
        motor->retry_count = 0U;
        return true;
    }
    if (response_timed_out()) {
        (void)retry_or_fault(retry_state, STEERING_ERROR_TIMEOUT);
    }
    return false;
}

static bool wait_for_parameter(SteeringState retry_state, uint8_t raw[4])
{
    size_t byte_index;
    if (s_parameter_sequence[s_motor_index] != s_wait_sequence) {
        __DMB();
        if (s_parameter_result[s_motor_index] != 0U) {
            enter_fault(STEERING_ERROR_PARAM_READ);
            return false;
        }
        if (s_parameter_reply_index[s_motor_index] != s_parameter_index) {
            enter_fault(STEERING_ERROR_PARAM_INDEX);
            return false;
        }
        for (byte_index = 0U; byte_index < 4U; ++byte_index) {
            raw[byte_index] = s_parameter_raw[s_motor_index][byte_index];
        }
        s_motors[s_motor_index].retry_count = 0U;
        return true;
    }
    if (response_timed_out()) {
        (void)retry_or_fault(retry_state, STEERING_ERROR_TIMEOUT);
    }
    return false;
}

static void send_parameter_read(uint16_t index, SteeringState wait_state,
                                SteeringState retry_state)
{
    const rs00_frame_t frame =
        rs00_make_read_param(STEERING_HOST_ID,
                             s_motor_ids[s_motor_index], index);
    s_parameter_index = index;
    (void)send_and_wait(&frame, s_parameter_sequence[s_motor_index],
                        wait_state, retry_state);
}

static void send_float_write(uint16_t index, float value,
                             SteeringState wait_state,
                             SteeringState retry_state)
{
    const rs00_frame_t frame =
        rs00_make_write_float(STEERING_HOST_ID,
                              s_motor_ids[s_motor_index], index, value);
    s_parameter_index = index;
    (void)send_and_wait(&frame, s_feedback_sequence[s_motor_index],
                        wait_state, retry_state);
}

static bool verify_float_reply(SteeringState retry_state, float expected)
{
    uint8_t raw[4];
    float actual;
    if (!wait_for_parameter(retry_state, raw)) {
        return false;
    }
    actual = raw_to_float(raw);
    if (!isfinite(actual) || (fabsf(actual - expected)
                              > STEERING_FLOAT_TOLERANCE)) {
        enter_fault(STEERING_ERROR_PARAM_VERIFY);
        return false;
    }
    return true;
}

static void parse_feedback(const rs00_frame_t *frame)
{
    rs00_feedback_t feedback;
    int index;
    if (!rs00_decode_feedback(frame, &feedback)
        || (feedback.master_id != STEERING_HOST_ID)) {
        return;
    }
    index = motor_index_from_id(feedback.motor_id);
    if (index < 0) {
        return;
    }
    s_motors[index].mode_state = (uint8_t)feedback.state;
    s_motors[index].position_rad = feedback.position_rad;
    s_motors[index].velocity_rad_s = feedback.velocity_rad_s;
    s_motors[index].torque_nm = feedback.torque_nm;
    s_motors[index].temperature_c = feedback.temperature_c;
    s_motors[index].fault =
        (frame->id >> 16) & 0x3FU;
    s_motors[index].online = true;
    s_motors[index].last_rx_tick = HAL_GetTick();
    __DMB();
    ++s_feedback_sequence[index];
}

static void parse_uid(const rs00_frame_t *frame)
{
    const uint8_t motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    int index;
    size_t byte_index;
    if ((frame->id & 0xFFU) != 0xFEU) {
        return;
    }
    index = motor_index_from_id(motor_id);
    if (index < 0) {
        return;
    }
    for (byte_index = 0U; byte_index < 8U; ++byte_index) {
        s_motors[index].received_uid[byte_index] = frame->data[byte_index];
    }
    s_motors[index].uid_received = true;
    s_motors[index].online = true;
    s_motors[index].last_rx_tick = HAL_GetTick();
    __DMB();
    ++s_uid_sequence[index];
}

static void parse_parameter(const rs00_frame_t *frame)
{
    const uint8_t motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    const uint8_t host_id = (uint8_t)frame->id;
    int index;
    size_t byte_index;
    if (host_id != STEERING_HOST_ID) {
        return;
    }
    index = motor_index_from_id(motor_id);
    if (index < 0) {
        return;
    }
    s_parameter_result[index] = (uint8_t)((frame->id >> 16) & 0xFFU);
    s_parameter_reply_index[index] = read_le_u16(frame->data);
    for (byte_index = 0U; byte_index < 4U; ++byte_index) {
        s_parameter_raw[index][byte_index] = frame->data[4U + byte_index];
    }
    s_motors[index].last_rx_tick = HAL_GetTick();
    __DMB();
    ++s_parameter_sequence[index];
}

static void parse_fault(const rs00_frame_t *frame)
{
    const uint8_t motor_id = (uint8_t)((frame->id >> 8) & 0xFFU);
    int index = motor_index_from_id(motor_id);
    size_t byte_index;
    if ((index < 0) || (((uint8_t)frame->id) != STEERING_HOST_ID)) {
        return;
    }
    for (byte_index = 0U; byte_index < 8U; ++byte_index) {
        s_motors[index].fault_raw[byte_index] = frame->data[byte_index];
    }
    /*
     * Preserve all raw bytes because field endianness varies by firmware.
     * Either endian interpretation is non-zero iff any corresponding byte is.
     */
    s_motors[index].fault = read_le_u32(&frame->data[0]);
    s_motors[index].warning = read_le_u32(&frame->data[4]);
    s_motors[index].last_rx_tick = HAL_GetTick();
    __DMB();
}

void SteeringController_OnCanFrame(uint32_t ext_id, const uint8_t data[8])
{
    rs00_frame_t frame;
    size_t index;
    uint8_t type;
    if ((data == NULL) || (ext_id > RS00_EXT_ID_MASK)) {
        return;
    }
    frame.id = ext_id;
    for (index = 0U; index < 8U; ++index) {
        frame.data[index] = data[index];
        g_steering_can_trace.last_rx_data[index] = data[index];
    }
    g_steering_can_trace.last_rx_id = ext_id;
    ++g_steering_can_trace.rx_count;

    type = rs00_get_type(ext_id);
    if (type == RS00_TYPE_GET_DEVICE_ID) {
        parse_uid(&frame);
    } else if (type == RS00_TYPE_FEEDBACK) {
        parse_feedback(&frame);
    } else if (type == RS00_TYPE_READ_PARAM) {
        parse_parameter(&frame);
    } else if (type == RS00_TYPE_FAULT) {
        parse_fault(&frame);
    }
}

static void fdcan_rx_adapter(const rs00_frame_t *frame)
{
    if (frame != NULL) {
        SteeringController_OnCanFrame(frame->id, frame->data);
    }
}

void SteeringController_Init(FDCAN_HandleTypeDef *hfdcan)
{
    size_t index;
    memset(s_motors, 0, sizeof(s_motors));
    memset(&g_steering_can_trace, 0, sizeof(g_steering_can_trace));
    memset((void *)s_feedback_sequence, 0, sizeof(s_feedback_sequence));
    memset((void *)s_uid_sequence, 0, sizeof(s_uid_sequence));
    memset((void *)s_parameter_sequence, 0, sizeof(s_parameter_sequence));
    memset(s_feedback_stale_pending, 0,
           sizeof(s_feedback_stale_pending));
    memset(s_feedback_stale_since, 0, sizeof(s_feedback_stale_since));
    memset(s_feedback_stale_last_rx, 0,
           sizeof(s_feedback_stale_last_rx));

    s_motor_index = 0U;
    s_ready = false;
    s_enable_requested = false;
    g_steering_debug_error = STEERING_ERROR_NONE;
    g_steering_debug_fault_motor_id = 0U;
    memset((void *)g_steering_debug_feedback_age_ms, 0,
           sizeof(g_steering_debug_feedback_age_ms));
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].name = s_motor_names[index];
        s_motors[index].motor_id = s_motor_ids[index];
        memcpy(s_motors[index].expected_uid, s_expected_uids[index], 8U);
    }
    if ((hfdcan == NULL) || !rs00_fdcan_start(hfdcan, fdcan_rx_adapter)) {
        s_state = STEERING_STATE_IDLE;
        enter_fault(STEERING_ERROR_FDCAN_START);
        return;
    }
    s_state = STEERING_STATE_IDLE;
    set_state(STEERING_STATE_BOOT_WAIT);
}

static void task_stop(void)
{
    if (s_state == STEERING_STATE_STOP_SEND) {
        const rs00_frame_t frame =
            rs00_make_stop(STEERING_HOST_ID,
                           s_motor_ids[s_motor_index], false);
        (void)send_and_wait(&frame, s_feedback_sequence[s_motor_index],
                            STEERING_STATE_STOP_WAIT,
                            STEERING_STATE_STOP_SEND);
    } else if (wait_for_feedback(STEERING_STATE_STOP_SEND)) {
        advance_motor_or_state(STEERING_STATE_STOP_SEND,
                               STEERING_STATE_QUERY_UID_SEND);
    }
}

static void task_uid(void)
{
    SteeringMotor *motor = &s_motors[s_motor_index];
    if (s_state == STEERING_STATE_QUERY_UID_SEND) {
        const rs00_frame_t frame =
            rs00_make_query(STEERING_HOST_ID, motor->motor_id);
        (void)send_and_wait(&frame, s_uid_sequence[s_motor_index],
                            STEERING_STATE_QUERY_UID_WAIT,
                            STEERING_STATE_QUERY_UID_SEND);
    } else if (s_uid_sequence[s_motor_index] != s_wait_sequence) {
        __DMB();
        if (!uid_is_valid(motor->received_uid)) {
            enter_fault(STEERING_ERROR_INVALID_UID);
        } else if ((STEERING_ENFORCE_UID_CHECK != 0)
                   && (memcmp(motor->received_uid, motor->expected_uid, 8U)
                       != 0)) {
            enter_fault(STEERING_ERROR_UID_MISMATCH);
        } else {
            log_message(STEERING_LOG_INFO,
                        "UID %s %02X%02X%02X%02X%02X%02X%02X%02X",
                        motor->name,
                        motor->received_uid[0], motor->received_uid[1],
                        motor->received_uid[2], motor->received_uid[3],
                        motor->received_uid[4], motor->received_uid[5],
                        motor->received_uid[6], motor->received_uid[7]);
            advance_motor_or_state(STEERING_STATE_QUERY_UID_SEND,
                                   STEERING_STATE_READ_POSITION_SEND);
        }
    } else if (response_timed_out()) {
        (void)retry_or_fault(STEERING_STATE_QUERY_UID_SEND,
                             STEERING_ERROR_TIMEOUT);
    }
}

static void task_read_position(void)
{
    uint8_t raw[4];
    SteeringMotor *motor = &s_motors[s_motor_index];
    if (s_state == STEERING_STATE_READ_POSITION_SEND) {
        send_parameter_read(RS00_PARAM_MECH_POS,
                            STEERING_STATE_READ_POSITION_WAIT,
                            STEERING_STATE_READ_POSITION_SEND);
    } else if (wait_for_parameter(STEERING_STATE_READ_POSITION_SEND, raw)) {
        memcpy(motor->startup_position_raw, raw, 4U);
        motor->startup_position_rad = raw_to_float(raw);
        motor->target_position_rad = motor->startup_position_rad;
        advance_motor_or_state(STEERING_STATE_READ_POSITION_SEND,
                               STEERING_STATE_VALIDATE_POSITION);
    }
}

static void task_validate_positions(void)
{
    size_t index;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        const float position = s_motors[index].startup_position_rad;
        if (!isfinite(position)
            || ((STEERING_ENABLE_MECHANICAL_LIMIT_CHECK != 0)
                && ((position < (s_min_position[index]
                                 - STEERING_LIMIT_EPSILON_RAD))
                    || (position > (s_max_position[index]
                                    + STEERING_LIMIT_EPSILON_RAD))))) {
            s_motor_index = (uint8_t)index;
            enter_fault(isfinite(position)
                            ? STEERING_ERROR_MECHANICAL_RANGE
                            : STEERING_ERROR_INVALID_POSITION);
            return;
        }
        if (position < s_min_position[index]) {
            s_motors[index].startup_position_rad = s_min_position[index];
        } else if (position > s_max_position[index]) {
            s_motors[index].startup_position_rad = s_max_position[index];
        }
        s_motors[index].target_position_rad =
            s_motors[index].startup_position_rad;
    }
    s_motor_index = 0U;
    set_state(STEERING_STATE_SET_MODE_SEND);
}

static void task_set_mode(void)
{
    if (s_state == STEERING_STATE_SET_MODE_SEND) {
        const rs00_frame_t frame =
            rs00_make_write_u8(STEERING_HOST_ID, s_motor_ids[s_motor_index],
                               RS00_PARAM_RUN_MODE, RS00_RUN_CSP);
        s_parameter_index = RS00_PARAM_RUN_MODE;
        (void)send_and_wait(&frame, s_feedback_sequence[s_motor_index],
                            STEERING_STATE_SET_MODE_WAIT,
                            STEERING_STATE_SET_MODE_SEND);
    } else if (s_state == STEERING_STATE_SET_MODE_WAIT) {
        if (wait_for_feedback(STEERING_STATE_SET_MODE_SEND)) {
            set_state(STEERING_STATE_VERIFY_MODE_SEND);
        }
    } else if (s_state == STEERING_STATE_VERIFY_MODE_SEND) {
        send_parameter_read(RS00_PARAM_RUN_MODE,
                            STEERING_STATE_VERIFY_MODE_WAIT,
                            STEERING_STATE_VERIFY_MODE_SEND);
    } else {
        uint8_t raw[4];
        if (wait_for_parameter(STEERING_STATE_VERIFY_MODE_SEND, raw)) {
            if ((raw[0] != RS00_RUN_CSP) || (raw[1] != 0U)
                || (raw[2] != 0U) || (raw[3] != 0U)) {
                enter_fault(STEERING_ERROR_PARAM_VERIFY);
            } else {
                advance_motor_or_state(STEERING_STATE_SET_MODE_SEND,
                                       STEERING_STATE_SET_LIMIT_SPEED_SEND);
            }
        }
    }
}

static void task_float_parameter(uint16_t index, float expected,
                                 SteeringState send_state,
                                 SteeringState wait_state,
                                 SteeringState verify_send_state,
                                 SteeringState verify_wait_state,
                                 SteeringState next_state)
{
    if (s_state == send_state) {
        send_float_write(index, expected, wait_state, send_state);
    } else if (s_state == wait_state) {
        if (wait_for_feedback(send_state)) {
            set_state(verify_send_state);
        }
    } else if (s_state == verify_send_state) {
        send_parameter_read(index, verify_wait_state, verify_send_state);
    } else if (verify_float_reply(verify_send_state, expected)) {
        advance_motor_or_state(send_state, next_state);
    }
}

static void task_timeout_parameter(void)
{
    if (s_state == STEERING_STATE_SET_TIMEOUT_SEND) {
        const rs00_frame_t frame =
            rs00_make_write_u32(STEERING_HOST_ID, s_motor_ids[s_motor_index],
                                RS00_PARAM_CAN_TIMEOUT,
                                STEERING_CAN_TIMEOUT_COUNTS);
        s_parameter_index = RS00_PARAM_CAN_TIMEOUT;
        (void)send_and_wait(&frame, s_feedback_sequence[s_motor_index],
                            STEERING_STATE_SET_TIMEOUT_WAIT,
                            STEERING_STATE_SET_TIMEOUT_SEND);
    } else if (s_state == STEERING_STATE_SET_TIMEOUT_WAIT) {
        if (wait_for_feedback(STEERING_STATE_SET_TIMEOUT_SEND)) {
            set_state(STEERING_STATE_VERIFY_TIMEOUT_SEND);
        }
    } else if (s_state == STEERING_STATE_VERIFY_TIMEOUT_SEND) {
        send_parameter_read(RS00_PARAM_CAN_TIMEOUT,
                            STEERING_STATE_VERIFY_TIMEOUT_WAIT,
                            STEERING_STATE_VERIFY_TIMEOUT_SEND);
    } else {
        uint8_t raw[4];
        if (wait_for_parameter(STEERING_STATE_VERIFY_TIMEOUT_SEND, raw)) {
            if (read_le_u32(raw) != STEERING_CAN_TIMEOUT_COUNTS) {
                enter_fault(STEERING_ERROR_PARAM_VERIFY);
            } else {
                advance_motor_or_state(STEERING_STATE_SET_TIMEOUT_SEND,
                                       STEERING_STATE_PRELOAD_POSITION_SEND);
            }
        }
    }
}

static void task_preload_position(void)
{
    SteeringMotor *motor = &s_motors[s_motor_index];
    if (s_state == STEERING_STATE_PRELOAD_POSITION_SEND) {
        const rs00_frame_t frame =
            rs00_make_write_float(STEERING_HOST_ID, motor->motor_id,
                                  RS00_PARAM_LOC_REF,
                                  motor->startup_position_rad);
        s_parameter_index = RS00_PARAM_LOC_REF;
        (void)send_and_wait(&frame, s_feedback_sequence[s_motor_index],
                            STEERING_STATE_PRELOAD_POSITION_WAIT,
                            STEERING_STATE_PRELOAD_POSITION_SEND);
    } else if (s_state == STEERING_STATE_PRELOAD_POSITION_WAIT) {
        if (wait_for_feedback(STEERING_STATE_PRELOAD_POSITION_SEND)) {
            set_state(STEERING_STATE_VERIFY_POSITION_SEND);
        }
    } else if (s_state == STEERING_STATE_VERIFY_POSITION_SEND) {
        send_parameter_read(RS00_PARAM_LOC_REF,
                            STEERING_STATE_VERIFY_POSITION_WAIT,
                            STEERING_STATE_VERIFY_POSITION_SEND);
    } else if (verify_float_reply(STEERING_STATE_VERIFY_POSITION_SEND,
                                  motor->startup_position_rad)) {
        motor->initialized = true;
        advance_motor_or_state(STEERING_STATE_PRELOAD_POSITION_SEND,
                               (STEERING_AUTO_ENABLE != 0)
                                   ? STEERING_STATE_ENABLE_SEND
                                   : STEERING_STATE_ARMED);
        if (s_state == STEERING_STATE_ARMED) {
            log_message(STEERING_LOG_INFO,
                        "Initialization complete; motors disabled "
                        "(STEERING_AUTO_ENABLE=0)");
        }
    }
}

static void task_enable(void)
{
    SteeringMotor *motor = &s_motors[s_motor_index];
    if (s_state == STEERING_STATE_ENABLE_SEND) {
        const rs00_frame_t frame =
            rs00_make_enable(STEERING_HOST_ID, motor->motor_id);
        (void)send_and_wait(&frame, s_feedback_sequence[s_motor_index],
                            STEERING_STATE_ENABLE_WAIT,
                            STEERING_STATE_ENABLE_SEND);
    } else if (wait_for_feedback(STEERING_STATE_ENABLE_SEND)) {
        if ((motor->mode_state != RS00_MODE_MOTOR)
            || (fabsf(motor->velocity_rad_s)
                > STEERING_ENABLE_MAX_SPEED_RAD_S)
            || (elapsed_ms(HAL_GetTick(), motor->last_rx_tick)
                > STEERING_RESPONSE_TIMEOUT_MS)) {
            enter_fault(motor->mode_state != RS00_MODE_MOTOR
                            ? STEERING_ERROR_MODE
                            : STEERING_ERROR_FEEDBACK_STALE);
            return;
        }
        motor->enabled = true;
        advance_motor_or_state(STEERING_STATE_ENABLE_SEND,
                               STEERING_STATE_VERIFY_ALL);
    }
}

static void task_verify_all(void)
{
    size_t index;
    const uint32_t now = HAL_GetTick();
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if (!s_motors[index].initialized || !s_motors[index].enabled
            || has_motor_fault(&s_motors[index])
            || (s_motors[index].mode_state != RS00_MODE_MOTOR)
            || (elapsed_ms(now, s_motors[index].last_rx_tick)
                > STEERING_ENABLE_SEQUENCE_TIMEOUT_MS)) {
            s_motor_index = (uint8_t)index;
            enter_fault(has_motor_fault(&s_motors[index])
                            ? STEERING_ERROR_MOTOR_FAULT
                            : STEERING_ERROR_FEEDBACK_STALE);
            return;
        }
    }
    s_ready = true;
    s_last_target_tick = now;
    set_state(STEERING_STATE_READY);
    log_message(STEERING_LOG_INFO, "Steering READY");
}

static void task_ready(void)
{
    size_t index;
    const uint32_t now = HAL_GetTick();
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        uint32_t last_rx_tick;
        uint32_t primask;
        bool motor_fault;

        primask = __get_PRIMASK();
        __disable_irq();
        motor_fault = has_motor_fault(&s_motors[index]);
        last_rx_tick = s_motors[index].last_rx_tick;
        __DMB();
        if (primask == 0U) {
            __enable_irq();
        }

        if (motor_fault) {
            s_motor_index = (uint8_t)index;
            enter_fault(STEERING_ERROR_MOTOR_FAULT);
            return;
        }
        if (elapsed_ms(now, last_rx_tick)
            <= STEERING_FEEDBACK_TIMEOUT_MS) {
            s_feedback_stale_pending[index] = false;
            continue;
        }
        if (!s_feedback_stale_pending[index]
            || (s_feedback_stale_last_rx[index] != last_rx_tick)) {
            s_feedback_stale_pending[index] = true;
            s_feedback_stale_since[index] = now;
            s_feedback_stale_last_rx[index] = last_rx_tick;
            continue;
        }
        if (elapsed_ms(now, s_feedback_stale_since[index])
            < STEERING_FEEDBACK_CONFIRM_MS) {
            continue;
        }

        /* Recheck after the confirmation window before latching a fault. */
        primask = __get_PRIMASK();
        __disable_irq();
        last_rx_tick = s_motors[index].last_rx_tick;
        __DMB();
        if (primask == 0U) {
            __enable_irq();
        }
        if ((last_rx_tick != s_feedback_stale_last_rx[index])
            || (elapsed_ms(HAL_GetTick(), last_rx_tick)
                <= STEERING_FEEDBACK_TIMEOUT_MS)) {
            s_feedback_stale_pending[index] = false;
            continue;
        }
        s_motor_index = (uint8_t)index;
        enter_fault(STEERING_ERROR_FEEDBACK_STALE);
        return;
    }
    if (elapsed_ms(now, s_last_target_tick) < STEERING_TARGET_PERIOD_MS) {
        return;
    }
    s_last_target_tick = now;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        const rs00_frame_t frame =
            rs00_make_write_float(STEERING_HOST_ID, s_motors[index].motor_id,
                                  RS00_PARAM_LOC_REF,
                                  s_motors[index].target_position_rad);
        s_motor_index = (uint8_t)index;
        if (!send_frame(&frame)) {
            enter_fault(s_motors[index].last_error);
            return;
        }
    }
    s_motor_index = 0U;
}

void SteeringController_Task(void)
{
    size_t index;
    const uint32_t now = HAL_GetTick();
    __DMB();
    if ((s_state != STEERING_STATE_FAULT)
        && (s_state != STEERING_STATE_IDLE)) {
        for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
            if (has_motor_fault(&s_motors[index])) {
                s_motor_index = (uint8_t)index;
                enter_fault(STEERING_ERROR_MOTOR_FAULT);
                return;
            }
            if (s_motors[index].online
                && (s_motors[index].temperature_c
                    >= STEERING_MAX_FEEDBACK_TEMPERATURE_C)) {
                s_motor_index = (uint8_t)index;
                enter_fault(STEERING_ERROR_OVERTEMPERATURE);
                return;
            }
        }
    }

    switch (s_state) {
    case STEERING_STATE_BOOT_WAIT:
        if (elapsed_ms(now, s_state_tick) >= STEERING_MOTOR_BOOT_DELAY_MS) {
            s_motor_index = 0U;
            set_state(STEERING_STATE_STOP_SEND);
        }
        break;
    case STEERING_STATE_STOP_SEND:
    case STEERING_STATE_STOP_WAIT:
        task_stop();
        break;
    case STEERING_STATE_QUERY_UID_SEND:
    case STEERING_STATE_QUERY_UID_WAIT:
        task_uid();
        break;
    case STEERING_STATE_READ_POSITION_SEND:
    case STEERING_STATE_READ_POSITION_WAIT:
        task_read_position();
        break;
    case STEERING_STATE_VALIDATE_POSITION:
        task_validate_positions();
        break;
    case STEERING_STATE_SET_MODE_SEND:
    case STEERING_STATE_SET_MODE_WAIT:
    case STEERING_STATE_VERIFY_MODE_SEND:
    case STEERING_STATE_VERIFY_MODE_WAIT:
        task_set_mode();
        break;
    case STEERING_STATE_SET_LIMIT_CURRENT_SEND:
    case STEERING_STATE_SET_LIMIT_CURRENT_WAIT:
    case STEERING_STATE_VERIFY_LIMIT_CURRENT_SEND:
    case STEERING_STATE_VERIFY_LIMIT_CURRENT_WAIT:
        task_float_parameter(RS00_PARAM_LIMIT_CUR,
                             s_csp_limit_current_a,
                             STEERING_STATE_SET_LIMIT_CURRENT_SEND,
                             STEERING_STATE_SET_LIMIT_CURRENT_WAIT,
                             STEERING_STATE_VERIFY_LIMIT_CURRENT_SEND,
                             STEERING_STATE_VERIFY_LIMIT_CURRENT_WAIT,
                             STEERING_STATE_SET_TIMEOUT_SEND);
        break;
    case STEERING_STATE_SET_LIMIT_SPEED_SEND:
    case STEERING_STATE_SET_LIMIT_SPEED_WAIT:
    case STEERING_STATE_VERIFY_LIMIT_SPEED_SEND:
    case STEERING_STATE_VERIFY_LIMIT_SPEED_WAIT:
        task_float_parameter(RS00_PARAM_LIMIT_SPD,
                             s_csp_limit_speed_rad_s,
                             STEERING_STATE_SET_LIMIT_SPEED_SEND,
                             STEERING_STATE_SET_LIMIT_SPEED_WAIT,
                             STEERING_STATE_VERIFY_LIMIT_SPEED_SEND,
                             STEERING_STATE_VERIFY_LIMIT_SPEED_WAIT,
                             STEERING_STATE_SET_LIMIT_CURRENT_SEND);
        break;
    case STEERING_STATE_SET_TIMEOUT_SEND:
    case STEERING_STATE_SET_TIMEOUT_WAIT:
    case STEERING_STATE_VERIFY_TIMEOUT_SEND:
    case STEERING_STATE_VERIFY_TIMEOUT_WAIT:
        task_timeout_parameter();
        break;
    case STEERING_STATE_PRELOAD_POSITION_SEND:
    case STEERING_STATE_PRELOAD_POSITION_WAIT:
    case STEERING_STATE_VERIFY_POSITION_SEND:
    case STEERING_STATE_VERIFY_POSITION_WAIT:
        task_preload_position();
        break;
    case STEERING_STATE_ARMED:
        if (s_enable_requested) {
            s_enable_requested = false;
            s_motor_index = 0U;
            set_state(STEERING_STATE_ENABLE_SEND);
        }
        break;
    case STEERING_STATE_ENABLE_SEND:
    case STEERING_STATE_ENABLE_WAIT:
        task_enable();
        break;
    case STEERING_STATE_VERIFY_ALL:
        task_verify_all();
        break;
    case STEERING_STATE_READY:
        task_ready();
        break;
    case STEERING_STATE_IDLE:
    case STEERING_STATE_FAULT:
    default:
        break;
    }
}

bool SteeringController_RequestEnable(void)
{
    size_t index;
    if (s_state != STEERING_STATE_ARMED) {
        return false;
    }
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if (!s_motors[index].initialized
            || has_motor_fault(&s_motors[index])) {
            return false;
        }
    }
    s_enable_requested = true;
    ++g_chassis_debug.steering_enable_request_count;
    return true;
}

void SteeringController_StopAll(void)
{
    size_t index;
    bool all_initialized = true;
    stop_frames_only(false);
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        all_initialized = all_initialized && s_motors[index].initialized;
    }
    if (s_state != STEERING_STATE_FAULT) {
        set_state(all_initialized ? STEERING_STATE_ARMED
                                  : STEERING_STATE_IDLE);
    }
}

void SteeringController_EmergencyStop(void)
{
    stop_frames_only(false);
    g_steering_debug_error = STEERING_ERROR_MOTOR_FAULT;
    ChassisDebug_SetFault(CHASSIS_FAULT_STEERING);
    set_state(STEERING_STATE_FAULT);
}

bool SteeringController_ClearFaultAndRestart(void)
{
    size_t index;
    stop_frames_only(true);
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].fault = 0U;
        s_motors[index].warning = 0U;
        s_motors[index].retry_count = 0U;
        s_motors[index].last_error = STEERING_ERROR_NONE;
        s_motors[index].initialized = false;
        s_motors[index].enabled = false;
    }
    g_steering_debug_error = STEERING_ERROR_NONE;
    g_steering_debug_fault_motor_id = 0U;
    memset((void *)g_steering_debug_feedback_age_ms, 0,
           sizeof(g_steering_debug_feedback_age_ms));
    memset(s_feedback_stale_pending, 0,
           sizeof(s_feedback_stale_pending));
    memset(s_feedback_stale_since, 0, sizeof(s_feedback_stale_since));
    memset(s_feedback_stale_last_rx, 0,
           sizeof(s_feedback_stale_last_rx));
    s_motor_index = 0U;
    set_state(STEERING_STATE_BOOT_WAIT);
    return true;
}

bool SteeringController_SetCspModeAndRestart(void)
{
    return SteeringController_ClearFaultAndRestart();
}

bool SteeringController_SetCspLimitsAndRestart(float speed_rad_s,
                                               float current_a)
{
    if (!isfinite(speed_rad_s) || !isfinite(current_a)
        || (speed_rad_s <= 0.0f)
        || (speed_rad_s > STEERING_CSP_LIMIT_SPEED_RAD_S)
        || (current_a <= 0.0f)
        || (current_a > STEERING_CSP_LIMIT_CURRENT_A)) {
        return false;
    }
    s_csp_limit_speed_rad_s = speed_rad_s;
    s_csp_limit_current_a = current_a;
    return SteeringController_ClearFaultAndRestart();
}

bool SteeringController_IsReady(void)
{
    return s_ready && (s_state == STEERING_STATE_READY);
}

SteeringState SteeringController_GetState(void)
{
    return s_state;
}

const SteeringMotor *SteeringController_GetMotor(SteeringMotorIndex index)
{
    if ((unsigned)index >= STEERING_MOTOR_COUNT) {
        return NULL;
    }
    return &s_motors[index];
}

bool SteeringController_GetMotorSnapshot(SteeringMotorIndex index,
                                         SteeringMotor *snapshot)
{
    uint32_t primask;
    if (((unsigned)index >= STEERING_MOTOR_COUNT) || (snapshot == NULL)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_motors[index];
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

static bool calculate_target(SteeringMotorIndex index,
                             float chassis_angle_rad,
                             float *motor_angle_rad)
{
    float command =
        (s_direction[index] * chassis_angle_rad) + s_zero_offset[index];
    if (!isfinite(chassis_angle_rad) || !isfinite(command)) {
        return false;
    }
    if ((STEERING_ENABLE_MECHANICAL_LIMIT_CHECK != 0)
        && ((command < (s_min_position[index] - STEERING_LIMIT_EPSILON_RAD))
            || (command > (s_max_position[index]
                           + STEERING_LIMIT_EPSILON_RAD)))) {
        return false;
    }
    if (command < s_min_position[index]) {
        command = s_min_position[index];
    } else if (command > s_max_position[index]) {
        command = s_max_position[index];
    }
    *motor_angle_rad = command;
    return true;
}

bool SteeringController_SetMotorAngle(SteeringMotorIndex index,
                                      float chassis_angle_rad)
{
    float target;
    if (!SteeringController_IsReady()
        || ((unsigned)index >= STEERING_MOTOR_COUNT)
        || has_motor_fault(&s_motors[index])
        || !calculate_target(index, chassis_angle_rad, &target)) {
        if (((unsigned)index < STEERING_MOTOR_COUNT)
            && SteeringController_IsReady()) {
            s_motor_index = (uint8_t)index;
            enter_fault(STEERING_ERROR_TARGET_RANGE);
        }
        return false;
    }
    if ((STEERING_CALIBRATION_CONFIRMED == 0)
        && (fabsf(target - s_motors[index].target_position_rad)
            > (STEERING_COMMISSIONING_MAX_STEP_RAD
               + STEERING_FLOAT_TOLERANCE))) {
        s_motor_index = (uint8_t)index;
        enter_fault(STEERING_ERROR_TARGET_STEP);
        return false;
    }
    s_motors[index].target_position_rad = target;
    return true;
}

bool SteeringController_SetAllAngles(float fl_rad, float fr_rad,
                                     float rl_rad, float rr_rad)
{
    const float chassis[STEERING_MOTOR_COUNT] =
        {fl_rad, fr_rad, rl_rad, rr_rad};
    float targets[STEERING_MOTOR_COUNT];
    size_t index;
    if (!SteeringController_IsReady()) {
        return false;
    }
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if (has_motor_fault(&s_motors[index])
            || !calculate_target((SteeringMotorIndex)index,
                                 chassis[index], &targets[index])) {
            s_motor_index = (uint8_t)index;
            enter_fault(STEERING_ERROR_TARGET_RANGE);
            return false;
        }
        if ((STEERING_CALIBRATION_CONFIRMED == 0)
            && (fabsf(targets[index]
                      - s_motors[index].target_position_rad)
                > (STEERING_COMMISSIONING_MAX_STEP_RAD
                   + STEERING_FLOAT_TOLERANCE))) {
            s_motor_index = (uint8_t)index;
            enter_fault(STEERING_ERROR_TARGET_STEP);
            return false;
        }
    }
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].target_position_rad = targets[index];
    }
    return true;
}

bool SteeringController_IsHealthy(void)
{
    size_t index;
    if ((s_state == STEERING_STATE_FAULT)
        || (s_state == STEERING_STATE_IDLE)) {
        return false;
    }
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if (has_motor_fault(&s_motors[index])) {
            return false;
        }
    }
    return true;
}

bool SteeringController_IsCalibrationConfirmed(void)
{
    return STEERING_CALIBRATION_CONFIRMED != 0;
}
