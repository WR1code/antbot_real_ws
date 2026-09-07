#include "drive_controller.h"

#include "drive_config.h"
#include "mini_drive_protocol.h"
#include "rs00_stm32_fdcan.h"
#include "stm32h7xx_hal.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI_F 6.28318530717958647692f
#define ALL_WHEELS_MASK ((uint8_t)((1U << DRIVE_WHEEL_COUNT) - 1U))
#define REQUIRED_FEEDBACK_MASK \
    ((1UL << MINI_DRIVE_FEEDBACK_FAULT) \
     | (1UL << MINI_DRIVE_FEEDBACK_SPEED) \
     | (1UL << MINI_DRIVE_FEEDBACK_VOLTAGE) \
     | (1UL << MINI_DRIVE_FEEDBACK_MOTOR_CURRENT) \
     | (1UL << MINI_DRIVE_FEEDBACK_TEMPERATURE))

_Static_assert(DRIVE_HEARTBEAT_PERIOD_MS < DRIVE_HEARTBEAT_TIMEOUT_MS,
               "MINI heartbeat period must be below its configured timeout");

typedef enum {
    CONFIG_HEARTBEAT = 0,
    CONFIG_ACCELERATION,
    CONFIG_DECELERATION,
    CONFIG_ZERO_SPEED,
    CONFIG_DONE
} DriveConfigPhase;

static const uint16_t s_can_ids[DRIVE_WHEEL_COUNT] = {
    DRIVE_CAN_ID_FL, DRIVE_CAN_ID_FR, DRIVE_CAN_ID_RL, DRIVE_CAN_ID_RR
};

static float s_target_mps[DRIVE_WHEEL_COUNT];
static uint32_t s_last_heartbeat_ms[DRIVE_WHEEL_COUNT];
static uint32_t s_last_speed_ms[DRIVE_WHEEL_COUNT];
static bool s_initialized;
static bool s_emergency_stopped;
static DriveConfigPhase s_config_phase;
static uint8_t s_config_wheel;
static uint8_t s_speed_dirty;
static uint8_t s_emergency_pending;
static uint8_t s_round_robin_wheel;
static uint32_t s_next_tx_ms;
static uint32_t s_stop_deadline_ms;
static uint32_t s_stop_feedback_since_ms;
static uint32_t s_tx_error_count;
static uint32_t s_consecutive_tx_errors;
static int32_t s_acceleration_erpm_s;
static int32_t s_deceleration_erpm_s;
static bool s_runtime_config_initialized;
static uint32_t s_next_query_ms;
static uint8_t s_query_wheel;
static uint8_t s_query_type_index;
static bool s_feedback_ever_ready;
static uint32_t s_safety_flags[DRIVE_WHEEL_COUNT];
volatile DriveMotorFeedback g_drive_feedback[DRIVE_WHEEL_COUNT];

static const MiniDriveFeedbackType s_query_types[] = {
    MINI_DRIVE_FEEDBACK_FAULT,
    MINI_DRIVE_FEEDBACK_SPEED,
    MINI_DRIVE_FEEDBACK_MOTOR_CURRENT,
    MINI_DRIVE_FEEDBACK_TEMPERATURE,
    MINI_DRIVE_FEEDBACK_VOLTAGE
};

static bool tick_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int32_t linear_to_erpm(float linear_value)
{
    const float scale = (60.0f * DRIVE_MOTOR_TO_WHEEL_RATIO
                         * DRIVE_MOTOR_POLE_PAIRS)
                        / (TWO_PI_F * DRIVE_WHEEL_RADIUS_M);
    const float converted = linear_value * scale;
    if (converted >= (float)DRIVE_MOTOR_MAX_ERPM) {
        return DRIVE_MOTOR_MAX_ERPM;
    }
    if (converted <= (float)-DRIVE_MOTOR_MAX_ERPM) {
        return -DRIVE_MOTOR_MAX_ERPM;
    }
    if (converted >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (converted <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(converted);
}

static float erpm_to_linear(int32_t erpm)
{
    const float scale = (60.0f * DRIVE_MOTOR_TO_WHEEL_RATIO
                         * DRIVE_MOTOR_POLE_PAIRS)
                        / (TWO_PI_F * DRIVE_WHEEL_RADIUS_M);
    return scale > 0.0f ? (float)erpm / scale : 0.0f;
}

static uint32_t feedback_type_tick(const DriveMotorFeedback *feedback,
                                   MiniDriveFeedbackType type)
{
    switch (type) {
    case MINI_DRIVE_FEEDBACK_FAULT: return feedback->last_fault_ms;
    case MINI_DRIVE_FEEDBACK_SPEED: return feedback->last_speed_ms;
    case MINI_DRIVE_FEEDBACK_VOLTAGE: return feedback->last_voltage_ms;
    case MINI_DRIVE_FEEDBACK_MOTOR_CURRENT: return feedback->last_current_ms;
    case MINI_DRIVE_FEEDBACK_TEMPERATURE:
        return feedback->last_temperature_ms;
    default: return 0U;
    }
}

static uint32_t calculate_safety_flags(unsigned wheel, uint32_t now)
{
    DriveMotorFeedback feedback;
    uint32_t flags = 0U;
    unsigned type_index;
    if (!DriveController_GetFeedback((DriveWheelIndex)wheel, &feedback)) {
        return DRIVE_SAFETY_MISSING;
    }
    if ((feedback.valid_mask & REQUIRED_FEEDBACK_MASK)
        != REQUIRED_FEEDBACK_MASK) {
        flags |= DRIVE_SAFETY_MISSING;
    }
    for (type_index = 0U;
         type_index < sizeof(s_query_types) / sizeof(s_query_types[0]);
         ++type_index) {
        const MiniDriveFeedbackType type = s_query_types[type_index];
        if (((feedback.valid_mask & (1UL << (unsigned)type)) != 0U)
            && ((now - feedback_type_tick(&feedback, type))
                > DRIVE_SAFETY_FEEDBACK_TIMEOUT_MS)) {
            flags |= DRIVE_SAFETY_STALE;
        }
    }
    if (feedback.fault_bits != 0U) {
        flags |= DRIVE_SAFETY_DRIVER_FAULT;
    }
    if (((feedback.valid_mask
          & (1UL << MINI_DRIVE_FEEDBACK_MOTOR_CURRENT)) != 0U)
        && (abs(feedback.motor_current_10ma)
            > DRIVE_MAX_FEEDBACK_CURRENT_10MA)) {
        flags |= DRIVE_SAFETY_OVERCURRENT;
    }
    if (((feedback.valid_mask
          & (1UL << MINI_DRIVE_FEEDBACK_TEMPERATURE)) != 0U)
        && (feedback.temperature_c
            >= DRIVE_MAX_FEEDBACK_TEMPERATURE_C)) {
        flags |= DRIVE_SAFETY_OVERTEMPERATURE;
    }
    if (((feedback.valid_mask
          & (1UL << MINI_DRIVE_FEEDBACK_VOLTAGE)) != 0U)
        && ((feedback.voltage_v < DRIVE_MIN_FEEDBACK_VOLTAGE_V)
            || (feedback.voltage_v > DRIVE_MAX_FEEDBACK_VOLTAGE_V))) {
        flags |= DRIVE_SAFETY_VOLTAGE;
    }
    if (((feedback.valid_mask
          & (1UL << MINI_DRIVE_FEEDBACK_SPEED)) != 0U)
        && (labs(feedback.speed_erpm) > DRIVE_MAX_FEEDBACK_SPEED_ERPM)) {
        flags |= DRIVE_SAFETY_OVERSPEED;
    }
    return flags;
}

static void update_safety(uint32_t now)
{
    unsigned wheel;
    bool ready = true;
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        s_safety_flags[wheel] = calculate_safety_flags(wheel, now);
        ready = ready && (s_safety_flags[wheel] == 0U);
    }
    s_feedback_ever_ready = s_feedback_ever_ready || ready;
}

static bool send_frame(const MiniDriveFrame *frame)
{
    const bool sent = (frame != NULL)
                      && rs00_fdcan_send_standard(
                             frame->id, frame->data, frame->dlc);
    if (sent) {
        s_consecutive_tx_errors = 0U;
    } else {
        ++s_tx_error_count;
        ++s_consecutive_tx_errors;
    }
    return sent;
}

static bool make_config_frame(MiniDriveFrame *frame)
{
    const uint16_t id = s_can_ids[s_config_wheel];
    switch (s_config_phase) {
    case CONFIG_HEARTBEAT:
        return MiniDrive_MakeHeartbeat(id, frame);
    case CONFIG_ACCELERATION:
        return MiniDrive_MakeAcceleration(
            id, s_acceleration_erpm_s, frame);
    case CONFIG_DECELERATION:
        return MiniDrive_MakeDeceleration(
            id, s_deceleration_erpm_s, frame);
    case CONFIG_ZERO_SPEED:
        return MiniDrive_MakeSpeed(id, 0, frame);
    default:
        return false;
    }
}

static void advance_config(uint32_t now)
{
    if (s_config_phase == CONFIG_HEARTBEAT) {
        s_last_heartbeat_ms[s_config_wheel] = now;
    } else if (s_config_phase == CONFIG_ZERO_SPEED) {
        s_last_speed_ms[s_config_wheel] = now;
    }
    ++s_config_wheel;
    if (s_config_wheel >= DRIVE_WHEEL_COUNT) {
        s_config_wheel = 0U;
        s_config_phase = (DriveConfigPhase)((unsigned)s_config_phase + 1U);
    }
}

bool DriveController_Init(void)
{
    unsigned index;
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        s_target_mps[index] = 0.0f;
        s_last_heartbeat_ms[index] = 0U;
        s_last_speed_ms[index] = 0U;
    }
    memset((void *)g_drive_feedback, 0, sizeof(g_drive_feedback));
    if (!s_runtime_config_initialized) {
        s_acceleration_erpm_s = linear_to_erpm(DRIVE_ACCELERATION_MPS2);
        s_deceleration_erpm_s = linear_to_erpm(DRIVE_DECELERATION_MPS2);
        s_runtime_config_initialized = true;
    }
    s_initialized = true;
    s_emergency_stopped = false;
    s_config_phase = CONFIG_HEARTBEAT;
    s_config_wheel = 0U;
    s_speed_dirty = ALL_WHEELS_MASK;
    s_emergency_pending = 0U;
    s_round_robin_wheel = 0U;
    s_next_tx_ms = 0U;
    s_stop_deadline_ms = 0U;
    s_stop_feedback_since_ms = 0U;
    s_tx_error_count = 0U;
    s_consecutive_tx_errors = 0U;
    s_next_query_ms = 0U;
    s_query_wheel = 0U;
    s_query_type_index = 0U;
    s_feedback_ever_ready = false;
    memset(s_safety_flags, 0, sizeof(s_safety_flags));
    return true;
}

void DriveController_Task(void)
{
    MiniDriveFrame frame;
    const uint32_t now = HAL_GetTick();
    unsigned offset;

    update_safety(now);

    if (!s_initialized || !tick_reached(now, s_next_tx_ms)) {
        return;
    }
    s_next_tx_ms = now + DRIVE_TX_INTERVAL_MS;

    if (s_emergency_stopped) {
        for (offset = 0U; offset < DRIVE_WHEEL_COUNT; ++offset) {
            const unsigned wheel =
                (s_round_robin_wheel + offset) % DRIVE_WHEEL_COUNT;
            const uint8_t mask = (uint8_t)(1U << wheel);
            if ((s_emergency_pending & mask) != 0U
                && MiniDrive_MakeCurrent(s_can_ids[wheel], 0, &frame)
                && send_frame(&frame)) {
                s_emergency_pending &= (uint8_t)~mask;
                s_round_robin_wheel =
                    (uint8_t)((wheel + 1U) % DRIVE_WHEEL_COUNT);
            }
            return;
        }
        return;
    }

    if (s_config_phase != CONFIG_DONE) {
        if (make_config_frame(&frame) && send_frame(&frame)) {
            advance_config(now);
        }
        return;
    }

    for (offset = 0U; offset < DRIVE_WHEEL_COUNT; ++offset) {
        const unsigned wheel =
            (s_round_robin_wheel + offset) % DRIVE_WHEEL_COUNT;
        if ((now - s_last_heartbeat_ms[wheel])
            >= DRIVE_HEARTBEAT_PERIOD_MS) {
            if (MiniDrive_MakeHeartbeat(s_can_ids[wheel], &frame)
                && send_frame(&frame)) {
                s_last_heartbeat_ms[wheel] = now;
                s_round_robin_wheel =
                    (uint8_t)((wheel + 1U) % DRIVE_WHEEL_COUNT);
            }
            return;
        }
    }

    for (offset = 0U; offset < DRIVE_WHEEL_COUNT; ++offset) {
        const unsigned wheel =
            (s_round_robin_wheel + offset) % DRIVE_WHEEL_COUNT;
        const uint8_t mask = (uint8_t)(1U << wheel);
        if (((s_speed_dirty & mask) != 0U)
            || ((now - s_last_speed_ms[wheel])
                >= DRIVE_SPEED_REFRESH_PERIOD_MS)) {
            if (MiniDrive_MakeSpeed(
                    s_can_ids[wheel],
                    linear_to_erpm(s_target_mps[wheel]), &frame)
                && send_frame(&frame)) {
                s_speed_dirty &= (uint8_t)~mask;
                s_last_speed_ms[wheel] = now;
                s_round_robin_wheel =
                    (uint8_t)((wheel + 1U) % DRIVE_WHEEL_COUNT);
            }
            return;
        }
    }

    if (tick_reached(now, s_next_query_ms)) {
        const MiniDriveFeedbackType type = s_query_types[s_query_type_index];
        if (MiniDrive_MakeQuery(s_can_ids[s_query_wheel], type, &frame)
            && send_frame(&frame)) {
            ++s_query_wheel;
            if (s_query_wheel >= DRIVE_WHEEL_COUNT) {
                s_query_wheel = 0U;
                s_query_type_index = (uint8_t)((s_query_type_index + 1U)
                    % (sizeof(s_query_types) / sizeof(s_query_types[0])));
            }
            s_next_query_ms = now + DRIVE_SAFETY_QUERY_INTERVAL_MS;
        }
        return;
    }
}

bool DriveController_SetWheelSpeed(DriveWheelIndex wheel, float speed_mps)
{
    if (!s_initialized || s_emergency_stopped
        || ((unsigned)wheel >= DRIVE_WHEEL_COUNT) || !isfinite(speed_mps)
        || (fabsf(speed_mps) > DRIVE_MAX_ABS_SPEED_MPS)
        || ((speed_mps != 0.0f) && !DriveController_IsSafetyFeedbackReady())) {
        return false;
    }
    s_target_mps[wheel] = (speed_mps == 0.0f) ? 0.0f : speed_mps;
    s_speed_dirty |= (uint8_t)(1U << (unsigned)wheel);
    return true;
}

bool DriveController_SetAllWheelSpeeds(float fl_mps, float fr_mps,
                                       float rl_mps, float rr_mps)
{
    const float speeds[DRIVE_WHEEL_COUNT] = {fl_mps, fr_mps, rl_mps, rr_mps};
    unsigned index;
    if (!s_initialized || s_emergency_stopped) {
        return false;
    }
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        if (!isfinite(speeds[index])
            || (fabsf(speeds[index]) > DRIVE_MAX_ABS_SPEED_MPS)) {
            return false;
        }
    }
    if (!DriveController_IsSafetyFeedbackReady()) {
        for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
            if (speeds[index] != 0.0f) {
                return false;
            }
        }
    }
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        s_target_mps[index] =
            (speeds[index] == 0.0f) ? 0.0f : speeds[index];
    }
    s_speed_dirty = ALL_WHEELS_MASK;
    return true;
}

bool DriveController_SetAccelerationErpmS(int32_t acceleration_erpm_s)
{
    if (!s_initialized || (acceleration_erpm_s <= 0)
        || (acceleration_erpm_s > DRIVE_SAFE_MAX_ACCELERATION_ERPM_S)) {
        return false;
    }
    DriveController_StopAll();
    s_acceleration_erpm_s = acceleration_erpm_s;
    s_config_phase = CONFIG_ACCELERATION;
    s_config_wheel = 0U;
    s_next_tx_ms = 0U;
    return true;
}

bool DriveController_SetDecelerationErpmS(int32_t deceleration_erpm_s)
{
    if (!s_initialized || (deceleration_erpm_s <= 0)
        || (deceleration_erpm_s > DRIVE_SAFE_MAX_DECELERATION_ERPM_S)) {
        return false;
    }
    DriveController_StopAll();
    s_deceleration_erpm_s = deceleration_erpm_s;
    s_config_phase = CONFIG_DECELERATION;
    s_config_wheel = 0U;
    s_next_tx_ms = 0U;
    return true;
}

void DriveController_StopAll(void)
{
    unsigned index;
    float highest_speed = 0.0f;
    const uint32_t now = HAL_GetTick();
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        const float speed = fabsf(s_target_mps[index]);
        if (speed > highest_speed) {
            highest_speed = speed;
        }
        s_target_mps[index] = 0.0f;
    }
    if (highest_speed > 0.0f) {
        const float deceleration_mps2 =
            fabsf(erpm_to_linear(s_deceleration_erpm_s));
        const float braking_ms =
            (highest_speed / deceleration_mps2) * 1000.0f;
        s_stop_deadline_ms =
            now + (uint32_t)ceilf(braking_ms) + DRIVE_STOP_SETTLE_MARGIN_MS;
        s_stop_feedback_since_ms = 0U;
    }
    s_speed_dirty = ALL_WHEELS_MASK;
}

void DriveController_EmergencyStop(void)
{
    DriveController_StopAll();
    s_emergency_stopped = true;
    s_emergency_pending = ALL_WHEELS_MASK;
    s_next_tx_ms = 0U;
}

bool DriveController_ClearEmergencyStopAndRestart(void)
{
    return DriveController_Init();
}

bool DriveController_AllWheelsStopped(void)
{
    unsigned index;
    const uint32_t now = HAL_GetTick();
    bool fresh_speed = true;
    bool below_threshold = true;
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        if (fabsf(s_target_mps[index]) > 0.0001f) {
            return false;
        }
    }
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        DriveMotorFeedback feedback;
        if (!DriveController_GetFeedback((DriveWheelIndex)index, &feedback)
            || ((feedback.valid_mask
                 & (1UL << MINI_DRIVE_FEEDBACK_SPEED)) == 0U)
            || ((now - feedback.last_speed_ms)
                > DRIVE_SAFETY_FEEDBACK_TIMEOUT_MS)) {
            fresh_speed = false;
            break;
        }
        below_threshold = below_threshold
            && (labs(feedback.speed_erpm)
                <= DRIVE_STOP_SPEED_THRESHOLD_ERPM);
    }
    if (fresh_speed) {
        if (!below_threshold) {
            s_stop_feedback_since_ms = 0U;
            return false;
        }
        if (s_stop_feedback_since_ms == 0U) {
            s_stop_feedback_since_ms = now;
        }
        return (now - s_stop_feedback_since_ms)
            >= DRIVE_STOP_FEEDBACK_STABLE_MS;
    }
    return !s_feedback_ever_ready && tick_reached(now, s_stop_deadline_ms);
}

bool DriveController_IsHealthy(void)
{
    unsigned wheel;
    if (!s_initialized || s_emergency_stopped
        || (s_consecutive_tx_errors >= DRIVE_MAX_CONSECUTIVE_TX_ERRORS)) {
        return false;
    }
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        const uint32_t immediate = DRIVE_SAFETY_DRIVER_FAULT
            | DRIVE_SAFETY_OVERCURRENT | DRIVE_SAFETY_OVERTEMPERATURE
            | DRIVE_SAFETY_VOLTAGE | DRIVE_SAFETY_OVERSPEED;
        if ((s_safety_flags[wheel] & immediate) != 0U
            || (s_feedback_ever_ready && (s_safety_flags[wheel] != 0U))) {
            return false;
        }
    }
    return true;
}

bool DriveController_IsSafetyFeedbackReady(void)
{
    unsigned wheel;
    if (!s_initialized || s_emergency_stopped) {
        return false;
    }
    update_safety(HAL_GetTick());
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        if (s_safety_flags[wheel] != 0U) {
            return false;
        }
    }
    return true;
}

uint32_t DriveController_GetSafetyFlags(DriveWheelIndex wheel)
{
    if ((unsigned)wheel >= DRIVE_WHEEL_COUNT) {
        return DRIVE_SAFETY_MISSING;
    }
    update_safety(HAL_GetTick());
    return s_safety_flags[(unsigned)wheel];
}

uint32_t DriveController_GetFeedbackAgeMs(DriveWheelIndex wheel)
{
    DriveMotorFeedback feedback;
    if (!DriveController_GetFeedback(wheel, &feedback)
        || (feedback.valid_mask == 0U)) {
        return UINT32_MAX;
    }
    return HAL_GetTick() - feedback.last_feedback_ms;
}

bool DriveController_GetTarget(DriveWheelIndex wheel, float *speed_mps)
{
    if (((unsigned)wheel >= DRIVE_WHEEL_COUNT) || (speed_mps == NULL)) {
        return false;
    }
    *speed_mps = s_target_mps[wheel];
    return true;
}

bool DriveController_IsConfigured(void)
{
    return s_config_phase == CONFIG_DONE;
}

uint32_t DriveController_GetTxErrorCount(void)
{
    return s_tx_error_count;
}

void DriveController_OnCanFrame(uint16_t standard_id, const uint8_t *data,
                                uint8_t length)
{
    MiniDriveFrame frame;
    MiniDriveFeedback decoded;
    unsigned wheel;

    if (!s_initialized || (data == NULL) || (length > sizeof(frame.data))) {
        return;
    }
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        if (s_can_ids[wheel] == standard_id) {
            break;
        }
    }
    if (wheel >= DRIVE_WHEEL_COUNT) {
        return;
    }
    memset(&frame, 0, sizeof(frame));
    frame.id = standard_id;
    frame.dlc = length;
    memcpy(frame.data, data, length);
    if (!MiniDrive_DecodeFeedback(&frame, &decoded)) {
        return;
    }

    {
        const uint32_t now = HAL_GetTick();
        g_drive_feedback[wheel].last_feedback_ms = now;
        switch (decoded.type) {
        case MINI_DRIVE_FEEDBACK_FAULT:
            g_drive_feedback[wheel].last_fault_ms = now; break;
        case MINI_DRIVE_FEEDBACK_SPEED:
            g_drive_feedback[wheel].last_speed_ms = now; break;
        case MINI_DRIVE_FEEDBACK_VOLTAGE:
            g_drive_feedback[wheel].last_voltage_ms = now; break;
        case MINI_DRIVE_FEEDBACK_MOTOR_CURRENT:
            g_drive_feedback[wheel].last_current_ms = now; break;
        case MINI_DRIVE_FEEDBACK_TEMPERATURE:
            g_drive_feedback[wheel].last_temperature_ms = now; break;
        default: break;
        }
    }
    g_drive_feedback[wheel].valid_mask |=
        (uint32_t)(1UL << (unsigned)decoded.type);
    switch (decoded.type) {
    case MINI_DRIVE_FEEDBACK_FAULT:
        g_drive_feedback[wheel].fault_bits = (uint16_t)decoded.value;
        break;
    case MINI_DRIVE_FEEDBACK_SPEED:
        g_drive_feedback[wheel].speed_erpm = decoded.value;
        break;
    case MINI_DRIVE_FEEDBACK_VOLTAGE:
        g_drive_feedback[wheel].voltage_v = (uint16_t)decoded.value;
        break;
    case MINI_DRIVE_FEEDBACK_MOTOR_CURRENT:
        g_drive_feedback[wheel].motor_current_10ma = (int16_t)decoded.value;
        break;
    case MINI_DRIVE_FEEDBACK_TEMPERATURE:
        g_drive_feedback[wheel].temperature_c = (int16_t)decoded.value;
        break;
    case MINI_DRIVE_FEEDBACK_POSITION:
        g_drive_feedback[wheel].position_centideg = decoded.value;
        break;
    default:
        break;
    }
}

bool DriveController_GetFeedback(DriveWheelIndex wheel,
                                 DriveMotorFeedback *feedback)
{
    uint32_t primask;
    if (((unsigned)wheel >= DRIVE_WHEEL_COUNT) || (feedback == NULL)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = g_drive_feedback[(unsigned)wheel];
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}
