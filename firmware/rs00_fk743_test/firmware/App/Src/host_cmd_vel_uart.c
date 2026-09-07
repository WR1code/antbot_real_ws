#include "host_cmd_vel_uart.h"

#include "chassis_debug.h"
#include "chassis_uart_ack.h"
#include "chassis_translation_controller.h"
#include "drive_controller.h"
#include "drive_config.h"
#include "host_cmd_vel_protocol.h"
#include "rs00_stm32_fdcan.h"
#include "steering_controller.h"
#include "system_watchdog.h"
#include "system_health.h"

#include <limits.h>
#include <string.h>

#define HOST_UART_RING_SIZE 64U
#define HOST_UART_ACK_QUEUE_DEPTH 4U
#define HOST_SYSTEM_RESET_ACK_TIMEOUT_MS 100U

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static volatile uint8_t s_ring[HOST_UART_RING_SIZE];
static volatile uint8_t s_write_index;
static volatile uint8_t s_read_index;
static HostCmdVelParser s_parser;
static HostCmdVelUartDebug s_debug;
static uint8_t s_ack_queue[HOST_UART_ACK_QUEUE_DEPTH][CHASSIS_ACK_FRAME_SIZE];
static volatile uint8_t s_ack_write;
static volatile uint8_t s_ack_read;
static volatile bool s_ack_tx_busy;
static bool s_have_sequence;
static uint8_t s_expected_sequence;
static uint32_t s_seen_timeout_count;
static uint32_t s_seen_can_error_count;
static bool s_seen_bus_off;
static uint8_t s_response_control_id;
static uint8_t s_response_detail_type;
static int32_t s_response_values[4];
static uint16_t s_response_valid_mask;
static volatile bool s_reset_requested;
static volatile bool s_reset_ack_sent;
static volatile uint8_t s_reset_sequence;
static uint32_t s_reset_request_tick;

static const uint8_t s_reset_key[4] = {'R', 'S', 'T', '!'};

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int32_t read_i32_le(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8U)
                     | ((uint32_t)data[2] << 16U)
                     | ((uint32_t)data[3] << 24U));
}

static void reset_response_detail(void)
{
    s_response_control_id = 0U;
    s_response_detail_type = ACK_DETAIL_NONE;
    memset(s_response_values, 0, sizeof(s_response_values));
    s_response_valid_mask = 0U;
}

static void arm_receive(void)
{
    if (s_uart != NULL) {
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}

bool HostCmdVelUart_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL) {
        return false;
    }
    s_uart = uart;
    s_write_index = 0U;
    s_read_index = 0U;
    memset(&s_debug, 0, sizeof(s_debug));
    memset(s_ack_queue, 0, sizeof(s_ack_queue));
    s_ack_write = 0U;
    s_ack_read = 0U;
    s_ack_tx_busy = false;
    s_have_sequence = false;
    s_expected_sequence = 0U;
    s_seen_timeout_count = g_chassis_debug.uart_timeout_count;
    s_seen_can_error_count = g_chassis_debug.can_tx_submit_error_count;
    s_seen_bus_off = false;
    s_reset_requested = false;
    s_reset_ack_sent = false;
    s_reset_sequence = 0U;
    s_reset_request_tick = 0U;
    reset_response_detail();
    HostCmdVelParser_Init(&s_parser);
    return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) == HAL_OK;
}

void HostCmdVelUart_RxCompleteCallback(UART_HandleTypeDef *uart)
{
    uint8_t next;

    if ((s_uart == NULL) || (uart != s_uart)) {
        return;
    }
    ChassisDebug_RecordUartRx(1U, HAL_GetTick());
    next = (uint8_t)((s_write_index + 1U) % HOST_UART_RING_SIZE);
    if (next == s_read_index) {
        ++s_debug.uart_overruns;
        ++g_chassis_debug.uart_overflow_count;
    } else {
        s_ring[s_write_index] = s_rx_byte;
        __DMB();
        s_write_index = next;
    }
    arm_receive();
}

void HostCmdVelUart_TxCompleteCallback(UART_HandleTypeDef *uart)
{
    const uint8_t *completed_ack;

    if ((s_uart == NULL) || (uart != s_uart) || !s_ack_tx_busy) {
        return;
    }
    completed_ack = s_ack_queue[s_ack_read];
    if (s_reset_requested
        && (completed_ack[4] == s_reset_sequence)
        && (completed_ack[30] == (uint8_t)HOST_CONTROL_SYSTEM_RESET)) {
        s_reset_ack_sent = true;
    }
    s_ack_read = (uint8_t)((s_ack_read + 1U)
                           % HOST_UART_ACK_QUEUE_DEPTH);
    s_ack_tx_busy = false;
    ++g_chassis_debug.uart_ack_tx_count;
}

void HostCmdVelUart_ErrorCallback(UART_HandleTypeDef *uart)
{
    if ((s_uart == NULL) || (uart != s_uart)) {
        return;
    }
    ++s_debug.uart_errors;
    ChassisDebug_SetFault(CHASSIS_FAULT_UART);
    arm_receive();
}

static void queue_ack(uint8_t sequence, ChassisAckStatus status)
{
#if CHASSIS_UART_ACK_ENABLE
    ChassisAck ack;
    unsigned index;
    const uint8_t next =
        (uint8_t)((s_ack_write + 1U) % HOST_UART_ACK_QUEUE_DEPTH);
    if (next == s_ack_read) {
        ++g_chassis_debug.uart_ack_drop_count;
        reset_response_detail();
        return;
    }
    ack.sequence = sequence;
    ack.status = status;
    ack.chassis_state = (uint8_t)g_chassis_debug.chassis_state;
    ack.reject_reason = (uint8_t)g_chassis_debug.chassis_reject_reason;
    ack.fault_flags = (uint16_t)g_chassis_debug.chassis_fault_flags;
    ack.steering_flags =
        (uint8_t)((g_chassis_debug.steering_enabled != 0U ? 1U : 0U)
                  | (g_chassis_debug.steering_homed != 0U ? 2U : 0U)
                  | (g_chassis_debug.steering_ready != 0U ? 4U : 0U)
                  | (g_chassis_debug.steering_fault != 0U ? 8U : 0U));
    ack.can_flags =
        (uint8_t)((g_chassis_debug.last_can_tx_success != 0U ? 1U : 0U)
                  | ((g_chassis_debug.last_can_rx_valid != 0U
                      && (HAL_GetTick()
                          - g_chassis_debug.last_can_rx_tick) <= 500U)
                         ? 2U : 0U)
                  | (g_chassis_debug.fdcan_bus_off != 0U ? 4U : 0U)
                  | (g_chassis_debug.fdcan_error_passive != 0U ? 8U : 0U));
    ack.uart_valid_count =
        (uint16_t)g_chassis_debug.uart_valid_frame_count;
    ack.can_tx_count =
        (uint16_t)g_chassis_debug.can_tx_submit_ok_count;
    ack.can_rx_count = (uint16_t)g_chassis_debug.can_rx_frame_count;
    ack.stm32_tick = HAL_GetTick();
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        SteeringMotor motor;
        ack.steering_position_mrad[index] = CHASSIS_ACK_POSITION_INVALID;
        if (SteeringController_GetMotorSnapshot(
                (SteeringMotorIndex)index, &motor)
            && motor.initialized) {
            float value = motor.position_rad * 1000.0f;
            if (value > (float)INT16_MAX) {
                value = (float)INT16_MAX;
            } else if (value < (float)INT16_MIN) {
                value = (float)INT16_MIN;
            }
            ack.steering_position_mrad[index] = (int16_t)value;
        }
    }
    ack.control_id = s_response_control_id;
    ack.detail_type = s_response_detail_type;
    memcpy(ack.detail_values, s_response_values, sizeof(ack.detail_values));
    ack.detail_valid_mask = s_response_valid_mask;
    (void)ChassisAck_Encode(&ack, s_ack_queue[s_ack_write]);
    reset_response_detail();
    __DMB();
    s_ack_write = next;
#else
    (void)sequence;
    (void)status;
#endif
}

static bool payload_length_is(const HostCmdVel *command, uint8_t expected)
{
    return command->payload_length == expected;
}

static ChassisAckStatus handle_control_command(const HostCmdVel *command)
{
    const SteeringState steering = SteeringController_GetState();
    s_response_control_id = (uint8_t)command->control_id;

    switch (command->control_id) {
    case HOST_CONTROL_QUERY_STATUS:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            unsigned wheel;
            uint32_t packed_motor_flags = 0U;
            uint32_t packed_fault_context;
            ChassisTranslationDebugState translation;
            s_response_detail_type = ACK_DETAIL_STEERING_STATUS;
            s_response_values[0] = (int32_t)g_steering_debug_state;
            s_response_values[1] = (int32_t)g_steering_debug_error;
            packed_fault_context =
                (uint32_t)g_steering_debug_fault_motor_id;
            if (ChassisTranslation_GetDebugSnapshot(&translation)) {
                packed_fault_context |=
                    ((uint32_t)translation.state & 0xFFU) << 8U;
                packed_fault_context |=
                    ((uint32_t)translation.last_error & 0xFFU) << 16U;
                packed_fault_context |=
                    ((uint32_t)translation.fault_wheel & 0xFFU) << 24U;
            }
            s_response_values[2] = (int32_t)packed_fault_context;
            for (wheel = 0U; wheel < STEERING_MOTOR_COUNT; ++wheel) {
                SteeringMotor motor;
                if (SteeringController_GetMotorSnapshot(
                        (SteeringMotorIndex)wheel, &motor)) {
                    const uint8_t flags =
                        (uint8_t)((motor.mode_state & 0x03U)
                                  | (motor.initialized ? 0x04U : 0U)
                                  | (motor.enabled ? 0x08U : 0U)
                                  | (motor.online ? 0x10U : 0U)
                                  | (motor.fault != 0U ? 0x20U : 0U));
                    packed_motor_flags |= (uint32_t)flags << (wheel * 8U);
                }
            }
            s_response_values[3] = (int32_t)packed_motor_flags;
            s_response_valid_mask = 0x000FU;
            return ACK_OK;
        }
    case HOST_CONTROL_QUERY_SYSTEM_BOOT:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            SystemHealthBootSnapshot snapshot;
            if (!SystemHealth_GetBootSnapshot(&snapshot)) {
                return ACK_UNSUPPORTED_COMMAND;
            }
            s_response_detail_type = ACK_DETAIL_SYSTEM_BOOT;
            s_response_values[0] = (int32_t)snapshot.boot_count;
            s_response_values[1] = (int32_t)snapshot.reset_flags;
            s_response_values[2] = (int32_t)snapshot.watchdog_reset_count;
            s_response_values[3] = (int32_t)snapshot.crash_count;
            s_response_valid_mask = 0x000FU;
            return ACK_OK;
        }
    case HOST_CONTROL_QUERY_SYSTEM_RUNTIME:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            SystemHealthRuntimeSnapshot snapshot;
            if (!SystemHealth_GetRuntimeSnapshot(&snapshot)) {
                return ACK_UNSUPPORTED_COMMAND;
            }
            s_response_detail_type = ACK_DETAIL_SYSTEM_RUNTIME;
            s_response_values[0] = (int32_t)snapshot.loop_last_us;
            s_response_values[1] = (int32_t)snapshot.loop_max_us;
            s_response_values[2] = (int32_t)snapshot.loop_average_us;
            s_response_values[3] = (int32_t)snapshot.stack_min_free_bytes;
            s_response_valid_mask = 0x000FU;
            return ACK_OK;
        }
    case HOST_CONTROL_QUERY_FIRMWARE:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        s_response_detail_type = ACK_DETAIL_FIRMWARE_IDENTITY;
        s_response_values[0] = (int32_t)SystemHealth_GetFirmwareVersion();
        s_response_values[1] = (int32_t)SystemHealth_GetGitHash();
        s_response_values[2] = (int32_t)SystemHealth_GetConfigHash();
        s_response_values[3] = (int32_t)SystemHealth_GetCapabilities();
        s_response_valid_mask = 0x000FU;
        return ACK_OK;
    case HOST_CONTROL_QUERY_CRASH_REGISTERS:
        if (!payload_length_is(command, 1U) || (command->payload[0] > 1U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            SystemHealthCrashSnapshot snapshot;
            if (!SystemHealth_GetCrashSnapshot(&snapshot)) {
                s_response_detail_type = ACK_DETAIL_CRASH_REGISTERS;
                return ACK_OK;
            }
            s_response_detail_type = ACK_DETAIL_CRASH_REGISTERS;
            if (command->payload[0] == 0U) {
                s_response_values[0] = (int32_t)snapshot.stacked_r0;
                s_response_values[1] = (int32_t)snapshot.stacked_r1;
                s_response_values[2] = (int32_t)snapshot.stacked_r2;
                s_response_values[3] = (int32_t)snapshot.stacked_r3;
            } else {
                s_response_values[0] = (int32_t)snapshot.stacked_r12;
                s_response_values[1] = (int32_t)snapshot.stacked_lr;
                s_response_values[2] = (int32_t)snapshot.stacked_pc;
                s_response_values[3] = (int32_t)snapshot.stacked_xpsr;
            }
            s_response_valid_mask = 0x000FU;
            return ACK_OK;
        }
    case HOST_CONTROL_QUERY_CRASH_FAULTS:
        if (!payload_length_is(command, 1U) || (command->payload[0] > 1U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            SystemHealthCrashSnapshot snapshot;
            if (!SystemHealth_GetCrashSnapshot(&snapshot)) {
                s_response_detail_type = ACK_DETAIL_CRASH_FAULTS;
                return ACK_OK;
            }
            s_response_detail_type = ACK_DETAIL_CRASH_FAULTS;
            if (command->payload[0] == 0U) {
                s_response_values[0] = (int32_t)snapshot.cfsr;
                s_response_values[1] = (int32_t)snapshot.hfsr;
                s_response_values[2] = (int32_t)snapshot.mmfar;
                s_response_values[3] = (int32_t)snapshot.bfar;
            } else {
                s_response_values[0] = (int32_t)snapshot.afsr;
                s_response_values[1] = (int32_t)snapshot.exc_return;
                s_response_values[2] = (int32_t)snapshot.fault_type;
                s_response_values[3] = 0;
            }
            s_response_valid_mask = command->payload[0] == 0U ? 0x000FU
                                                               : 0x0007U;
            return ACK_OK;
        }
    case HOST_CONTROL_QUERY_EVENT_LOG:
        if (!payload_length_is(command, 1U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            SystemHealthEvent event;
            uint32_t event_count = 0U;
            s_response_detail_type = ACK_DETAIL_SYSTEM_EVENT;
            if (SystemHealth_GetEvent(command->payload[0], &event,
                                      &event_count)) {
                s_response_values[0] = (int32_t)event.tick;
                s_response_values[1] = (int32_t)event.code;
                s_response_values[2] = (int32_t)event.detail;
                s_response_values[3] = (int32_t)event_count;
                s_response_valid_mask = 0x000FU;
            }
            return ACK_OK;
        }
    case HOST_CONTROL_QUERY_POWER:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        s_response_detail_type = ACK_DETAIL_SYSTEM_POWER;
        s_response_values[0] = SystemHealth_IsVddLow() ? 1 : 0;
        s_response_values[1] = 2850;
        s_response_values[2] = 0; /* external voltage ADC not wired */
        s_response_values[3] = 0; /* internal temperature ADC not configured */
        s_response_valid_mask = 0x0003U;
        return ACK_OK;
    case HOST_CONTROL_RS00_READ_POSITION:
        return payload_length_is(command, 0U) ? ACK_OK
                                              : ACK_UNSUPPORTED_COMMAND;
    case HOST_CONTROL_RS00_QUERY_FEEDBACK_AGE:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            unsigned wheel;
            const uint32_t now = HAL_GetTick();
            const bool latched =
                SteeringController_GetState() == STEERING_STATE_FAULT;
            s_response_detail_type = ACK_DETAIL_STEERING_FEEDBACK_AGE_MS;
            for (wheel = 0U; wheel < STEERING_MOTOR_COUNT; ++wheel) {
                SteeringMotor motor;
                if (latched) {
                    s_response_values[wheel] = (int32_t)
                        g_steering_debug_feedback_age_ms[wheel];
                } else if (SteeringController_GetMotorSnapshot(
                               (SteeringMotorIndex)wheel, &motor)) {
                    s_response_values[wheel] =
                        (int32_t)(now - motor.last_rx_tick);
                }
                s_response_valid_mask |= (uint16_t)(1U << wheel);
            }
            return ACK_OK;
        }
    case HOST_CONTROL_STEERING_ENABLE:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        if (SteeringController_IsReady()) {
            return ACK_OK;
        }
        if (steering != STEERING_STATE_ARMED) {
            return steering == STEERING_STATE_FAULT
                       ? ACK_REJECTED_STEERING_FAULT
                       : ACK_REJECTED_NOT_HOMED;
        }
        return SteeringController_RequestEnable()
                   ? ACK_ACCEPTED_WAIT_STEERING : ACK_REJECTED_DISABLED;
    case HOST_CONTROL_STEERING_SET_ALL:
        if (!payload_length_is(command, 2U)
            || !SteeringController_IsReady()) {
            return !SteeringController_IsReady()
                       ? ACK_REJECTED_DISABLED : ACK_UNSUPPORTED_COMMAND;
        }
        {
            const float angle = (float)read_i16_le(command->payload) / 1000.0f;
            if (ChassisTranslation_IsMoving()
                || !DriveController_AllWheelsStopped()) {
                ChassisTranslation_Stop();
                return ACK_ACCEPTED_WAIT_STEERING;
            }
            return SteeringController_SetAllAngles(angle, angle, angle, angle)
                       ? ACK_OK : ACK_UNSUPPORTED_COMMAND;
        }
    case HOST_CONTROL_STEERING_SET_EACH:
        if (!payload_length_is(command, 8U)
            || !SteeringController_IsReady()) {
            return !SteeringController_IsReady()
                       ? ACK_REJECTED_DISABLED : ACK_UNSUPPORTED_COMMAND;
        }
        if (ChassisTranslation_IsMoving()
            || !DriveController_AllWheelsStopped()) {
            ChassisTranslation_Stop();
            return ACK_ACCEPTED_WAIT_STEERING;
        }
        return SteeringController_SetAllAngles(
                   (float)read_i16_le(&command->payload[0]) / 1000.0f,
                   (float)read_i16_le(&command->payload[2]) / 1000.0f,
                   (float)read_i16_le(&command->payload[4]) / 1000.0f,
                   (float)read_i16_le(&command->payload[6]) / 1000.0f)
                   ? ACK_OK : ACK_UNSUPPORTED_COMMAND;
    case HOST_CONTROL_STEERING_DISABLE:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        ChassisTranslation_Stop();
        SteeringController_StopAll();
        return ACK_OK;
    case HOST_CONTROL_RS00_CLEAR_FAULT:
    case HOST_CONTROL_RS00_SET_CSP:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        ChassisTranslation_Stop();
        return command->control_id == HOST_CONTROL_RS00_SET_CSP
                   ? (SteeringController_SetCspModeAndRestart()
                          ? ACK_ACCEPTED_WAIT_STEERING
                          : ACK_UNSUPPORTED_COMMAND)
                   : (SteeringController_ClearFaultAndRestart()
                          ? ACK_ACCEPTED_WAIT_STEERING
                          : ACK_UNSUPPORTED_COMMAND);
    case HOST_CONTROL_RS00_QUERY_UID:
        if (!payload_length_is(command, 1U)
            || (command->payload[0] >= STEERING_MOTOR_COUNT)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            SteeringMotor motor;
            const uint8_t wheel = command->payload[0];
            if (!SteeringController_GetMotorSnapshot(
                    (SteeringMotorIndex)wheel, &motor)
                || !motor.uid_received) {
                return ACK_REJECTED_NOT_HOMED;
            }
            s_response_detail_type = ACK_DETAIL_STEERING_UID;
            memcpy(&s_response_values[0], &motor.received_uid[0], 4U);
            memcpy(&s_response_values[1], &motor.received_uid[4], 4U);
            s_response_values[2] = wheel;
            s_response_valid_mask = 0x0003U;
            return ACK_OK;
        }
    case HOST_CONTROL_RS00_SET_LIMITS:
        if (!payload_length_is(command, 4U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        ChassisTranslation_Stop();
        return SteeringController_SetCspLimitsAndRestart(
                   (float)read_u16_le(&command->payload[0]) / 1000.0f,
                   (float)read_u16_le(&command->payload[2]) / 1000.0f)
                   ? ACK_ACCEPTED_WAIT_STEERING : ACK_UNSUPPORTED_COMMAND;
    case HOST_CONTROL_DRIVE_SET_ACCELERATION:
        return payload_length_is(command, 4U)
                   && DriveController_SetAccelerationErpmS(
                          read_i32_le(command->payload))
                   ? ACK_OK : ACK_UNSUPPORTED_COMMAND;
    case HOST_CONTROL_DRIVE_SET_DECELERATION:
        return payload_length_is(command, 4U)
                   && DriveController_SetDecelerationErpmS(
                          read_i32_le(command->payload))
                   ? ACK_OK : ACK_UNSUPPORTED_COMMAND;
    case HOST_CONTROL_DRIVE_SET_ALL:
        if (!payload_length_is(command, 8U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            const int16_t fl = read_i16_le(&command->payload[0]);
            const int16_t fr = read_i16_le(&command->payload[2]);
            const int16_t rl = read_i16_le(&command->payload[4]);
            const int16_t rr = read_i16_le(&command->payload[6]);
            if ((fl == 0) && (fr == 0) && (rl == 0) && (rr == 0)) {
                DriveController_StopAll();
                return ACK_OK;
            }
            if ((DRIVE_ALLOW_RAW_WHEEL_COMMANDS == 0)
                && ((fl != 0) || (fr != 0) || (rl != 0) || (rr != 0))) {
                ChassisTranslation_Stop();
                return ACK_UNSUPPORTED_COMMAND;
            }
            if (!SteeringController_IsCalibrationConfirmed()
                || !SteeringController_IsReady()
                || !DriveController_IsSafetyFeedbackReady()) {
                return ACK_REJECTED_DISABLED;
            }
            return DriveController_SetAllWheelSpeeds(
                       (float)fl / 1000.0f, (float)fr / 1000.0f,
                       (float)rl / 1000.0f, (float)rr / 1000.0f)
                       ? ACK_OK : ACK_UNSUPPORTED_COMMAND;
        }
    case HOST_CONTROL_DRIVE_STOP:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        DriveController_StopAll();
        return ACK_OK;
    case HOST_CONTROL_DRIVE_WITHDRAW_CURRENT:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        DriveController_EmergencyStop();
        return ACK_OK;
    case HOST_CONTROL_DRIVE_QUERY_FEEDBACK:
        if (!payload_length_is(command, 1U)
            || (command->payload[0] > 8U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        {
            unsigned wheel;
            const uint8_t selector = command->payload[0];
            static const uint8_t detail_type[9] = {
                ACK_DETAIL_DRIVE_VALID_MASK,
                ACK_DETAIL_DRIVE_FAULT_BITS,
                ACK_DETAIL_DRIVE_SPEED_ERPM,
                ACK_DETAIL_DRIVE_CURRENT_10MA,
                ACK_DETAIL_DRIVE_POSITION_CENTIDEG,
                ACK_DETAIL_DRIVE_TEMPERATURE_C,
                ACK_DETAIL_DRIVE_FEEDBACK_AGE_MS,
                ACK_DETAIL_DRIVE_SAFETY_FLAGS,
                ACK_DETAIL_DRIVE_VOLTAGE_V
            };
            s_response_detail_type = detail_type[selector];
            for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
                DriveMotorFeedback feedback;
                if (DriveController_GetFeedback(
                        (DriveWheelIndex)wheel, &feedback)) {
                    switch (selector) {
                    case 0U: s_response_values[wheel] =
                                 (int32_t)feedback.valid_mask; break;
                    case 1U: s_response_values[wheel] =
                                 (int32_t)feedback.fault_bits; break;
                    case 2U: s_response_values[wheel] =
                                 feedback.speed_erpm; break;
                    case 3U: s_response_values[wheel] =
                                 feedback.motor_current_10ma; break;
                    case 4U: s_response_values[wheel] =
                                 feedback.position_centideg; break;
                    case 5U: s_response_values[wheel] =
                                 feedback.temperature_c; break;
                    case 6U: s_response_values[wheel] = (int32_t)
                                 DriveController_GetFeedbackAgeMs(
                                     (DriveWheelIndex)wheel); break;
                    case 7U: s_response_values[wheel] = (int32_t)
                                 DriveController_GetSafetyFlags(
                                     (DriveWheelIndex)wheel); break;
                    case 8U: s_response_values[wheel] =
                                 feedback.voltage_v; break;
                    default: break;
                    }
                    s_response_valid_mask |= (uint16_t)(1U << wheel);
                }
            }
            return ACK_OK;
        }
    case HOST_CONTROL_EMERGENCY_STOP:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        ChassisTranslation_EmergencyStop();
        return ACK_OK;
    case HOST_CONTROL_CLEAR_FAULT_RESTART:
        if (!payload_length_is(command, 0U)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        ChassisDebug_ClearFaults();
        (void)SteeringController_ClearFaultAndRestart();
        ChassisTranslation_Init();
        return ACK_ACCEPTED_WAIT_STEERING;
    case HOST_CONTROL_SYSTEM_RESET:
        if (!payload_length_is(command, sizeof(s_reset_key))
            || (memcmp(command->payload, s_reset_key,
                       sizeof(s_reset_key)) != 0)) {
            return ACK_UNSUPPORTED_COMMAND;
        }
        ChassisTranslation_EmergencyStop();
        SystemHealth_RecordEvent(SYSTEM_EVENT_REMOTE_RESET,
                                 command->sequence);
        s_reset_sequence = command->sequence;
        s_reset_request_tick = HAL_GetTick();
        s_reset_ack_sent = false;
        s_reset_requested = true;
        return ACK_ACCEPTED_RESET;
    default:
        return ACK_UNSUPPORTED_COMMAND;
    }
}

static ChassisAckStatus handle_command(const HostCmdVel *command)
{
    const SteeringState steering = SteeringController_GetState();

    if (rs00_fdcan_is_bus_off()) {
        ChassisDebug_SetReject(CHASSIS_REJECT_CAN_FAULT);
        ChassisTranslation_Stop();
        return ACK_CAN_BUS_OFF;
    }
    if (steering == STEERING_STATE_FAULT) {
        ChassisDebug_SetReject(CHASSIS_REJECT_STEERING_FAULT);
        ChassisTranslation_Stop();
        return ACK_REJECTED_STEERING_FAULT;
    }
    if (steering < STEERING_STATE_ARMED) {
        ChassisDebug_SetReject(CHASSIS_REJECT_NOT_HOMED);
        ChassisTranslation_Stop();
        return ACK_REJECTED_NOT_HOMED;
    }
    if (!SteeringController_IsReady()) {
        ChassisDebug_SetReject(CHASSIS_REJECT_DISABLED);
        ChassisTranslation_Stop();
        return ACK_REJECTED_DISABLED;
    }
    if (command->wz_mrad_s != 0) {
        ++s_debug.rejected_angular_commands;
        ChassisDebug_SetReject(CHASSIS_REJECT_NONZERO_ANGULAR_Z);
        ChassisTranslation_Stop();
        return ACK_REJECTED_ANGULAR_Z;
    }
    if (!ChassisTranslation_CommandVelocity(
            (float)command->vx_mm_s / 1000.0f,
            (float)command->vy_mm_s / 1000.0f)) {
        ++s_debug.rejected_motion_commands;
        ChassisDebug_SetReject(CHASSIS_REJECT_INVALID_COMMAND);
        ChassisTranslation_Stop();
        return ACK_UNSUPPORTED_COMMAND;
    }
    ++s_debug.accepted_commands;
    ChassisDebug_SetReject(CHASSIS_REJECT_NONE);
    return ChassisTranslation_GetState() == CHASSIS_TRANSLATION_DRIVING
               ? ACK_ACCEPTED_DRIVING : ACK_ACCEPTED_WAIT_STEERING;
}

void HostCmdVelUart_Task(void)
{
    HostCmdVel command;

    while (s_read_index != s_write_index) {
        HostCmdVelParseResult result;
        const uint8_t byte = s_ring[s_read_index];
        s_read_index =
            (uint8_t)((s_read_index + 1U) % HOST_UART_RING_SIZE);
        result = HostCmdVelParser_PushByteDetailed(
            &s_parser, byte, &command);
        g_chassis_debug.uart_candidate_frame_count =
            s_parser.candidate_frame_count;
        g_chassis_debug.uart_header_error_count =
            s_parser.header_error_count;
        g_chassis_debug.uart_crc_error_count = s_parser.crc_error_count;
        g_chassis_debug.uart_resync_count = s_parser.resync_count;
        if (result == HOST_CMD_VEL_PARSE_INCOMPLETE) {
            continue;
        }
        if (result == HOST_CMD_VEL_PARSE_BAD_HEADER) {
            DebugHook_UartFrameRejected(ACK_BAD_HEADER);
            queue_ack(s_debug.last_sequence, ACK_BAD_HEADER);
            continue;
        }
        ChassisDebug_RecordRawFrame(s_parser.last_frame);
        if (result == HOST_CMD_VEL_PARSE_BAD_CRC) {
            const uint8_t rejected_sequence =
                s_parser.last_frame[2] == HOST_CONTROL_VERSION
                    ? s_parser.last_frame[4] : s_parser.last_frame[3];
            DebugHook_UartFrameRejected(ACK_BAD_CRC);
            queue_ack(rejected_sequence, ACK_BAD_CRC);
            continue;
        }
        if (result == HOST_CMD_VEL_PARSE_BAD_VERSION) {
            DebugHook_UartFrameRejected(ACK_UNSUPPORTED_COMMAND);
            queue_ack(s_parser.last_frame[3], ACK_UNSUPPORTED_COMMAND);
            continue;
        }

        ++g_chassis_debug.uart_valid_frame_count;
        DebugHook_UartFrameValid();
        s_debug.last_sequence = command.sequence;
        if (s_have_sequence && (command.sequence != s_expected_sequence)) {
            ++g_chassis_debug.uart_sequence_error_count;
        }
        s_have_sequence = true;
        s_expected_sequence = (uint8_t)(command.sequence + 1U);
        if (command.version == HOST_CONTROL_VERSION) {
            ChassisDebug_RecordCommand(
                command.sequence, 0, 0, 0, HAL_GetTick());
            queue_ack(command.sequence, handle_control_command(&command));
            if (s_reset_requested) {
                break;
            }
        } else {
            ChassisDebug_RecordCommand(
                command.sequence, command.vx_mm_s, command.vy_mm_s,
                command.wz_mrad_s, HAL_GetTick());
            queue_ack(command.sequence, handle_command(&command));
        }
    }
    s_debug.crc_errors = s_parser.crc_error_count;
    if (s_seen_timeout_count != g_chassis_debug.uart_timeout_count) {
        s_seen_timeout_count = g_chassis_debug.uart_timeout_count;
        queue_ack(s_debug.last_sequence, ACK_TIMEOUT_STOP);
    }
    if (s_seen_can_error_count
        != g_chassis_debug.can_tx_submit_error_count) {
        s_seen_can_error_count =
            g_chassis_debug.can_tx_submit_error_count;
        queue_ack(s_debug.last_sequence, ACK_CAN_TX_ERROR);
    }
    if (!s_seen_bus_off && (g_chassis_debug.fdcan_bus_off != 0U)) {
        s_seen_bus_off = true;
        queue_ack(s_debug.last_sequence, ACK_CAN_BUS_OFF);
    }

#if CHASSIS_UART_ACK_ENABLE
    if (!s_ack_tx_busy && (s_ack_read != s_ack_write)) {
        if (HAL_UART_Transmit_IT(s_uart, s_ack_queue[s_ack_read],
                                CHASSIS_ACK_FRAME_SIZE) == HAL_OK) {
            s_ack_tx_busy = true;
        }
    }
#endif

    if (s_reset_requested
        && (s_reset_ack_sent
            || ((HAL_GetTick() - s_reset_request_tick)
                >= HOST_SYSTEM_RESET_ACK_TIMEOUT_MS))) {
        SystemWatchdog_SystemReset();
    }
}

bool HostCmdVelUart_GetDebugSnapshot(HostCmdVelUartDebug *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_debug;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}
