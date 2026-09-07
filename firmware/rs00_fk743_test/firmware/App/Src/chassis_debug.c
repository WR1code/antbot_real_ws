#include "chassis_debug.h"

#include <string.h>

volatile ChassisDebugSnapshot g_chassis_debug;
volatile uint8_t uart_last_raw_frame[12];

#if defined(__GNUC__)
#define DEBUG_NOINLINE __attribute__((noinline))
#else
#define DEBUG_NOINLINE
#endif

#if CHASSIS_DEBUG_ENABLE
static void copy_volatile(uint8_t volatile *destination, const uint8_t *source,
                          uint8_t length)
{
    uint8_t index;
    for (index = 0U; index < length; ++index) {
        destination[index] = source[index];
    }
    for (; index < 8U; ++index) {
        destination[index] = 0U;
    }
}
#endif

void ChassisDebug_Init(void)
{
    memset((void *)&g_chassis_debug, 0, sizeof(g_chassis_debug));
    memset((void *)uart_last_raw_frame, 0, sizeof(uart_last_raw_frame));
    g_chassis_debug.magic = CHASSIS_DEBUG_MAGIC;
    g_chassis_debug.version = CHASSIS_DEBUG_VERSION;
    g_chassis_debug.chassis_state = CHASSIS_STATE_BOOT;
}

void ChassisDebug_TaskTick(uint32_t tick)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.system_tick = tick;
#else
    (void)tick;
#endif
}

void ChassisDebug_RecordUartRx(uint32_t length, uint32_t tick)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.uart_rx_byte_count += length;
    ++g_chassis_debug.uart_rx_event_count;
    g_chassis_debug.uart_last_rx_length = length;
    g_chassis_debug.uart_last_rx_tick = tick;
    DebugHook_UartRx();
#else
    (void)length;
    (void)tick;
#endif
}

void ChassisDebug_RecordRawFrame(const uint8_t frame[12])
{
#if CHASSIS_DEBUG_ENABLE
    uint8_t index;
    for (index = 0U; index < 12U; ++index) {
        uart_last_raw_frame[index] = frame[index];
    }
#else
    (void)frame;
#endif
}

void ChassisDebug_RecordCommand(uint8_t sequence, int16_t vx, int16_t vy,
                                int16_t wz, uint32_t tick)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.command_sequence = sequence;
    g_chassis_debug.command_vx_raw = vx;
    g_chassis_debug.command_vy_raw = vy;
    g_chassis_debug.command_wz_raw = wz;
    g_chassis_debug.command_vx_mps = (float)vx / 1000.0f;
    g_chassis_debug.command_vy_mps = (float)vy / 1000.0f;
    g_chassis_debug.command_wz_rps = (float)wz / 1000.0f;
    g_chassis_debug.last_valid_command_tick = tick;
#else
    (void)sequence; (void)vx; (void)vy; (void)wz; (void)tick;
#endif
}

void ChassisDebug_SetState(ChassisDebugState state, uint32_t tick)
{
#if CHASSIS_DEBUG_ENABLE
    const uint32_t old_state = g_chassis_debug.chassis_state;
    if (old_state != (uint32_t)state) {
        g_chassis_debug.previous_chassis_state = old_state;
        g_chassis_debug.chassis_state = (uint32_t)state;
        ++g_chassis_debug.chassis_state_change_count;
        g_chassis_debug.last_state_change_tick = tick;
        DebugHook_ChassisStateChanged(old_state, (uint32_t)state);
    }
#else
    (void)state; (void)tick;
#endif
}

void ChassisDebug_SetReject(ChassisRejectReason reason)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.chassis_reject_reason = (uint32_t)reason;
    if (reason != CHASSIS_REJECT_NONE) {
        DebugHook_UartFrameRejected((uint32_t)reason);
    }
#else
    (void)reason;
#endif
}

void ChassisDebug_SetFault(uint32_t fault_flags)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.chassis_fault_flags |= fault_flags;
    if (fault_flags != 0U) {
        DebugHook_Fault(g_chassis_debug.chassis_fault_flags);
    }
#else
    (void)fault_flags;
#endif
}

void ChassisDebug_ClearFaults(void)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.chassis_fault_flags = CHASSIS_FAULT_NONE;
    g_chassis_debug.chassis_reject_reason = CHASSIS_REJECT_NONE;
#endif
}

void ChassisDebug_SetSteering(bool enabled, bool homed, bool ready, bool fault)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.steering_enabled = enabled ? 1U : 0U;
    g_chassis_debug.steering_homed = homed ? 1U : 0U;
    g_chassis_debug.steering_ready = ready ? 1U : 0U;
    g_chassis_debug.steering_fault = fault ? 1U : 0U;
#else
    (void)enabled; (void)homed; (void)ready; (void)fault;
#endif
}

void ChassisDebug_RecordCanTx(uint32_t id, const uint8_t *data,
                              uint8_t length, bool submitted, uint32_t tick)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.last_can_tx_id = id;
    g_chassis_debug.last_can_tx_dlc = length;
    g_chassis_debug.last_can_tx_tick = tick;
    g_chassis_debug.last_can_tx_success = submitted ? 1U : 0U;
    copy_volatile(g_chassis_debug.last_can_tx_data, data, length);
    if (submitted) {
        ++g_chassis_debug.can_tx_submit_ok_count;
        DebugHook_CanTxSubmitted(id, data);
    } else {
        ++g_chassis_debug.can_tx_submit_error_count;
    }
#else
    (void)id; (void)data; (void)length; (void)submitted; (void)tick;
#endif
}

void ChassisDebug_RecordCanRx(uint32_t id, const uint8_t *data,
                              uint8_t length, bool rs00, bool mini,
                              uint32_t tick)
{
#if CHASSIS_DEBUG_ENABLE
    ++g_chassis_debug.can_rx_frame_count;
    g_chassis_debug.last_can_rx_id = id;
    g_chassis_debug.last_can_rx_dlc = length;
    g_chassis_debug.last_can_rx_tick = tick;
    g_chassis_debug.last_can_rx_valid = (rs00 || mini) ? 1U : 0U;
    copy_volatile(g_chassis_debug.last_can_rx_data, data, length);
    if (rs00) {
        ++g_chassis_debug.can_rx_rs00_count;
    } else if (mini) {
        ++g_chassis_debug.can_rx_mini_count;
    } else {
        ++g_chassis_debug.can_unknown_id_count;
    }
    DebugHook_CanRxReceived(id, data);
#else
    (void)id; (void)data; (void)length; (void)rs00; (void)mini; (void)tick;
#endif
}

void ChassisDebug_RecordFdcanStatus(bool bus_off, bool error_passive,
                                   bool warning, uint32_t last_error,
                                   uint32_t tx_errors, uint32_t rx_errors)
{
#if CHASSIS_DEBUG_ENABLE
    g_chassis_debug.fdcan_bus_off = bus_off ? 1U : 0U;
    g_chassis_debug.fdcan_error_passive = error_passive ? 1U : 0U;
    g_chassis_debug.fdcan_warning = warning ? 1U : 0U;
    g_chassis_debug.fdcan_last_error_code = last_error;
    g_chassis_debug.fdcan_tx_error_count = tx_errors;
    g_chassis_debug.fdcan_rx_error_count = rx_errors;
#else
    (void)bus_off; (void)error_passive; (void)warning;
    (void)last_error; (void)tx_errors; (void)rx_errors;
#endif
}

#define DEFINE_HOOK0(name) \
    DEBUG_NOINLINE void name(void) { ++g_chassis_debug.debug_hook_counter; }

DEFINE_HOOK0(DebugHook_UartRx)
DEFINE_HOOK0(DebugHook_UartFrameValid)
DEFINE_HOOK0(DebugHook_TimeoutStop)

DEBUG_NOINLINE void DebugHook_UartFrameRejected(uint32_t reason)
{
    g_chassis_debug.debug_hook_last_reason = reason;
    ++g_chassis_debug.debug_hook_counter;
}

DEBUG_NOINLINE void DebugHook_ChassisStateChanged(uint32_t old_state,
                                                  uint32_t new_state)
{
    g_chassis_debug.debug_hook_last_reason = (old_state << 16U) | new_state;
    ++g_chassis_debug.debug_hook_counter;
}

DEBUG_NOINLINE void DebugHook_CanTxBeforeSubmit(uint32_t id,
                                                const uint8_t *data)
{
    (void)data;
    g_chassis_debug.debug_hook_last_reason = id;
    ++g_chassis_debug.debug_hook_counter;
}

DEBUG_NOINLINE void DebugHook_CanTxSubmitted(uint32_t id,
                                             const uint8_t *data)
{
    (void)data;
    g_chassis_debug.debug_hook_last_reason = id;
    ++g_chassis_debug.debug_hook_counter;
}

DEBUG_NOINLINE void DebugHook_CanTxFailed(uint32_t id, uint32_t error)
{
    g_chassis_debug.debug_hook_last_reason = id ^ error;
    ++g_chassis_debug.debug_hook_counter;
}

DEBUG_NOINLINE void DebugHook_CanRxReceived(uint32_t id,
                                            const uint8_t *data)
{
    (void)data;
    g_chassis_debug.debug_hook_last_reason = id;
    ++g_chassis_debug.debug_hook_counter;
}

DEBUG_NOINLINE void DebugHook_Fault(uint32_t fault_flags)
{
    g_chassis_debug.debug_hook_last_reason = fault_flags;
    ++g_chassis_debug.debug_hook_counter;
}
