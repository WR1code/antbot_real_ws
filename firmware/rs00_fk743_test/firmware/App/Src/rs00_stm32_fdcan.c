#include "rs00_stm32_fdcan.h"

#include "chassis_debug.h"
#include "drive_config.h"

static FDCAN_HandleTypeDef *s_hfdcan;
static FDCAN_HandleTypeDef *s_mini_hfdcan;
static rs00_rx_callback_t s_rx_callback;
static mini_rx_callback_t s_mini_rx_callback;
static volatile uint32_t s_rx_drop_count;
static bool s_started;
static bool s_mini_started;
static bool s_bus_off_latched;
static bool s_mini_bus_off_latched;
static uint32_t s_last_status_tick;

static bool is_mini_id(uint32_t id)
{
    return (id == DRIVE_CAN_ID_FL) || (id == DRIVE_CAN_ID_FR)
           || (id == DRIVE_CAN_ID_RL) || (id == DRIVE_CAN_ID_RR);
}

static uint8_t dlc_to_length(uint32_t dlc)
{
    static const uint32_t values[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    uint8_t length;
    for (length = 0U; length <= 8U; ++length) {
        if (values[length] == dlc) {
            return length;
        }
    }
    return 0U;
}

bool rs00_fdcan_start(FDCAN_HandleTypeDef *hfdcan,
                      rs00_rx_callback_t rx_callback)
{
    FDCAN_FilterTypeDef filter = {0};

    if ((hfdcan == NULL) || (rx_callback == NULL)) {
        return false;
    }
    if (s_started) {
        if (hfdcan != s_hfdcan) {
            return false;
        }
        s_rx_callback = rx_callback;
        return true;
    }

    /*
     * RS00 replies use several extended-ID layouts. Accept all extended data
     * frames into FIFO0 during bring-up. Keep the pre-existing standard-filter
     * slot for layout compatibility; MINI traffic now uses the other instance.
     */
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return false;
    }

    /* Standard traffic is not dispatched to the RS00 callback. */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return false;
    }

    s_hfdcan = hfdcan;
    s_rx_callback = rx_callback;
    s_rx_drop_count = 0U;
    s_bus_off_latched = false;
    s_last_status_tick = 0U;

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(
            hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY,
            0U) != HAL_OK) {
        return false;
    }
    s_started = true;
    return true;
}

bool mini_fdcan_start(FDCAN_HandleTypeDef *hfdcan,
                      mini_rx_callback_t rx_callback)
{
    FDCAN_FilterTypeDef filter = {0};

    if ((hfdcan == NULL) || (rx_callback == NULL)) {
        return false;
    }
    if (s_mini_started) {
        if (hfdcan != s_mini_hfdcan) {
            return false;
        }
        s_mini_rx_callback = rx_callback;
        return true;
    }
    if (hfdcan == s_hfdcan) {
        /* Different bit rates require physically separate FDCAN instances. */
        return false;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return false;
    }

    s_mini_hfdcan = hfdcan;
    s_mini_rx_callback = rx_callback;
    s_mini_bus_off_latched = false;
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        s_mini_hfdcan = NULL;
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(
            hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY,
            0U) != HAL_OK) {
        s_mini_hfdcan = NULL;
        return false;
    }
    s_mini_started = true;
    return true;
}

bool rs00_fdcan_send(const rs00_frame_t *frame)
{
    return rs00_fdcan_send_detailed(frame) == RS00_FDCAN_OK;
}

rs00_fdcan_result_t rs00_fdcan_send_detailed(const rs00_frame_t *frame)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((frame == NULL) || (frame->id > RS00_EXT_ID_MASK)) {
        return RS00_FDCAN_INVALID_ARGUMENT;
    }
    if ((s_hfdcan == NULL) || !s_started) {
        return RS00_FDCAN_NOT_STARTED;
    }
    if (s_bus_off_latched) {
        ChassisDebug_RecordCanTx(frame->id, frame->data, 8U, false,
                                 HAL_GetTick());
        DebugHook_CanTxFailed(frame->id, RS00_FDCAN_HAL_ERROR);
        return RS00_FDCAN_HAL_ERROR;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) == 0U) {
        ChassisDebug_RecordCanTx(frame->id, frame->data, 8U, false,
                                 HAL_GetTick());
        DebugHook_CanTxFailed(frame->id, RS00_FDCAN_TX_FIFO_FULL);
        return RS00_FDCAN_TX_FIFO_FULL;
    }

    header.Identifier = frame->id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    DebugHook_CanTxBeforeSubmit(frame->id, frame->data);
    if (HAL_FDCAN_AddMessageToTxFifoQ(
            s_hfdcan, &header, (uint8_t *)frame->data) != HAL_OK) {
        ChassisDebug_RecordCanTx(frame->id, frame->data, 8U, false,
                                 HAL_GetTick());
        DebugHook_CanTxFailed(frame->id, RS00_FDCAN_HAL_ERROR);
        return RS00_FDCAN_HAL_ERROR;
    }
    ChassisDebug_RecordCanTx(frame->id, frame->data, 8U, true, HAL_GetTick());
    return RS00_FDCAN_OK;
}

bool rs00_fdcan_send_standard(uint16_t standard_id, const uint8_t *data,
                              uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    static const uint32_t dlc[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };

    if ((standard_id > 0x7FFU) || (data == NULL)
        || (length == 0U) || (length > 8U)
        || (s_mini_hfdcan == NULL) || !s_mini_started
        || s_mini_bus_off_latched) {
        if (data != NULL) {
            ChassisDebug_RecordCanTx(standard_id, data, length, false,
                                     HAL_GetTick());
        }
        return false;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_mini_hfdcan) == 0U) {
        ChassisDebug_RecordCanTx(standard_id, data, length, false,
                                 HAL_GetTick());
        DebugHook_CanTxFailed(standard_id, RS00_FDCAN_TX_FIFO_FULL);
        return false;
    }

    header.Identifier = standard_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc[length];
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    DebugHook_CanTxBeforeSubmit(standard_id, data);
    if (HAL_FDCAN_AddMessageToTxFifoQ(
            s_mini_hfdcan, &header, (uint8_t *)data) != HAL_OK) {
        ChassisDebug_RecordCanTx(standard_id, data, length, false,
                                 HAL_GetTick());
        DebugHook_CanTxFailed(standard_id, RS00_FDCAN_HAL_ERROR);
        return false;
    }
    ChassisDebug_RecordCanTx(standard_id, data, length, true, HAL_GetTick());
    return true;
}

void rs00_fdcan_on_rx_fifo0(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef header;
    rs00_frame_t frame;

    if ((hfdcan != s_hfdcan) && (hfdcan != s_mini_hfdcan)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        uint8_t length;
        if (HAL_FDCAN_GetRxMessage(
                hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            ++s_rx_drop_count;
            return;
        }
        if (header.RxFrameType != FDCAN_DATA_FRAME) {
            ++s_rx_drop_count;
            continue;
        }
        length = dlc_to_length(header.DataLength);
        {
            const bool rs00 = header.IdType == FDCAN_EXTENDED_ID
                               && length == 8U;
            const bool mini = header.IdType == FDCAN_STANDARD_ID
                               && is_mini_id(header.Identifier);
            ChassisDebug_RecordCanRx(header.Identifier, frame.data, length,
                                     rs00, mini, HAL_GetTick());
        }
        /*
         * MINI query/monitor replies are standard frames. They are accepted
         * by the hardware so a future traction-feedback decoder can consume
         * them, but the RS00 callback must only receive its fixed 8-byte
         * extended frames.
         */
        if (header.IdType == FDCAN_STANDARD_ID) {
            if ((hfdcan == s_mini_hfdcan) && (s_mini_rx_callback != NULL)
                && is_mini_id(header.Identifier)) {
                s_mini_rx_callback((uint16_t)header.Identifier,
                                   frame.data, length);
            }
            continue;
        }
        if ((hfdcan != s_hfdcan) || (s_rx_callback == NULL)) {
            ++s_rx_drop_count;
            continue;
        }
        if ((header.IdType != FDCAN_EXTENDED_ID)
            || (header.DataLength != FDCAN_DLC_BYTES_8)) {
            ++s_rx_drop_count;
            continue;
        }
        frame.id = header.Identifier;
        s_rx_callback(&frame);
    }
}

void rs00_fdcan_on_tx_fifo_empty(FDCAN_HandleTypeDef *hfdcan)
{
    if ((hfdcan == s_hfdcan) || (hfdcan == s_mini_hfdcan)) {
        ++g_chassis_debug.can_tx_complete_count;
    }
}

static void update_bus_status(FDCAN_HandleTypeDef *hfdcan,
                              bool *bus_off_latched)
{
    FDCAN_ProtocolStatusTypeDef protocol;
    FDCAN_ErrorCountersTypeDef counters;

    if ((hfdcan == NULL)
        || (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol) != HAL_OK)
        || (HAL_FDCAN_GetErrorCounters(hfdcan, &counters) != HAL_OK)) {
        ChassisDebug_SetFault(CHASSIS_FAULT_CAN);
        return;
    }
    ChassisDebug_RecordFdcanStatus(protocol.BusOff != 0U,
                                   protocol.ErrorPassive != 0U,
                                   protocol.Warning != 0U,
                                   protocol.LastErrorCode,
                                   counters.TxErrorCnt,
                                   counters.RxErrorCnt);
    if ((protocol.BusOff != 0U) && !*bus_off_latched) {
        *bus_off_latched = true;
        ChassisDebug_SetFault(CHASSIS_FAULT_CAN
                              | CHASSIS_FAULT_CAN_BUS_OFF);
        DebugHook_Fault(CHASSIS_FAULT_CAN_BUS_OFF);
    }
}

void rs00_fdcan_debug_task(void)
{
    const uint32_t now = HAL_GetTick();

    if ((s_hfdcan == NULL) || !s_started
        || ((now - s_last_status_tick) < 20U)) {
        return;
    }
    s_last_status_tick = now;
    update_bus_status(s_hfdcan, &s_bus_off_latched);
    if (s_mini_started) {
        update_bus_status(s_mini_hfdcan, &s_mini_bus_off_latched);
    }
}

bool rs00_fdcan_is_bus_off(void)
{
    return s_bus_off_latched || s_mini_bus_off_latched;
}

uint32_t rs00_fdcan_get_rx_drop_count(void)
{
    return s_rx_drop_count;
}
