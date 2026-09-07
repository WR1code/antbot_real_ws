#ifndef TEST_MOCK_STM32H7XX_HAL_H
#define TEST_MOCK_STM32H7XX_HAL_H

#include <stdint.h>

typedef struct {
    uint32_t dummy;
} FDCAN_HandleTypeDef;

typedef struct {
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

typedef struct {
    uint32_t LastErrorCode;
    uint32_t ErrorPassive;
    uint32_t Warning;
    uint32_t BusOff;
} FDCAN_ProtocolStatusTypeDef;

typedef struct {
    uint32_t TxErrorCnt;
    uint32_t RxErrorCnt;
} FDCAN_ErrorCountersTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

#define FDCAN_EXTENDED_ID             1U
#define FDCAN_STANDARD_ID             14U
#define FDCAN_FILTER_MASK             2U
#define FDCAN_FILTER_TO_RXFIFO0       3U
#define FDCAN_REJECT                  4U
#define FDCAN_REJECT_REMOTE           5U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 6U
#define FDCAN_DATA_FRAME              7U
#define FDCAN_DLC_BYTES_8             8U
#define FDCAN_DLC_BYTES_0             0U
#define FDCAN_DLC_BYTES_1             1U
#define FDCAN_DLC_BYTES_2             2U
#define FDCAN_DLC_BYTES_3             3U
#define FDCAN_DLC_BYTES_4             4U
#define FDCAN_DLC_BYTES_5             5U
#define FDCAN_DLC_BYTES_6             6U
#define FDCAN_DLC_BYTES_7             7U
#define FDCAN_ESI_ACTIVE              9U
#define FDCAN_BRS_OFF                 10U
#define FDCAN_CLASSIC_CAN             11U
#define FDCAN_NO_TX_EVENTS            12U
#define FDCAN_RX_FIFO0                13U
#define FDCAN_IT_TX_FIFO_EMPTY        15U
#define __DMB()                       ((void)0)

static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __enable_irq(void)
{
}

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t non_matching_standard,
    uint32_t non_matching_extended,
    uint32_t reject_standard_remote,
    uint32_t reject_extended_remote);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(
    FDCAN_HandleTypeDef *hfdcan, uint32_t active_it, uint32_t buffer_indexes);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *hfdcan,
    FDCAN_TxHeaderTypeDef *header,
    uint8_t *data);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(
    FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_location,
    FDCAN_RxHeaderTypeDef *header,
    uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *protocol_status);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(
    FDCAN_HandleTypeDef *hfdcan, FDCAN_ErrorCountersTypeDef *error_counters);

#endif
