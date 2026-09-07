#ifndef HOST_CMD_VEL_UART_H
#define HOST_CMD_VEL_UART_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t accepted_commands;
    uint32_t rejected_angular_commands;
    uint32_t rejected_motion_commands;
    uint32_t uart_overruns;
    uint32_t uart_errors;
    uint32_t crc_errors;
    uint8_t last_sequence;
} HostCmdVelUartDebug;

bool HostCmdVelUart_Init(UART_HandleTypeDef *uart);
void HostCmdVelUart_Task(void);
void HostCmdVelUart_RxCompleteCallback(UART_HandleTypeDef *uart);
void HostCmdVelUart_TxCompleteCallback(UART_HandleTypeDef *uart);
void HostCmdVelUart_ErrorCallback(UART_HandleTypeDef *uart);
bool HostCmdVelUart_GetDebugSnapshot(HostCmdVelUartDebug *snapshot);

#endif
