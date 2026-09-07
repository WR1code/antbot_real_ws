#ifndef BOARD_RELIABILITY_IO_H
#define BOARD_RELIABILITY_IO_H

#include <stdbool.h>

void BoardReliabilityIO_Init(void);
void BoardReliabilityIO_PollInputs(void);
void BoardReliabilityIO_Heartbeat(void);
bool BoardReliabilityIO_IsEmergencyStopActive(void);
bool BoardReliabilityIO_HasExternalWatchdog(void);

#endif
