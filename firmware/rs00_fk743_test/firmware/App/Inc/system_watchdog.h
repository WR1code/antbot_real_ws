#ifndef SYSTEM_WATCHDOG_H
#define SYSTEM_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/* Nominal timeout using the 32 kHz LSI, prescaler 256 and reload 249. */
#define SYSTEM_WATCHDOG_TIMEOUT_MS 2000U

bool SystemWatchdog_Init(void);
void SystemWatchdog_Refresh(void);
uint32_t SystemWatchdog_GetBootResetFlags(void);
bool SystemWatchdog_WasWatchdogReset(void);
void SystemWatchdog_SystemReset(void) __attribute__((noreturn));

#endif
