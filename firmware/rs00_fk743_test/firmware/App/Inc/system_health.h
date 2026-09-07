#ifndef SYSTEM_HEALTH_H
#define SYSTEM_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

/* Capabilities reported to the host. Hardware-dependent features stay off
 * until their pins and electrical limits are explicitly configured. */
#define SYSTEM_CAP_IWDG              (1UL << 0)
#define SYSTEM_CAP_BACKUP_BLACKBOX   (1UL << 1)
#define SYSTEM_CAP_HARDFAULT_CAPTURE (1UL << 2)
#define SYSTEM_CAP_LOOP_MONITOR      (1UL << 3)
#define SYSTEM_CAP_STACK_WATERMARK   (1UL << 4)
#define SYSTEM_CAP_PVD_2V85          (1UL << 5)
#define SYSTEM_CAP_EXTERNAL_VOLTAGE  (1UL << 6)
#define SYSTEM_CAP_MCU_TEMPERATURE   (1UL << 7)
#define SYSTEM_CAP_EXTERNAL_WATCHDOG (1UL << 8)
#define SYSTEM_CAP_SAFE_UPDATER      (1UL << 9)
#define SYSTEM_CAP_PHYSICAL_ESTOP    (1UL << 10)
#define SYSTEM_CAP_STEERING_CALIBRATED (1UL << 11)
#define SYSTEM_CAP_DRIVE_FEEDBACK_GUARD (1UL << 12)

typedef enum {
    SYSTEM_EVENT_BOOT = 1,
    SYSTEM_EVENT_WATCHDOG_RESET = 2,
    SYSTEM_EVENT_CPU_FAULT = 3,
    SYSTEM_EVENT_REMOTE_RESET = 4,
    SYSTEM_EVENT_VDD_LOW = 5,
    SYSTEM_EVENT_VDD_RECOVERED = 6,
    SYSTEM_EVENT_ERROR_HANDLER = 7,
    SYSTEM_EVENT_PHYSICAL_ESTOP = 8,
    SYSTEM_EVENT_DRIVE_SAFETY = 9,
    SYSTEM_EVENT_STEERING_SAFETY = 10
} SystemHealthEventCode;

typedef struct {
    uint32_t boot_count;
    uint32_t reset_flags;
    uint32_t watchdog_reset_count;
    uint32_t crash_count;
} SystemHealthBootSnapshot;

typedef struct {
    uint32_t loop_last_us;
    uint32_t loop_max_us;
    uint32_t loop_average_us;
    uint32_t stack_min_free_bytes;
} SystemHealthRuntimeSnapshot;

typedef struct {
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_xpsr;
    uint32_t exc_return;
    uint32_t fault_type;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t afsr;
} SystemHealthCrashSnapshot;

typedef struct {
    uint32_t tick;
    uint32_t code;
    uint32_t detail;
} SystemHealthEvent;

bool SystemHealth_Init(uint32_t reset_flags);
void SystemHealth_RuntimeInit(void);
void SystemHealth_LoopBegin(void);
void SystemHealth_LoopEnd(void);
void SystemHealth_Task(void);
void SystemHealth_RecordEvent(SystemHealthEventCode code, uint32_t detail);
void SystemHealth_RecordErrorHandler(uint32_t detail);
bool SystemHealth_GetBootSnapshot(SystemHealthBootSnapshot *snapshot);
bool SystemHealth_GetRuntimeSnapshot(SystemHealthRuntimeSnapshot *snapshot);
bool SystemHealth_GetCrashSnapshot(SystemHealthCrashSnapshot *snapshot);
bool SystemHealth_GetEvent(uint32_t newest_index, SystemHealthEvent *event,
                           uint32_t *event_count);
uint32_t SystemHealth_GetCapabilities(void);
uint32_t SystemHealth_GetFirmwareVersion(void);
uint32_t SystemHealth_GetGitHash(void);
uint32_t SystemHealth_GetConfigHash(void);
bool SystemHealth_IsVddLow(void);

/* Called directly by the Cortex exception wrappers; never returns. */
void SystemHealth_CaptureFault(uint32_t *stack_frame, uint32_t exc_return,
                               uint32_t fault_type)
    __attribute__((noreturn));

#endif
