#include "system_health.h"

#include "stm32h7xx_hal.h"
#include "board_reliability_config.h"
#include "steering_config.h"

#include <stddef.h>
#include <string.h>

#ifndef FIRMWARE_VERSION_PACKED
#define FIRMWARE_VERSION_PACKED 0x00010002UL
#endif
#ifndef FIRMWARE_GIT_HASH32
#define FIRMWARE_GIT_HASH32 0UL
#endif
#ifndef FIRMWARE_CONFIG_HASH32
#define FIRMWARE_CONFIG_HASH32 0UL
#endif

#define HEALTH_MAGIC 0x484C5448UL /* "HLTH" */
#define HEALTH_LAYOUT_VERSION 1U
#define HEALTH_EVENT_CAPACITY 32U
#define HEALTH_STACK_PATTERN 0xA5A5A5A5UL
#define HEALTH_STACK_GUARD_BYTES 128U
#define HEALTH_PVD_THRESHOLD_MV 2850U

typedef struct {
    uint32_t magic;
    uint16_t layout_version;
    uint16_t size;
    uint32_t checksum;
    uint32_t boot_count;
    uint32_t reset_flags;
    uint32_t watchdog_reset_count;
    uint32_t crash_count;
    uint32_t event_count;
    uint32_t event_write_index;
    SystemHealthCrashSnapshot crash;
    SystemHealthEvent events[HEALTH_EVENT_CAPACITY];
} PersistentHealth;

_Static_assert(sizeof(PersistentHealth) <= 4096U,
               "Persistent health data must fit in Backup SRAM");
_Static_assert((sizeof(PersistentHealth) % sizeof(uint32_t)) == 0U,
               "Persistent health data must be word aligned");

extern uint32_t _sstack;
extern uint32_t _estack;

static volatile PersistentHealth *const s_persistent =
    (volatile PersistentHealth *)D3_BKPSRAM_BASE;
static bool s_initialized;
static bool s_vdd_low;
static uint32_t s_loop_started_cycles;
static uint32_t s_loop_last_us;
static uint32_t s_loop_max_us;
static uint32_t s_loop_average_us;
static uint32_t s_loop_samples;
static uint32_t s_stack_min_free_bytes;

static uint32_t checksum_bytes(const volatile uint8_t *data, size_t length)
{
    uint32_t hash = 2166136261UL;
    size_t index;
    for (index = 0U; index < length; ++index) {
        hash ^= data[index];
        hash *= 16777619UL;
    }
    return hash;
}

static uint32_t persistent_checksum(void)
{
    const size_t prefix = offsetof(PersistentHealth, checksum);
    const size_t suffix = prefix + sizeof(s_persistent->checksum);
    uint32_t hash = checksum_bytes((const volatile uint8_t *)s_persistent,
                                   prefix);
    const volatile uint8_t *data = (const volatile uint8_t *)s_persistent;
    size_t index;
    for (index = suffix; index < sizeof(*s_persistent); ++index) {
        hash ^= data[index];
        hash *= 16777619UL;
    }
    return hash;
}

static void commit_persistent(void)
{
    s_persistent->checksum = persistent_checksum();
    __DSB();
}

static bool persistent_is_valid(void)
{
    return (s_persistent->magic == HEALTH_MAGIC)
        && (s_persistent->layout_version == HEALTH_LAYOUT_VERSION)
        && (s_persistent->size == sizeof(*s_persistent))
        && (s_persistent->checksum == persistent_checksum());
}

static void clear_persistent(void)
{
    volatile uint32_t *word = (volatile uint32_t *)s_persistent;
    size_t index;
    for (index = 0U; index < sizeof(*s_persistent) / sizeof(uint32_t); ++index) {
        word[index] = 0U;
    }
    s_persistent->magic = HEALTH_MAGIC;
    s_persistent->layout_version = HEALTH_LAYOUT_VERSION;
    s_persistent->size = sizeof(*s_persistent);
    commit_persistent();
}

void SystemHealth_RecordEvent(SystemHealthEventCode code, uint32_t detail)
{
    uint32_t index;
    if (!s_initialized) {
        return;
    }
    index = s_persistent->event_write_index % HEALTH_EVENT_CAPACITY;
    s_persistent->events[index].tick = HAL_GetTick();
    s_persistent->events[index].code = (uint32_t)code;
    s_persistent->events[index].detail = detail;
    s_persistent->event_write_index = (index + 1U) % HEALTH_EVENT_CAPACITY;
    if (s_persistent->event_count < UINT32_MAX) {
        ++s_persistent->event_count;
    }
    commit_persistent();
}

bool SystemHealth_Init(uint32_t reset_flags)
{
    PWR_PVDTypeDef pvd = {0};

    __HAL_RCC_BKPRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    if (HAL_PWREx_EnableBkUpReg() != HAL_OK) {
        return false;
    }
    if (!persistent_is_valid()) {
        clear_persistent();
    }
    s_initialized = true;
    ++s_persistent->boot_count;
    s_persistent->reset_flags = reset_flags;
    if ((reset_flags & RCC_RSR_IWDG1RSTF) != 0U) {
        ++s_persistent->watchdog_reset_count;
    }
    commit_persistent();

    pvd.PVDLevel = PWR_PVDLEVEL_6;
    pvd.Mode = PWR_PVD_MODE_NORMAL;
    HAL_PWR_ConfigPVD(&pvd);
    HAL_PWR_EnablePVD();
    s_vdd_low = (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != 0U);
    SystemHealth_RecordEvent(SYSTEM_EVENT_BOOT, reset_flags);
    if ((reset_flags & RCC_RSR_IWDG1RSTF) != 0U) {
        SystemHealth_RecordEvent(SYSTEM_EVENT_WATCHDOG_RESET, reset_flags);
    }
    if (s_vdd_low) {
        SystemHealth_RecordEvent(SYSTEM_EVENT_VDD_LOW,
                                 HEALTH_PVD_THRESHOLD_MV);
    }
    return true;
}

void SystemHealth_RuntimeInit(void)
{
    const uintptr_t stack_start = (uintptr_t)&_sstack;
    const uintptr_t stack_limit = (uintptr_t)&_estack;
    uintptr_t fill_end = (uintptr_t)__get_MSP();
    uint32_t *word;

    /* _sstack/_estack are linker-defined addresses, not C array objects.
     * Keep range arithmetic in uintptr_t so an optimizing compiler does not
     * interpret it as out-of-bounds arithmetic on a one-word C object. */
    if (fill_end > stack_limit) {
        fill_end = stack_limit;
    }
    if (fill_end > (stack_start + HEALTH_STACK_GUARD_BYTES)) {
        fill_end -= HEALTH_STACK_GUARD_BYTES;
        for (word = (uint32_t *)stack_start;
             (uintptr_t)word < fill_end;
             word = (uint32_t *)((uintptr_t)word + sizeof(*word))) {
            *word = HEALTH_STACK_PATTERN;
        }
    } else {
        fill_end = stack_start;
    }
    s_stack_min_free_bytes = (uint32_t)(fill_end - stack_start);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void SystemHealth_LoopBegin(void)
{
    s_loop_started_cycles = DWT->CYCCNT;
}

void SystemHealth_LoopEnd(void)
{
    const uint32_t cycles = DWT->CYCCNT - s_loop_started_cycles;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    s_loop_last_us = cycles_per_us == 0U ? 0U : cycles / cycles_per_us;
    if (s_loop_last_us > s_loop_max_us) {
        s_loop_max_us = s_loop_last_us;
    }
    if (s_loop_samples == 0U) {
        s_loop_average_us = s_loop_last_us;
    } else {
        s_loop_average_us = (s_loop_average_us * 15U + s_loop_last_us) / 16U;
    }
    if (s_loop_samples < UINT32_MAX) {
        ++s_loop_samples;
    }
}

void SystemHealth_Task(void)
{
    uintptr_t address = (uintptr_t)&_sstack;
    const uintptr_t stack_end = (uintptr_t)&_estack;
    uint32_t free_bytes = 0U;
    const bool vdd_low = (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != 0U);

    while ((address < stack_end)
           && (*(const uint32_t *)address == HEALTH_STACK_PATTERN)) {
        free_bytes += sizeof(uint32_t);
        address += sizeof(uint32_t);
    }
    if ((s_stack_min_free_bytes == 0U) || (free_bytes < s_stack_min_free_bytes)) {
        s_stack_min_free_bytes = free_bytes;
    }
    if (vdd_low != s_vdd_low) {
        s_vdd_low = vdd_low;
        SystemHealth_RecordEvent(vdd_low ? SYSTEM_EVENT_VDD_LOW
                                        : SYSTEM_EVENT_VDD_RECOVERED,
                                 HEALTH_PVD_THRESHOLD_MV);
    }
}

void SystemHealth_RecordErrorHandler(uint32_t detail)
{
    SystemHealth_RecordEvent(SYSTEM_EVENT_ERROR_HANDLER, detail);
}

bool SystemHealth_GetBootSnapshot(SystemHealthBootSnapshot *snapshot)
{
    if ((snapshot == NULL) || !s_initialized) {
        return false;
    }
    snapshot->boot_count = s_persistent->boot_count;
    snapshot->reset_flags = s_persistent->reset_flags;
    snapshot->watchdog_reset_count = s_persistent->watchdog_reset_count;
    snapshot->crash_count = s_persistent->crash_count;
    return true;
}

bool SystemHealth_GetRuntimeSnapshot(SystemHealthRuntimeSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    snapshot->loop_last_us = s_loop_last_us;
    snapshot->loop_max_us = s_loop_max_us;
    snapshot->loop_average_us = s_loop_average_us;
    snapshot->stack_min_free_bytes = s_stack_min_free_bytes;
    return true;
}

bool SystemHealth_GetCrashSnapshot(SystemHealthCrashSnapshot *snapshot)
{
    if ((snapshot == NULL) || !s_initialized) {
        return false;
    }
    memcpy(snapshot, (const void *)&s_persistent->crash, sizeof(*snapshot));
    return s_persistent->crash_count != 0U;
}

bool SystemHealth_GetEvent(uint32_t newest_index, SystemHealthEvent *event,
                           uint32_t *event_count)
{
    uint32_t available;
    uint32_t index;
    if ((event == NULL) || !s_initialized) {
        return false;
    }
    available = s_persistent->event_count;
    if (available > HEALTH_EVENT_CAPACITY) {
        available = HEALTH_EVENT_CAPACITY;
    }
    if (event_count != NULL) {
        *event_count = s_persistent->event_count;
    }
    if (newest_index >= available) {
        return false;
    }
    index = (s_persistent->event_write_index + HEALTH_EVENT_CAPACITY
             - 1U - newest_index) % HEALTH_EVENT_CAPACITY;
    memcpy(event, (const void *)&s_persistent->events[index], sizeof(*event));
    return true;
}

uint32_t SystemHealth_GetCapabilities(void)
{
    uint32_t capabilities = SYSTEM_CAP_IWDG | SYSTEM_CAP_BACKUP_BLACKBOX
        | SYSTEM_CAP_HARDFAULT_CAPTURE | SYSTEM_CAP_LOOP_MONITOR
        | SYSTEM_CAP_STACK_WATERMARK | SYSTEM_CAP_PVD_2V85
        | SYSTEM_CAP_DRIVE_FEEDBACK_GUARD;
#if STEERING_CALIBRATION_CONFIRMED
    capabilities |= SYSTEM_CAP_STEERING_CALIBRATED;
#endif
#if BOARD_EXTERNAL_WATCHDOG_ENABLE
    capabilities |= SYSTEM_CAP_EXTERNAL_WATCHDOG;
#endif
#if BOARD_PHYSICAL_ESTOP_ENABLE
    capabilities |= SYSTEM_CAP_PHYSICAL_ESTOP;
#endif
    return capabilities;
}

uint32_t SystemHealth_GetFirmwareVersion(void) { return FIRMWARE_VERSION_PACKED; }
uint32_t SystemHealth_GetGitHash(void) { return FIRMWARE_GIT_HASH32; }
uint32_t SystemHealth_GetConfigHash(void) { return FIRMWARE_CONFIG_HASH32; }
bool SystemHealth_IsVddLow(void) { return s_vdd_low; }

static bool valid_stack_frame(const uint32_t *frame)
{
    const uintptr_t address = (uintptr_t)frame;
    const uintptr_t end = address + 8U * sizeof(uint32_t);
    return ((address >= 0x20000000UL) && (end <= 0x20020000UL))
        || ((address >= 0x24000000UL) && (end <= 0x24080000UL))
        || ((address >= 0x30000000UL) && (end <= 0x30048000UL))
        || ((address >= 0x38000000UL) && (end <= 0x38010000UL));
}

void SystemHealth_CaptureFault(uint32_t *stack_frame, uint32_t exc_return,
                               uint32_t fault_type)
{
    if (s_initialized) {
        volatile SystemHealthCrashSnapshot *crash = &s_persistent->crash;
        if (valid_stack_frame(stack_frame)) {
            crash->stacked_r0 = stack_frame[0];
            crash->stacked_r1 = stack_frame[1];
            crash->stacked_r2 = stack_frame[2];
            crash->stacked_r3 = stack_frame[3];
            crash->stacked_r12 = stack_frame[4];
            crash->stacked_lr = stack_frame[5];
            crash->stacked_pc = stack_frame[6];
            crash->stacked_xpsr = stack_frame[7];
        } else {
            volatile uint32_t *word = (volatile uint32_t *)crash;
            unsigned index;
            for (index = 0U; index < 8U; ++index) {
                word[index] = 0U;
            }
        }
        crash->exc_return = exc_return;
        crash->fault_type = fault_type;
        crash->cfsr = SCB->CFSR;
        crash->hfsr = SCB->HFSR;
        crash->mmfar = SCB->MMFAR;
        crash->bfar = SCB->BFAR;
        crash->afsr = SCB->AFSR;
        ++s_persistent->crash_count;
        commit_persistent();
        SystemHealth_RecordEvent(SYSTEM_EVENT_CPU_FAULT, fault_type);
    }
    __disable_irq();
    __DSB();
    NVIC_SystemReset();
    for (;;) {
    }
}
