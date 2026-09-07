#include "system_watchdog.h"

#include "stm32h7xx_hal.h"

#define IWDG_KEY_RELOAD              0x0000AAAAU
#define IWDG_KEY_ENABLE              0x0000CCCCU
#define IWDG_KEY_WRITE_ACCESS_ENABLE 0x00005555U
#define IWDG_PRESCALER_256_VALUE     0x00000006U
#define IWDG_RELOAD_2_SECONDS        249U
#define IWDG_UPDATE_FLAGS            (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)
#define IWDG_REGISTER_UPDATE_TIMEOUT_MS 100U

static uint32_t s_boot_reset_flags;
static bool s_initialized;

bool SystemWatchdog_Init(void)
{
    const uint32_t started_at = HAL_GetTick();

    s_boot_reset_flags = RCC->RSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

#if defined(DEBUG)
    /* Breakpoints must not look like firmware deadlocks during SWD debugging. */
    __HAL_DBGMCU_FREEZE_IWDG1();
#endif

    /* IWDG1 runs from LSI and remains alive if the CPU/system clock stalls. */
    WRITE_REG(IWDG1->KR, IWDG_KEY_ENABLE);
    WRITE_REG(IWDG1->KR, IWDG_KEY_WRITE_ACCESS_ENABLE);
    WRITE_REG(IWDG1->PR, IWDG_PRESCALER_256_VALUE);
    WRITE_REG(IWDG1->RLR, IWDG_RELOAD_2_SECONDS);

    while ((IWDG1->SR & IWDG_UPDATE_FLAGS) != 0U) {
        if ((HAL_GetTick() - started_at)
            > IWDG_REGISTER_UPDATE_TIMEOUT_MS) {
            return false;
        }
    }

    WRITE_REG(IWDG1->KR, IWDG_KEY_RELOAD);
    s_initialized = true;
    return true;
}

void SystemWatchdog_Refresh(void)
{
    if (s_initialized) {
        WRITE_REG(IWDG1->KR, IWDG_KEY_RELOAD);
    }
}

uint32_t SystemWatchdog_GetBootResetFlags(void)
{
    return s_boot_reset_flags;
}

bool SystemWatchdog_WasWatchdogReset(void)
{
    return (s_boot_reset_flags & RCC_RSR_IWDG1RSTF) != 0U;
}

void SystemWatchdog_SystemReset(void)
{
    __disable_irq();
    __DSB();
    NVIC_SystemReset();
    while (1) {
    }
}
