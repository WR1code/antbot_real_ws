#include "board_reliability_io.h"

#include "board_reliability_config.h"
#include "chassis_translation_controller.h"
#include "stm32h7xx_hal.h"
#include "system_health.h"

static bool s_estop_active;
#if BOARD_EXTERNAL_WATCHDOG_ENABLE
static uint32_t s_last_watchdog_edge_tick;
#endif

void BoardReliabilityIO_Init(void)
{
#if BOARD_EXTERNAL_WATCHDOG_ENABLE
    GPIO_InitTypeDef gpio = {0};
    BOARD_EXTERNAL_WATCHDOG_GPIO_CLOCK_ENABLE();
    gpio.Pin = BOARD_EXTERNAL_WATCHDOG_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(BOARD_EXTERNAL_WATCHDOG_GPIO_PORT,
                      BOARD_EXTERNAL_WATCHDOG_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_Init(BOARD_EXTERNAL_WATCHDOG_GPIO_PORT, &gpio);
    s_last_watchdog_edge_tick = HAL_GetTick();
#endif
#if BOARD_PHYSICAL_ESTOP_ENABLE
    {
        GPIO_InitTypeDef gpio = {0};
        BOARD_PHYSICAL_ESTOP_GPIO_CLOCK_ENABLE();
        gpio.Pin = BOARD_PHYSICAL_ESTOP_GPIO_PIN;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = BOARD_PHYSICAL_ESTOP_ACTIVE_STATE == GPIO_PIN_RESET
                        ? GPIO_PULLUP : GPIO_PULLDOWN;
        HAL_GPIO_Init(BOARD_PHYSICAL_ESTOP_GPIO_PORT, &gpio);
    }
#endif
}

void BoardReliabilityIO_PollInputs(void)
{
#if BOARD_PHYSICAL_ESTOP_ENABLE
    const bool active = HAL_GPIO_ReadPin(BOARD_PHYSICAL_ESTOP_GPIO_PORT,
        BOARD_PHYSICAL_ESTOP_GPIO_PIN) == BOARD_PHYSICAL_ESTOP_ACTIVE_STATE;
    if (active && !s_estop_active) {
        SystemHealth_RecordEvent(SYSTEM_EVENT_PHYSICAL_ESTOP, 1U);
    } else if (!active && s_estop_active) {
        SystemHealth_RecordEvent(SYSTEM_EVENT_PHYSICAL_ESTOP, 0U);
    }
    s_estop_active = active;
    if (active) {
        ChassisTranslation_EmergencyStop();
    }
#endif
}

void BoardReliabilityIO_Heartbeat(void)
{
#if BOARD_EXTERNAL_WATCHDOG_ENABLE
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_watchdog_edge_tick)
        >= BOARD_EXTERNAL_WATCHDOG_EDGE_PERIOD_MS) {
        HAL_GPIO_TogglePin(BOARD_EXTERNAL_WATCHDOG_GPIO_PORT,
                          BOARD_EXTERNAL_WATCHDOG_GPIO_PIN);
        s_last_watchdog_edge_tick = now;
    }
#endif
}

bool BoardReliabilityIO_IsEmergencyStopActive(void)
{
    return s_estop_active;
}

bool BoardReliabilityIO_HasExternalWatchdog(void)
{
#if BOARD_EXTERNAL_WATCHDOG_ENABLE
    return true;
#else
    return false;
#endif
}
