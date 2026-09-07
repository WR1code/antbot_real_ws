#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include "rs00_protocol.h"
#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STEERING_MOTOR_FL = 0,
    STEERING_MOTOR_FR,
    STEERING_MOTOR_RL,
    STEERING_MOTOR_RR,
    STEERING_MOTOR_COUNT
} SteeringMotorIndex;

typedef enum {
    STEERING_STATE_IDLE = 0,
    STEERING_STATE_BOOT_WAIT,
    STEERING_STATE_STOP_SEND,
    STEERING_STATE_STOP_WAIT,
    STEERING_STATE_QUERY_UID_SEND,
    STEERING_STATE_QUERY_UID_WAIT,
    STEERING_STATE_READ_POSITION_SEND,
    STEERING_STATE_READ_POSITION_WAIT,
    STEERING_STATE_VALIDATE_POSITION,
    STEERING_STATE_SET_MODE_SEND,
    STEERING_STATE_SET_MODE_WAIT,
    STEERING_STATE_VERIFY_MODE_SEND,
    STEERING_STATE_VERIFY_MODE_WAIT,
    STEERING_STATE_SET_LIMIT_SPEED_SEND,
    STEERING_STATE_SET_LIMIT_SPEED_WAIT,
    STEERING_STATE_VERIFY_LIMIT_SPEED_SEND,
    STEERING_STATE_VERIFY_LIMIT_SPEED_WAIT,
    STEERING_STATE_SET_LIMIT_CURRENT_SEND,
    STEERING_STATE_SET_LIMIT_CURRENT_WAIT,
    STEERING_STATE_VERIFY_LIMIT_CURRENT_SEND,
    STEERING_STATE_VERIFY_LIMIT_CURRENT_WAIT,
    STEERING_STATE_SET_TIMEOUT_SEND,
    STEERING_STATE_SET_TIMEOUT_WAIT,
    STEERING_STATE_VERIFY_TIMEOUT_SEND,
    STEERING_STATE_VERIFY_TIMEOUT_WAIT,
    STEERING_STATE_PRELOAD_POSITION_SEND,
    STEERING_STATE_PRELOAD_POSITION_WAIT,
    STEERING_STATE_VERIFY_POSITION_SEND,
    STEERING_STATE_VERIFY_POSITION_WAIT,
    STEERING_STATE_ARMED,
    STEERING_STATE_ENABLE_SEND,
    STEERING_STATE_ENABLE_WAIT,
    STEERING_STATE_VERIFY_ALL,
    STEERING_STATE_READY,
    STEERING_STATE_FAULT
} SteeringState;

typedef enum {
    STEERING_ERROR_NONE = 0,
    STEERING_ERROR_ARGUMENT,
    STEERING_ERROR_FDCAN_START,
    STEERING_ERROR_TX_FIFO_FULL,
    STEERING_ERROR_HAL_TX,
    STEERING_ERROR_TIMEOUT,
    STEERING_ERROR_RESPONSE_TYPE,
    STEERING_ERROR_RESPONSE_MOTOR,
    STEERING_ERROR_PARAM_INDEX,
    STEERING_ERROR_PARAM_READ,
    STEERING_ERROR_INVALID_UID,
    STEERING_ERROR_UID_MISMATCH,
    STEERING_ERROR_INVALID_POSITION,
    STEERING_ERROR_MECHANICAL_RANGE,
    STEERING_ERROR_PARAM_VERIFY,
    STEERING_ERROR_MOTOR_FAULT,
    STEERING_ERROR_MODE,
    STEERING_ERROR_FEEDBACK_STALE,
    STEERING_ERROR_TARGET_RANGE,
    STEERING_ERROR_TARGET_STEP,
    STEERING_ERROR_OVERTEMPERATURE
} SteeringError;

typedef struct {
    const char *name;
    uint8_t motor_id;
    uint8_t expected_uid[8];
    uint8_t received_uid[8];
    bool uid_received;
    bool online;
    bool initialized;
    bool enabled;
    uint32_t fault;
    uint32_t warning;
    uint8_t mode_state;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
    float startup_position_rad;
    uint8_t startup_position_raw[4];
    float target_position_rad;
    uint32_t last_rx_tick;
    uint8_t retry_count;
    SteeringError last_error;
    uint8_t fault_raw[8];
} SteeringMotor;

typedef enum {
    STEERING_LOG_ERROR = 1,
    STEERING_LOG_WARN,
    STEERING_LOG_INFO,
    STEERING_LOG_DEBUG
} SteeringLogLevel;

typedef struct {
    volatile uint32_t last_tx_id;
    volatile uint8_t last_tx_data[8];
    volatile uint32_t last_rx_id;
    volatile uint8_t last_rx_data[8];
    volatile uint32_t tx_count;
    volatile uint32_t rx_count;
} SteeringCanTrace;

void SteeringController_Init(FDCAN_HandleTypeDef *hfdcan);
void SteeringController_Task(void);
void SteeringController_OnCanFrame(uint32_t ext_id, const uint8_t data[8]);
bool SteeringController_RequestEnable(void);
void SteeringController_StopAll(void);
void SteeringController_EmergencyStop(void);
bool SteeringController_ClearFaultAndRestart(void);
bool SteeringController_SetCspModeAndRestart(void);
bool SteeringController_SetCspLimitsAndRestart(float speed_rad_s,
                                               float current_a);
bool SteeringController_IsReady(void);
SteeringState SteeringController_GetState(void);
const SteeringMotor *SteeringController_GetMotor(SteeringMotorIndex index);
bool SteeringController_GetMotorSnapshot(SteeringMotorIndex index,
                                         SteeringMotor *snapshot);
bool SteeringController_SetMotorAngle(SteeringMotorIndex index,
                                      float chassis_angle_rad);
bool SteeringController_SetAllAngles(float fl_rad, float fr_rad,
                                     float rl_rad, float rr_rad);
bool SteeringController_IsHealthy(void);
bool SteeringController_IsCalibrationConfirmed(void);

/* Override this weak hook to route messages to an existing UART/RTT logger. */
void SteeringController_Log(SteeringLogLevel level, const char *message);

extern volatile SteeringState g_steering_debug_state;
extern volatile SteeringError g_steering_debug_error;
extern volatile uint8_t g_steering_debug_fault_motor_id;
extern volatile uint32_t
    g_steering_debug_feedback_age_ms[STEERING_MOTOR_COUNT];
extern SteeringCanTrace g_steering_can_trace;

#ifdef __cplusplus
}
#endif

#endif
