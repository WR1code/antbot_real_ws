#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DRIVE_WHEEL_FL = 0,
    DRIVE_WHEEL_FR,
    DRIVE_WHEEL_RL,
    DRIVE_WHEEL_RR,
    DRIVE_WHEEL_COUNT
} DriveWheelIndex;

typedef struct {
    uint32_t valid_mask;
    uint32_t last_feedback_ms;
    uint32_t last_fault_ms;
    uint32_t last_speed_ms;
    uint32_t last_voltage_ms;
    uint32_t last_current_ms;
    uint32_t last_temperature_ms;
    uint16_t fault_bits;
    int32_t speed_erpm;
    uint16_t voltage_v;
    int16_t motor_current_10ma;
    int16_t temperature_c;
    int32_t position_centideg;
} DriveMotorFeedback;

typedef enum {
    DRIVE_SAFETY_MISSING = 1U << 0,
    DRIVE_SAFETY_STALE = 1U << 1,
    DRIVE_SAFETY_DRIVER_FAULT = 1U << 2,
    DRIVE_SAFETY_OVERCURRENT = 1U << 3,
    DRIVE_SAFETY_OVERTEMPERATURE = 1U << 4,
    DRIVE_SAFETY_VOLTAGE = 1U << 5,
    DRIVE_SAFETY_OVERSPEED = 1U << 6
} DriveSafetyFlag;

extern volatile DriveMotorFeedback g_drive_feedback[DRIVE_WHEEL_COUNT];

bool DriveController_Init(void);
void DriveController_Task(void);
bool DriveController_SetWheelSpeed(DriveWheelIndex wheel, float speed_mps);
bool DriveController_SetAllWheelSpeeds(float fl_mps, float fr_mps,
                                       float rl_mps, float rr_mps);
bool DriveController_SetAccelerationErpmS(int32_t acceleration_erpm_s);
bool DriveController_SetDecelerationErpmS(int32_t deceleration_erpm_s);
void DriveController_StopAll(void);
void DriveController_EmergencyStop(void);
bool DriveController_ClearEmergencyStopAndRestart(void);
bool DriveController_AllWheelsStopped(void);
bool DriveController_IsHealthy(void);
bool DriveController_IsSafetyFeedbackReady(void);
uint32_t DriveController_GetSafetyFlags(DriveWheelIndex wheel);
uint32_t DriveController_GetFeedbackAgeMs(DriveWheelIndex wheel);
bool DriveController_GetTarget(DriveWheelIndex wheel, float *speed_mps);
bool DriveController_IsConfigured(void);
uint32_t DriveController_GetTxErrorCount(void);
void DriveController_OnCanFrame(uint16_t standard_id, const uint8_t *data,
                                uint8_t length);
bool DriveController_GetFeedback(DriveWheelIndex wheel,
                                 DriveMotorFeedback *feedback);

#endif
