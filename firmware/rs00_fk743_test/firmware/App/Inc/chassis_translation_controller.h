#ifndef CHASSIS_TRANSLATION_CONTROLLER_H
#define CHASSIS_TRANSLATION_CONTROLLER_H

#include "chassis_direction.h"

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_SPEED_DEADBAND_MPS             0.001f
#define CHASSIS_DIRECTION_DEADBAND_DEG         0.5f
#define CHASSIS_COMMAND_TIMEOUT_MS             300U
#define CHASSIS_AUTO_ENABLE_AFTER_STARTUP      0
#define CHASSIS_ENABLE_ENDPOINT_APPROXIMATION  0
#define DIRECTION_ENDPOINT_EPSILON_DEG         0.5f

/* Steering-axis centers form a 330 mm square. */
#define CHASSIS_WHEELBASE_M                    0.330f
#define CHASSIS_TRACK_WIDTH_M                  0.330f
#define CHASSIS_ROTATION_RADIUS_M              0.23334524f
#define CHASSIS_MAX_ABS_ANGULAR_SPEED_RAD_S    1.0f

typedef enum {
    CHASSIS_TRANSLATION_IDLE = 0,
    CHASSIS_TRANSLATION_STOPPING_DRIVE,
    CHASSIS_TRANSLATION_STEERING,
    CHASSIS_TRANSLATION_WAIT_ALIGNMENT,
    CHASSIS_TRANSLATION_DRIVING,
    CHASSIS_TRANSLATION_TIMEOUT_STOP,
    CHASSIS_TRANSLATION_FAULT
} ChassisTranslationState;

typedef struct {
    ChassisTranslationState state;
    float requested_direction_deg;
    float normalized_direction_deg;
    float requested_speed_mps;
    float signed_speed_mps;
    float logical_steering_angle_rad;
    float motor_target_rad[4];
    float motor_actual_rad[4];
    float motor_error_rad[4];
    float motor_velocity_rad_s[4];
    int8_t drive_direction;
    float drive_target_mps[4];
    bool steering_aligned[4];
    bool all_steering_aligned;
    bool all_drive_stopped;
    uint32_t last_command_tick;
    uint32_t state_enter_tick;
    int32_t last_error;
    uint8_t fault_wheel;
} ChassisTranslationDebugState;

void ChassisTranslation_Init(void);
void ChassisTranslation_Task(void);
bool ChassisTranslation_CommandDirection(float direction_deg, float speed_mps);
bool ChassisTranslation_CommandVelocity(float vx_mps, float vy_mps);
bool ChassisTranslation_CommandTwist(float vx_mps, float vy_mps,
                                     float wz_rad_s);
void ChassisTranslation_Stop(void);
void ChassisTranslation_EmergencyStop(void);
bool ChassisTranslation_IsMoving(void);
ChassisTranslationState ChassisTranslation_GetState(void);
bool ChassisTranslation_GetLastSolution(TranslationSolution *solution);
bool ChassisTranslation_GetDebugSnapshot(ChassisTranslationDebugState *snapshot);

#endif
