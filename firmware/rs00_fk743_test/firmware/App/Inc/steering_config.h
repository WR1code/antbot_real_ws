#ifndef STEERING_CONFIG_H
#define STEERING_CONFIG_H

#include <stdint.h>

/*
 * RS00 steering layout (top of the chassis drawing is forward):
 *   FL = 左上 = 1, FR = 右上 = 2, RL = 左下 = 3, RR = 右下 = 4.
 * Motor IDs must only be changed here.
 */
#define STEERING_HOST_ID                       0xFDU
#define STEERING_MOTOR_ID_FL                    0x01U
#define STEERING_MOTOR_ID_FR                    0x02U
#define STEERING_MOTOR_ID_RL                    0x03U
#define STEERING_MOTOR_ID_RR                    0x04U
#define STEERING_MOTOR_IDS                     \
    {STEERING_MOTOR_ID_FL, STEERING_MOTOR_ID_FR, \
     STEERING_MOTOR_ID_RL, STEERING_MOTOR_ID_RR}

/* Safe first-power-up policy: initialization ends in ARMED, never ENABLED. */
#define STEERING_AUTO_ENABLE                   0
#define STEERING_ENFORCE_UID_CHECK             0
#define STEERING_ENABLE_MECHANICAL_LIMIT_CHECK 1

/* Whole-chassis motion remains locked until UID, installation direction,
 * zero offsets and four mechanical ranges have been measured. Direct steering
 * calibration remains available but is limited to 0.01 rad per command. */
#define STEERING_CALIBRATION_CONFIRMED          0
#define STEERING_COMMISSIONING_MAX_STEP_RAD     0.01f
#define STEERING_MAX_FEEDBACK_TEMPERATURE_C     85.0f

#define STEERING_MOTOR_BOOT_DELAY_MS           500U
#define STEERING_RESPONSE_TIMEOUT_MS           100U
#define STEERING_MAX_RETRIES                   3U
#define STEERING_TARGET_PERIOD_MS              20U
#define STEERING_FEEDBACK_TIMEOUT_MS           500U
#define STEERING_FEEDBACK_CONFIRM_MS           100U
/*
 * Enabling is sequential.  Earlier motors must remain acceptable while the
 * remaining motors each use their response/retry budget.  READY keeps the
 * tighter STEERING_FEEDBACK_TIMEOUT_MS watchdog below.
 */
#define STEERING_ENABLE_SEQUENCE_TIMEOUT_MS    1700U
#define STEERING_ENABLE_MAX_SPEED_RAD_S        0.5f
#define STEERING_FLOAT_TOLERANCE               0.001f
#define STEERING_LIMIT_EPSILON_RAD              0.001f
#define STEERING_MECH_MIN_RAD                   0.0f
#define STEERING_MECH_MAX_RAD                   3.14159265358979323846f

/*
 * Measured chassis mounting convention:
 *   steering 0 deg  + positive traction = chassis left  (+Y / 90 deg)
 *   steering 90 deg + positive traction = chassis front (+X / 0 deg)
 * Therefore positive steering motion rotates the positive traction direction
 * clockwise in chassis coordinates.
 */
#define CHASSIS_DRIVE_DIRECTION_AT_STEERING_ZERO_DEG 90.0f
#define CHASSIS_DIRECTION_PER_STEERING_DEG           (-1.0f)

#define STEERING_CSP_LIMIT_SPEED_RAD_S           1.0f
#define STEERING_CSP_LIMIT_CURRENT_A             2.0f
#define STEERING_CAN_TIMEOUT_COUNTS             10000U

#define STEERING_ALIGNMENT_TOLERANCE_RAD         0.03f
#define STEERING_ALIGNMENT_STABLE_MS             200U
#define STEERING_ALIGNMENT_TIMEOUT_MS            5000U

/*
 * Installation values are placeholders. Determine direction and zero offset
 * with every wheel lifted; CAN ID does not imply a mechanical direction.
 */
#define STEERING_DIRECTION_SIGNS                {1.0f, 1.0f, 1.0f, 1.0f}
#define STEERING_ZERO_OFFSETS_RAD                {0.0f, 0.0f, 0.0f, 0.0f}

/* TODO: replace each endpoint with its measured safe mechanical range. */
#define STEERING_MIN_POSITIONS_RAD               \
    {STEERING_MECH_MIN_RAD, STEERING_MECH_MIN_RAD, \
     STEERING_MECH_MIN_RAD, STEERING_MECH_MIN_RAD}
#define STEERING_MAX_POSITIONS_RAD               \
    {STEERING_MECH_MAX_RAD, STEERING_MECH_MAX_RAD, \
     STEERING_MECH_MAX_RAD, STEERING_MECH_MAX_RAD}

/* All-zero entries mean "learn and display only" when UID enforcement is off. */
#define STEERING_EXPECTED_UIDS                   \
    {{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},        \
     {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},        \
     {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},        \
     {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}}

#if STEERING_CALIBRATION_CONFIRMED && !STEERING_ENFORCE_UID_CHECK
#error "Confirmed steering calibration requires UID enforcement"
#endif

/* 0=off, 1=error, 2=warn, 3=info, 4=debug. Hook is weak/no-op by default. */
#define STEERING_LOG_LEVEL                      3

#endif
