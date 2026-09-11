#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

/*
 * OIDelec MINI standard-CAN settings.
 *
 * Layout (top of the chassis drawing is forward):
 *   FL = 左上 = 1, FR = 右上 = 2, RL = 左下 = 3, RR = 右下 = 4.
 * These four IDs must match the addresses written with the OIDelec PC tool.
 * MINI drives use the dedicated FDCAN2 bus at 500 kbit/s; RS00 steering stays
 * on FDCAN1 at 1 Mbit/s.
 */
#define DRIVE_CAN_ID_FL 0x005U
#define DRIVE_CAN_ID_FR 0x006U
#define DRIVE_CAN_ID_RL 0x007U
#define DRIVE_CAN_ID_RR 0x008U

/*
 * Conversion from chassis m/s to the protocol's electrical rpm (erpm):
 *   erpm = m/s / (2*pi*wheel_radius) * 60
 *          * motor_to_wheel_ratio * pole_pairs
 *
 * 105 mm direct-drive hub motor, 10 pole pairs, per the supplied motor data.
 */
#define DRIVE_WHEEL_RADIUS_M             0.0525f
#define DRIVE_MOTOR_TO_WHEEL_RATIO       1.0f
#define DRIVE_MOTOR_POLE_PAIRS           10.0f
#define DRIVE_ENCODER_CONFIG_COUNT        4096U
#define DRIVE_MOTOR_RATED_POWER_W         150U
#define DRIVE_MOTOR_RATED_CURRENT_A       8.0f
#define DRIVE_MOTOR_PEAK_CURRENT_A        17.0f
#define DRIVE_CONTROLLER_CONT_CURRENT_A   4.0f
#define DRIVE_CONTROLLER_MAX_CURRENT_A    8.0f
#define DRIVE_MOTOR_MAX_ERPM              10000

/* Chassis command limit. 1.50 m/s is about 2728 erpm with the parameters
 * above, safely below both the 3000 erpm feedback guard and the drive's
 * configured 10000 erpm absolute limit. */
#define DRIVE_MAX_ABS_SPEED_MPS          1.50f
#define DRIVE_ACCELERATION_MPS2          0.20f
#define DRIVE_DECELERATION_MPS2          0.30f
/* Written in the OIDelec PC tool; it is not set by a CAN motion command. */
#define DRIVE_HEARTBEAT_TIMEOUT_MS       1000U
#define DRIVE_HEARTBEAT_PERIOD_MS        100U
#define DRIVE_FAULT_FEEDBACK_PERIOD_MS   100U
#define DRIVE_SPEED_FEEDBACK_PERIOD_MS   10U
#define DRIVE_CURRENT_FEEDBACK_PERIOD_MS 10U
#define DRIVE_POSITION_FEEDBACK_PERIOD_MS 10U
#define DRIVE_TEMPERATURE_FEEDBACK_PERIOD_MS 100U
#define DRIVE_SPEED_REFRESH_PERIOD_MS    50U
#define DRIVE_TX_INTERVAL_MS             2U
/* A full hardware TX FIFO is retried every 2 ms. The installed four-drive
 * bus can remain arbitration-busy for more than 100 ms; 500 ms still trips
 * before the drives' independent 1000 ms heartbeat timeout. Bus-Off is
 * handled separately and is not delayed by this value. */
#define DRIVE_TX_FAILURE_TIMEOUT_MS       500U
#define DRIVE_STOP_SETTLE_MARGIN_MS      100U

/* Active safety feedback. One query is sent every 5 ms; a complete set for
 * four drives is refreshed in about 100 ms even when async reporting is off. */
#define DRIVE_SAFETY_QUERY_INTERVAL_MS       5U
#ifndef DRIVE_SPEED_FEEDBACK_TIMEOUT_MS
#define DRIVE_SPEED_FEEDBACK_TIMEOUT_MS    250U
#endif
#ifndef DRIVE_CURRENT_FEEDBACK_TIMEOUT_MS
#define DRIVE_CURRENT_FEEDBACK_TIMEOUT_MS  250U
#endif
#ifndef DRIVE_FAULT_FEEDBACK_TIMEOUT_MS
#define DRIVE_FAULT_FEEDBACK_TIMEOUT_MS    500U
#endif
#ifndef DRIVE_TEMPERATURE_FEEDBACK_TIMEOUT_MS
#define DRIVE_TEMPERATURE_FEEDBACK_TIMEOUT_MS 750U
#endif
/* Voltage is actively polled rather than streamed by the installed MINI
 * firmware, so tolerate several missed query/reply cycles. */
#ifndef DRIVE_VOLTAGE_FEEDBACK_TIMEOUT_MS
#define DRIVE_VOLTAGE_FEEDBACK_TIMEOUT_MS  2000U
#endif
#define DRIVE_STOP_SPEED_THRESHOLD_ERPM     30
#define DRIVE_STOP_FEEDBACK_STABLE_MS      100U
#define DRIVE_MAX_FEEDBACK_CURRENT_10MA    800
#define DRIVE_MAX_FEEDBACK_TEMPERATURE_C    85
/* Installed traction bus is nominally 48 V (verified from MINI feedback). */
#define DRIVE_MIN_FEEDBACK_VOLTAGE_V         36
#define DRIVE_MAX_FEEDBACK_VOLTAGE_V         60
#define DRIVE_MAX_FEEDBACK_SPEED_ERPM      3000
#define DRIVE_SAFE_MAX_ACCELERATION_ERPM_S 1000
#define DRIVE_SAFE_MAX_DECELERATION_ERPM_S 1500

/* Raw per-wheel speed commands bypass chassis steering sequencing and are
 * disabled for normal builds. Use cmd_vel/ChassisTranslation instead. */
#define DRIVE_ALLOW_RAW_WHEEL_COMMANDS 0

#endif
