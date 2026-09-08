#include "chassis_translation_controller.h"

#include "drive_controller.h"
#include "chassis_debug.h"
#include "rs00_stm32_fdcan.h"
#include "steering_config.h"
#include "steering_controller.h"
#include "stm32h7xx_hal.h"
#include "system_health.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define TWO_PI_RAD 6.28318530717958647692f
#define PI_RAD 3.14159265358979323846f
#define PATH_TIE_EPSILON_RAD 0.001f

enum {
    CHASSIS_ERROR_NONE = 0,
    CHASSIS_ERROR_DRIVE = 1,
    CHASSIS_ERROR_STEERING = 2,
    CHASSIS_ERROR_ALIGNMENT_TIMEOUT = 3
};

/*
 * Array order is fixed: FL/左上, FR/右上, RL/左下, RR/右下.
 * RS00 IDs are 1/2/3/4; matching MINI drive IDs are 5/6/7/8.
 * TODO: lift the chassis and calibrate every traction motor's forward sign.
 */
static const int8_t s_drive_install_sign[DRIVE_WHEEL_COUNT] =
    {1, 1, 1, 1};

static ChassisTranslationState s_state;
static TranslationSolution s_solution;
static ChassisTranslationDebugState s_debug;
static bool s_have_solution;
static bool s_command_pending;
static uint32_t s_alignment_stable_tick;
static bool s_alignment_timing;
static bool s_auto_enable_requested;

static ChassisDebugState debug_state_for(ChassisTranslationState state)
{
    const SteeringState steering = SteeringController_GetState();
    if (state == CHASSIS_TRANSLATION_FAULT) {
        return CHASSIS_STATE_FAULT;
    }
    if (state == CHASSIS_TRANSLATION_TIMEOUT_STOP) {
        return CHASSIS_STATE_TIMEOUT_STOP;
    }
    if (steering == STEERING_STATE_FAULT) {
        return CHASSIS_STATE_FAULT;
    }
    if (steering < STEERING_STATE_ARMED) {
        return CHASSIS_STATE_WAIT_HOMING;
    }
    if ((steering >= STEERING_STATE_ARMED)
        && (steering < STEERING_STATE_READY)) {
        return CHASSIS_STATE_WAIT_ENABLE;
    }
    if ((state == CHASSIS_TRANSLATION_STEERING)
        || (state == CHASSIS_TRANSLATION_WAIT_ALIGNMENT)
        || (state == CHASSIS_TRANSLATION_STOPPING_DRIVE)) {
        return CHASSIS_STATE_STEERING;
    }
    if (state == CHASSIS_TRANSLATION_DRIVING) {
        return CHASSIS_STATE_DRIVE;
    }
    return CHASSIS_STATE_IDLE;
}

static uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
    return now - then;
}

static uint8_t wheel_index_from_motor_id(uint8_t motor_id)
{
    static const uint8_t ids[STEERING_MOTOR_COUNT] = STEERING_MOTOR_IDS;
    uint8_t index;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if (ids[index] == motor_id) {
            return index;
        }
    }
    return 0xFFU;
}

static uint8_t first_drive_fault_wheel(void)
{
    uint8_t wheel;
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        if (DriveController_GetSafetyFlags((DriveWheelIndex)wheel) != 0U) {
            return wheel;
        }
    }
    return 0xFFU;
}

static void set_state(ChassisTranslationState state)
{
    s_state = state;
    s_debug.state = state;
    s_debug.state_enter_tick = HAL_GetTick();
    ChassisDebug_SetState(debug_state_for(state), s_debug.state_enter_tick);
}

static void enter_fault(int32_t error, uint8_t wheel)
{
    uint32_t event_detail = ((uint32_t)error & 0xFFU)
                            | ((uint32_t)wheel << 8U);
    if ((error == CHASSIS_ERROR_DRIVE) && (wheel < DRIVE_WHEEL_COUNT)) {
        event_detail |= (DriveController_GetSafetyFlags(
                             (DriveWheelIndex)wheel) & 0xFFFFU) << 16U;
    }
    DriveController_EmergencyStop();
    /*
     * A chassis alignment/drive failure is not an RS00 motor fault. Stop and
     * disarm steering, while preserving the real chassis error below.
     */
    SteeringController_StopAll();
    s_command_pending = false;
    s_debug.last_error = error;
    s_debug.fault_wheel = wheel;
    set_state(CHASSIS_TRANSLATION_FAULT);
    ChassisDebug_SetFault((error == CHASSIS_ERROR_DRIVE)
                              ? CHASSIS_FAULT_DRIVE
                              : CHASSIS_FAULT_STEERING);
    SystemHealth_RecordEvent(
        error == CHASSIS_ERROR_DRIVE ? SYSTEM_EVENT_DRIVE_SAFETY
                                     : SYSTEM_EVENT_STEERING_SAFETY,
        event_detail);
}

static float circular_difference_deg(float first, float second)
{
    float difference = fabsf(first - second);
    if (difference > 180.0f) {
        difference = 360.0f - difference;
    }
    return difference;
}

static void optimize_steering_path(TranslationSolution *next)
{
    float direct;
    float reversed;
    float direct_travel;
    float reversed_travel;
    bool use_reversed;

    if (next == NULL) {
        return;
    }

    /* A wheel axis at angle A with traction sign S is equivalent to angle
     * A+pi with sign -S. Compare both candidates against all four measured
     * steering positions, not the previous requested target. This preserves
     * the <=90-degree property even when the joystick changes direction
     * before an earlier steering command has finished. At an exact tie,
     * preserve traction direction to avoid needless drive reversal. */
    direct = next->steering_angle_rad;
    reversed = direct + PI_RAD;
    if (!SteeringController_EvaluateAngleTravel(direct, &direct_travel)
        || !SteeringController_EvaluateAngleTravel(
            reversed, &reversed_travel)) {
        return;
    }
    use_reversed =
        (reversed_travel + PATH_TIE_EPSILON_RAD < direct_travel)
        || (s_have_solution
            && (fabsf(reversed_travel - direct_travel)
             <= PATH_TIE_EPSILON_RAD)
            && (-next->drive_direction == s_solution.drive_direction));

    if (use_reversed) {
        next->steering_angle_rad = reversed;
        next->drive_direction = (int8_t)-next->drive_direction;
        next->signed_speed_mps = -next->signed_speed_mps;
    } else {
        next->steering_angle_rad = direct;
    }
    next->steering_angle_deg = next->steering_angle_rad * RAD_TO_DEG;
}

static bool command_drive(float signed_speed_mps)
{
    float wheel[DRIVE_WHEEL_COUNT];
    unsigned index;
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        wheel[index] = signed_speed_mps * (float)s_drive_install_sign[index];
    }
    if (!DriveController_SetAllWheelSpeeds(
            wheel[0], wheel[1], wheel[2], wheel[3])) {
        return false;
    }
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        s_debug.drive_target_mps[index] = wheel[index];
    }
    return true;
}

void ChassisTranslation_Init(void)
{
    memset(&s_solution, 0, sizeof(s_solution));
    memset(&s_debug, 0, sizeof(s_debug));
    s_have_solution = false;
    s_command_pending = false;
    s_alignment_timing = false;
    s_auto_enable_requested = false;
    (void)DriveController_Init();
    set_state(CHASSIS_TRANSLATION_IDLE);
}

bool ChassisTranslation_CommandDirection(float direction_deg, float speed_mps)
{
    TranslationSolution next;
    const uint32_t now = HAL_GetTick();
    bool same_requested_direction;

    if ((s_state == CHASSIS_TRANSLATION_FAULT)
        || !SteeringController_IsCalibrationConfirmed()
        || ((fabsf(speed_mps) >= CHASSIS_SPEED_DEADBAND_MPS)
            && !DriveController_IsSafetyFeedbackReady())
        || !ChassisDirection_Solve(direction_deg, speed_mps, &next)) {
        return false;
    }

    s_debug.requested_direction_deg = direction_deg;
    s_debug.requested_speed_mps = speed_mps;
    s_debug.last_command_tick = now;

    /* Detect a repeated command before optimizing its equivalent steering
     * representation. Otherwise a 20 Hz refresh can alternate between
     * A/+V and A+pi/-V while the steering motor is moving. A chosen target
     * must remain locked until the requested chassis direction changes. */
    same_requested_direction =
        s_have_solution
        && (circular_difference_deg(next.normalized_direction_deg,
                                    s_solution.normalized_direction_deg)
            < CHASSIS_DIRECTION_DEADBAND_DEG);

    /*
     * A ROS-style command source refreshes the same target periodically.
     * While stopping or aligning, such a refresh must not restart the state
     * machine or reset the 100 ms alignment timer. Only update the speed that
     * will be applied after alignment. In DRIVING it can be applied at once.
     */
    if (same_requested_direction
        && (s_state != CHASSIS_TRANSLATION_IDLE)) {
        const float magnitude = fabsf(speed_mps);
        s_solution.signed_speed_mps =
            (magnitude == 0.0f)
                ? 0.0f
                : magnitude * (float)s_solution.drive_direction;
        s_debug.requested_speed_mps = speed_mps;
        s_debug.signed_speed_mps = s_solution.signed_speed_mps;
        if (s_state == CHASSIS_TRANSLATION_DRIVING) {
            return command_drive(s_solution.signed_speed_mps);
        }
        return true;
    }

    optimize_steering_path(&next);
    s_solution = next;
    s_have_solution = true;
    s_command_pending = true;
    s_debug.normalized_direction_deg = next.normalized_direction_deg;
    s_debug.signed_speed_mps = next.signed_speed_mps;
    s_debug.logical_steering_angle_rad = next.steering_angle_rad;
    s_debug.drive_direction = next.drive_direction;
    DriveController_StopAll();
    set_state(CHASSIS_TRANSLATION_STOPPING_DRIVE);
    return true;
}

bool ChassisTranslation_CommandVelocity(float vx_mps, float vy_mps)
{
    float speed;
    float direction;
    if (!isfinite(vx_mps) || !isfinite(vy_mps)) {
        return false;
    }
    speed = sqrtf((vx_mps * vx_mps) + (vy_mps * vy_mps));
    if (speed < CHASSIS_SPEED_DEADBAND_MPS) {
        ChassisTranslation_Stop();
        return true;
    }
    direction = atan2f(vy_mps, vx_mps) * RAD_TO_DEG;
    return ChassisTranslation_CommandDirection(direction, speed);
}

static bool update_alignment(void)
{
    unsigned index;
    bool all_aligned = true;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        SteeringMotor motor;
        bool aligned;
        if (!SteeringController_GetMotorSnapshot(
                (SteeringMotorIndex)index, &motor)) {
            s_debug.fault_wheel = (uint8_t)index;
            return false;
        }
        s_debug.motor_target_rad[index] = motor.target_position_rad;
        s_debug.motor_actual_rad[index] = motor.position_rad;
        /* RS00 position feedback can report the same physical steering
         * orientation on a different full-turn branch. Compare modulo 2*pi
         * so +theta, theta-2*pi and theta+2*pi all count as aligned. */
        s_debug.motor_error_rad[index] = remainderf(
            motor.position_rad - motor.target_position_rad, TWO_PI_RAD);
        s_debug.motor_velocity_rad_s[index] = motor.velocity_rad_s;
        /*
         * RS00 velocity feedback is quantized and remained non-zero on the
         * real chassis after position had settled. Requiring position to stay
         * inside the tolerance for STEERING_ALIGNMENT_STABLE_MS rejects a
         * fast pass-through without depending on that noisy velocity value.
         */
        aligned =
            fabsf(s_debug.motor_error_rad[index])
                <= STEERING_ALIGNMENT_TOLERANCE_RAD;
        s_debug.steering_aligned[index] = aligned;
        all_aligned = all_aligned && aligned;
    }
    s_debug.all_steering_aligned = all_aligned;
    return all_aligned;
}

static void start_steering(void)
{
    const float angle = s_solution.steering_angle_rad;
    if (!SteeringController_SetAllAngles(angle, angle, angle, angle)) {
        enter_fault(CHASSIS_ERROR_STEERING, g_steering_debug_fault_motor_id);
        return;
    }
    s_command_pending = false;
    s_alignment_timing = false;
    set_state(CHASSIS_TRANSLATION_STEERING);
}

void ChassisTranslation_Task(void)
{
    const uint32_t now = HAL_GetTick();

    DriveController_Task();
    rs00_fdcan_debug_task();
    ChassisDebug_TaskTick(now);
    {
        const SteeringState steering = SteeringController_GetState();
        bool enabled = false;
        bool homed = steering >= STEERING_STATE_ARMED;
        unsigned index;
        for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
            SteeringMotor motor;
            if (SteeringController_GetMotorSnapshot(
                    (SteeringMotorIndex)index, &motor)) {
                enabled = enabled || motor.enabled;
                homed = homed && motor.initialized;
            }
        }
        ChassisDebug_SetSteering(enabled, homed,
                                 SteeringController_IsReady(),
                                 steering == STEERING_STATE_FAULT);
        /*
         * The translation state may remain IDLE while the independent
         * steering startup state advances through homing, ARMED and READY.
         * Refresh the public chassis snapshot every cycle so UART status does
         * not keep reporting the WAIT_HOMING value captured during init.
         */
        ChassisDebug_SetState(debug_state_for(s_state), now);
    }
    if (s_state == CHASSIS_TRANSLATION_FAULT) {
        return;
    }
    if (rs00_fdcan_is_bus_off()) {
        ChassisDebug_SetReject(CHASSIS_REJECT_CAN_FAULT);
        enter_fault(CHASSIS_ERROR_DRIVE, 0U);
        return;
    }
    if (SteeringController_GetState() == STEERING_STATE_FAULT) {
        enter_fault(CHASSIS_ERROR_STEERING,
                    wheel_index_from_motor_id(
                        g_steering_debug_fault_motor_id));
        return;
    }
    if (!DriveController_IsHealthy()) {
        enter_fault(CHASSIS_ERROR_DRIVE, first_drive_fault_wheel());
        return;
    }

#if CHASSIS_AUTO_ENABLE_AFTER_STARTUP
    if (!s_auto_enable_requested
        && (SteeringController_GetState() == STEERING_STATE_ARMED)) {
        s_auto_enable_requested = SteeringController_RequestEnable();
    }
#else
    (void)s_auto_enable_requested;
#endif

    s_debug.all_drive_stopped = DriveController_AllWheelsStopped();

    /*
     * A command may expire while traction is stopping or steering is still
     * aligning. Do not start moving later from a stale command.
     */
    if (((s_state == CHASSIS_TRANSLATION_STOPPING_DRIVE)
         || (s_state == CHASSIS_TRANSLATION_STEERING)
         || (s_state == CHASSIS_TRANSLATION_WAIT_ALIGNMENT))
        && (elapsed_ms(now, s_debug.last_command_tick)
            >= CHASSIS_COMMAND_TIMEOUT_MS)) {
        DriveController_StopAll();
        s_command_pending = false;
        s_alignment_timing = false;
        ++g_chassis_debug.uart_timeout_count;
        ChassisDebug_SetReject(CHASSIS_REJECT_COMM_TIMEOUT);
        DebugHook_TimeoutStop();
        set_state(CHASSIS_TRANSLATION_TIMEOUT_STOP);
        return;
    }

    switch (s_state) {
    case CHASSIS_TRANSLATION_STOPPING_DRIVE:
        DriveController_StopAll();
        if (DriveController_AllWheelsStopped()
            && SteeringController_IsReady()) {
            start_steering();
        }
        break;
    case CHASSIS_TRANSLATION_STEERING:
        set_state(CHASSIS_TRANSLATION_WAIT_ALIGNMENT);
        break;
    case CHASSIS_TRANSLATION_WAIT_ALIGNMENT:
        if (s_command_pending) {
            DriveController_StopAll();
            set_state(CHASSIS_TRANSLATION_STOPPING_DRIVE);
        } else if (update_alignment()) {
            if (!s_alignment_timing) {
                s_alignment_timing = true;
                s_alignment_stable_tick = now;
            } else if (elapsed_ms(now, s_alignment_stable_tick)
                       >= STEERING_ALIGNMENT_STABLE_MS) {
                if (fabsf(s_solution.signed_speed_mps)
                    < CHASSIS_SPEED_DEADBAND_MPS) {
                    DriveController_StopAll();
                    set_state(CHASSIS_TRANSLATION_IDLE);
                } else if (!command_drive(s_solution.signed_speed_mps)) {
                    enter_fault(CHASSIS_ERROR_DRIVE, 0U);
                } else {
                    set_state(CHASSIS_TRANSLATION_DRIVING);
                }
            }
        } else {
            s_alignment_timing = false;
            if (elapsed_ms(now, s_debug.state_enter_tick)
                >= STEERING_ALIGNMENT_TIMEOUT_MS) {
                unsigned index;
                uint8_t fault_wheel = 0U;
                for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
                    if (!s_debug.steering_aligned[index]) {
                        fault_wheel = (uint8_t)index;
                        break;
                    }
                }
                enter_fault(CHASSIS_ERROR_ALIGNMENT_TIMEOUT, fault_wheel);
            }
        }
        break;
    case CHASSIS_TRANSLATION_DRIVING:
        if (s_command_pending) {
            DriveController_StopAll();
            set_state(CHASSIS_TRANSLATION_STOPPING_DRIVE);
        } else if (elapsed_ms(now, s_debug.last_command_tick)
                   >= CHASSIS_COMMAND_TIMEOUT_MS) {
            DriveController_StopAll();
            ++g_chassis_debug.uart_timeout_count;
            ChassisDebug_SetReject(CHASSIS_REJECT_COMM_TIMEOUT);
            DebugHook_TimeoutStop();
            set_state(CHASSIS_TRANSLATION_TIMEOUT_STOP);
        }
        break;
    case CHASSIS_TRANSLATION_IDLE:
    case CHASSIS_TRANSLATION_TIMEOUT_STOP:
    default:
        break;
    }
}

void ChassisTranslation_Stop(void)
{
    DriveController_StopAll();
    s_command_pending = false;
    if (s_state != CHASSIS_TRANSLATION_FAULT) {
        set_state(CHASSIS_TRANSLATION_IDLE);
    }
}

void ChassisTranslation_EmergencyStop(void)
{
    DriveController_EmergencyStop();
    SteeringController_EmergencyStop();
    s_command_pending = false;
    s_debug.last_error = CHASSIS_ERROR_STEERING;
    s_debug.fault_wheel =
        wheel_index_from_motor_id(g_steering_debug_fault_motor_id);
    ChassisDebug_SetState(CHASSIS_STATE_EMERGENCY_STOP, HAL_GetTick());
    set_state(CHASSIS_TRANSLATION_FAULT);
    ChassisDebug_SetFault(CHASSIS_FAULT_STEERING);
}

bool ChassisTranslation_IsMoving(void)
{
    return s_state == CHASSIS_TRANSLATION_DRIVING;
}

ChassisTranslationState ChassisTranslation_GetState(void)
{
    return s_state;
}

bool ChassisTranslation_GetLastSolution(TranslationSolution *solution)
{
    if ((solution == NULL) || !s_have_solution) {
        return false;
    }
    *solution = s_solution;
    return true;
}

bool ChassisTranslation_GetDebugSnapshot(ChassisTranslationDebugState *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_debug;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}
