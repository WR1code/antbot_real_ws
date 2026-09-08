#include "chassis_translation_controller.h"
#include "chassis_debug.h"
#include "drive_controller.h"
#include "drive_config.h"
#include "steering_config.h"
#include "steering_controller.h"
#include "system_health.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_tick;
static SteeringState s_steering_state = STEERING_STATE_READY;
static bool s_can_bus_off;
static SteeringMotor s_motors[STEERING_MOTOR_COUNT];
volatile SteeringState g_steering_debug_state = STEERING_STATE_READY;
volatile SteeringError g_steering_debug_error = STEERING_ERROR_NONE;
volatile uint8_t g_steering_debug_fault_motor_id;
SteeringCanTrace g_steering_can_trace;

void SystemHealth_RecordEvent(SystemHealthEventCode code, uint32_t detail)
{
    (void)code;
    (void)detail;
}

bool rs00_fdcan_send_standard(uint16_t standard_id, const uint8_t *data,
                              uint8_t length)
{
    assert(standard_id <= 0x7FFU);
    assert(data != NULL);
    assert(length >= 1U && length <= 8U);
    return true;
}

void rs00_fdcan_debug_task(void)
{
}

bool rs00_fdcan_is_bus_off(void)
{
    return s_can_bus_off;
}

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

bool SteeringController_IsReady(void)
{
    return s_steering_state == STEERING_STATE_READY;
}

SteeringState SteeringController_GetState(void)
{
    return s_steering_state;
}

bool SteeringController_RequestEnable(void)
{
    return false;
}

bool SteeringController_IsCalibrationConfirmed(void)
{
    return true;
}

static void feed_drive_safety_feedback(void)
{
    unsigned wheel;
    for (wheel = 0U; wheel < DRIVE_WHEEL_COUNT; ++wheel) {
        const uint16_t id = (uint16_t)(DRIVE_CAN_ID_FL + wheel);
        const uint8_t fault[] = {0x0FU, 0x00U, 0x00U, 0x00U};
        const uint8_t speed[] = {0x0FU, 0x01U, 0U, 0U, 0U, 0U};
        const uint16_t nominal_voltage =
            (DRIVE_MIN_FEEDBACK_VOLTAGE_V
             + DRIVE_MAX_FEEDBACK_VOLTAGE_V) / 2U;
        const uint8_t voltage[] = {
            0x0FU, 0x04U, (uint8_t)(nominal_voltage >> 8U),
            (uint8_t)nominal_voltage
        };
        const uint8_t current[] = {0x0FU, 0x05U, 0x00U, 0x00U};
        const uint8_t temperature[] = {0x0FU, 0x07U, 0x00U, 0x19U};
        DriveController_OnCanFrame(id, fault, sizeof(fault));
        DriveController_OnCanFrame(id, speed, sizeof(speed));
        DriveController_OnCanFrame(id, voltage, sizeof(voltage));
        DriveController_OnCanFrame(id, current, sizeof(current));
        DriveController_OnCanFrame(id, temperature, sizeof(temperature));
    }
}

static void init_translation_with_safe_drive(void)
{
    ChassisTranslation_Init();
    feed_drive_safety_feedback();
    (void)DriveController_AllWheelsStopped();
    s_tick += DRIVE_STOP_FEEDBACK_STABLE_MS;
}

bool SteeringController_SetAllAngles(float fl, float fr, float rl, float rr)
{
    const float target[STEERING_MOTOR_COUNT] = {fl, fr, rl, rr};
    unsigned index;
    if (!SteeringController_IsReady()) {
        return false;
    }
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        if ((STEERING_ENABLE_MECHANICAL_LIMIT_CHECK != 0)
            && ((target[index] < STEERING_MECH_MIN_RAD)
                || (target[index] > STEERING_MECH_MAX_RAD))) {
            return false;
        }
        s_motors[index].target_position_rad = target[index];
    }
    return true;
}

bool SteeringController_GetMotorSnapshot(SteeringMotorIndex index,
                                         SteeringMotor *snapshot)
{
    if (((unsigned)index >= STEERING_MOTOR_COUNT) || (snapshot == NULL)) {
        return false;
    }
    *snapshot = s_motors[index];
    return true;
}

bool SteeringController_EvaluateAngleTravel(float chassis_angle_rad,
                                            float *max_travel_rad)
{
    float maximum = 0.0f;
    unsigned index;
    if (!isfinite(chassis_angle_rad) || (max_travel_rad == NULL)) {
        return false;
    }
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        const float travel = fabsf(remainderf(
            chassis_angle_rad - s_motors[index].position_rad,
            6.28318530717958647692f));
        if (travel > maximum) {
            maximum = travel;
        }
    }
    *max_travel_rad = maximum;
    return true;
}

void SteeringController_EmergencyStop(void)
{
    s_steering_state = STEERING_STATE_FAULT;
}

void SteeringController_StopAll(void)
{
    s_steering_state = STEERING_STATE_ARMED;
}

static float drive_target(unsigned index)
{
    float value;
    assert(DriveController_GetTarget((DriveWheelIndex)index, &value));
    return value;
}

static void align_all(void)
{
    unsigned index;
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].position_rad = s_motors[index].target_position_rad;
        s_motors[index].velocity_rad_s = 0.0f;
    }
}

static void start_and_align(float direction, float speed)
{
    unsigned index;

    assert(ChassisTranslation_CommandDirection(direction, speed));
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_STOPPING_DRIVE);
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_STEERING);
    assert(drive_target(0U) == 0.0f);
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_WAIT_ALIGNMENT);
    align_all();
    /* Quantized RS00 velocity may remain non-zero after position settles. */
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].velocity_rad_s = 0.2f;
    }
    ChassisTranslation_Task();
    s_tick += STEERING_ALIGNMENT_STABLE_MS;
    assert(ChassisTranslation_CommandDirection(direction, speed));
    ChassisTranslation_Task();
}

static void verify_periodic_refresh_during_alignment(void)
{
    unsigned index;

    init_translation_with_safe_drive();
    assert(ChassisTranslation_CommandVelocity(0.2f, 0.0f));
    ChassisTranslation_Task();
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_WAIT_ALIGNMENT);
    align_all();
    ChassisTranslation_Task();

    /* A normal 50 Hz cmd_vel refresh must not reset alignment timing. */
    for (index = 0U;
         index < (STEERING_ALIGNMENT_STABLE_MS / 20U);
         ++index) {
        s_tick += 20U;
        assert(ChassisTranslation_CommandVelocity(0.25f, 0.0f));
        assert(ChassisTranslation_GetState()
               == CHASSIS_TRANSLATION_WAIT_ALIGNMENT);
        ChassisTranslation_Task();
    }
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_DRIVING);
    assert(fabsf(drive_target(0U) - 0.25f) < 0.001f);
}

static void verify_equivalent_full_turns_align(void)
{
    static const float turns[STEERING_MOTOR_COUNT] = {
        -12.56637061435917295384f,
        6.28318530717958647692f,
        -6.28318530717958647692f,
        12.56637061435917295384f,
    };
    unsigned index;

    init_translation_with_safe_drive();
    assert(ChassisTranslation_CommandDirection(180.0f, 0.2f));
    ChassisTranslation_Task();
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_WAIT_ALIGNMENT);
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].position_rad =
            s_motors[index].target_position_rad + turns[index];
    }
    ChassisTranslation_Task();
    s_tick += STEERING_ALIGNMENT_STABLE_MS;
    assert(ChassisTranslation_CommandDirection(180.0f, 0.2f));
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_DRIVING);
    assert(fabsf(drive_target(0U) + 0.2f) < 0.001f);
}

static void verify_pending_command_timeout(void)
{
    init_translation_with_safe_drive();
    assert(ChassisTranslation_CommandVelocity(0.2f, 0.0f));
    ChassisTranslation_Task();
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_WAIT_ALIGNMENT);

    s_tick += CHASSIS_COMMAND_TIMEOUT_MS;
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_TIMEOUT_STOP);
    align_all();
    s_tick += STEERING_ALIGNMENT_STABLE_MS;
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_TIMEOUT_STOP);
    assert(drive_target(0U) == 0.0f);
}

static void verify_diagonal_tie_keeps_drive_direction(void)
{
    TranslationSolution solution;
    TranslationSolution repeated;
    float travel;
    unsigned index;

    init_translation_with_safe_drive();
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].position_rad =
            135.0f * 3.14159265358979323846f / 180.0f;
    }
    /* ROS +X/+Y is left-forward; +X/-Y is right-forward. */
    assert(ChassisTranslation_CommandVelocity(0.2f, 0.2f));
    assert(ChassisTranslation_GetLastSolution(&solution));
    assert(fabsf(solution.steering_angle_deg - 135.0f) < 0.001f);
    assert(solution.drive_direction == -1);

    /* While that target is still moving, a periodic refresh of the same
     * requested direction must not optimize again or change branches. */
    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].position_rad =
            150.0f * 3.14159265358979323846f / 180.0f;
    }
    assert(ChassisTranslation_CommandVelocity(0.25f, 0.25f));
    assert(ChassisTranslation_GetLastSolution(&repeated));
    assert(fabsf(repeated.steering_angle_rad - solution.steering_angle_rad)
           < 0.001f);
    assert(repeated.drive_direction == solution.drive_direction);

    assert(ChassisTranslation_CommandVelocity(0.2f, -0.2f));
    assert(ChassisTranslation_GetLastSolution(&solution));
    /* Both paths are 90 degrees. Prefer +90 degrees (135 -> 225) so the
     * traction direction remains negative instead of reversing. */
    assert(fabsf(solution.steering_angle_deg - 225.0f) < 0.001f);
    assert(solution.drive_direction == -1);
    assert(SteeringController_EvaluateAngleTravel(
        solution.steering_angle_rad, &travel));
    assert(travel <= (3.14159265358979323846f / 2.0f) + 0.001f);
}

static void verify_debug_state_tracks_steering_startup(void)
{
    unsigned index;

    ChassisDebug_Init();
    init_translation_with_safe_drive();
    s_steering_state = STEERING_STATE_BOOT_WAIT;
    ChassisTranslation_Task();
    assert(g_chassis_debug.chassis_state == CHASSIS_STATE_WAIT_HOMING);

    for (index = 0U; index < STEERING_MOTOR_COUNT; ++index) {
        s_motors[index].initialized = true;
    }
    s_steering_state = STEERING_STATE_ARMED;
    ChassisTranslation_Task();
    assert(g_chassis_debug.chassis_state == CHASSIS_STATE_WAIT_ENABLE);

    s_steering_state = STEERING_STATE_READY;
    ChassisTranslation_Task();
    assert(g_chassis_debug.chassis_state == CHASSIS_STATE_IDLE);
}

int main(void)
{
    TranslationSolution solution;
    unsigned index;

    memset(s_motors, 0, sizeof(s_motors));
    verify_debug_state_tracks_steering_startup();
    verify_periodic_refresh_during_alignment();
    verify_equivalent_full_turns_align();
    verify_pending_command_timeout();
    verify_diagonal_tie_keeps_drive_direction();
    init_translation_with_safe_drive();
    start_and_align(90.0f, 0.5f);
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_DRIVING);
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        assert(fabsf(drive_target(index) + 0.5f) < 0.001f);
    }

    /* A moving direction change must stop traction before steering. */
    assert(ChassisTranslation_CommandDirection(180.0f, 0.4f));
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_STOPPING_DRIVE);
    for (index = 0U; index < DRIVE_WHEEL_COUNT; ++index) {
        assert(drive_target(index) == 0.0f);
    }
    s_tick += 2000U;
    assert(ChassisTranslation_CommandDirection(180.0f, 0.4f));
    ChassisTranslation_Task();
    s_tick += DRIVE_STOP_FEEDBACK_STABLE_MS;
    ChassisTranslation_Task();
    align_all();
    ChassisTranslation_Task();
    ChassisTranslation_Task();
    s_tick += STEERING_ALIGNMENT_STABLE_MS;
    assert(ChassisTranslation_CommandDirection(180.0f, 0.4f));
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_DRIVING);
    assert(drive_target(0U) < 0.0f);

    /* Moving commands cannot latch forever. */
    s_tick += CHASSIS_COMMAND_TIMEOUT_MS;
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState()
           == CHASSIS_TRANSLATION_TIMEOUT_STOP);
    assert(drive_target(0U) == 0.0f);

    assert(ChassisTranslation_CommandVelocity(1.0f, 0.0f));
    assert(ChassisTranslation_GetLastSolution(&solution));
    assert(fabsf(solution.normalized_direction_deg) < 0.001f);
    assert(fabsf(solution.steering_angle_deg) < 0.001f);
    assert(solution.drive_direction == 1);
    assert(ChassisTranslation_CommandVelocity(0.0f, 1.0f));
    assert(ChassisTranslation_GetLastSolution(&solution));
    assert(fabsf(solution.steering_angle_deg - 270.0f) < 0.001f);
    assert(solution.drive_direction == 1);
    assert(ChassisTranslation_CommandVelocity(-1.0f, 0.0f));
    assert(ChassisTranslation_GetLastSolution(&solution));
    assert(fabsf(solution.steering_angle_deg) < 0.001f);
    assert(solution.drive_direction == -1);
    assert(ChassisTranslation_CommandVelocity(0.0f, -1.0f));
    assert(ChassisTranslation_GetLastSolution(&solution));
    assert(fabsf(solution.steering_angle_deg - 270.0f) < 0.001f);
    assert(solution.drive_direction == -1);
    assert(!ChassisTranslation_CommandVelocity(NAN, 0.0f));

    /* One wheel never aligning causes a global emergency stop. */
    s_steering_state = STEERING_STATE_READY;
    init_translation_with_safe_drive();
    assert(ChassisTranslation_CommandDirection(45.0f, 0.2f));
    ChassisTranslation_Task();
    ChassisTranslation_Task();
    align_all();
    s_motors[2].position_rad += 0.5f;
    s_tick += STEERING_ALIGNMENT_TIMEOUT_MS;
    feed_drive_safety_feedback();
    assert(ChassisTranslation_CommandDirection(45.0f, 0.2f));
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_FAULT);
    assert(drive_target(0U) == 0.0f);

    s_steering_state = STEERING_STATE_READY;
    init_translation_with_safe_drive();
    ChassisTranslation_EmergencyStop();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_FAULT);
    assert(drive_target(0U) == 0.0f);

    /* Bus-Off latches a fault and cannot resume the previous command. */
    s_steering_state = STEERING_STATE_READY;
    init_translation_with_safe_drive();
    assert(ChassisTranslation_CommandVelocity(0.2f, 0.0f));
    s_can_bus_off = true;
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_FAULT);
    s_can_bus_off = false;
    ChassisTranslation_Task();
    assert(ChassisTranslation_GetState() == CHASSIS_TRANSLATION_FAULT);
    assert(drive_target(0U) == 0.0f);

    puts("Chassis translation controller tests passed");
    return 0;
}
