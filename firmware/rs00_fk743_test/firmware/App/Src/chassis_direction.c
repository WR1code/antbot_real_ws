#include "chassis_direction.h"
#include "steering_config.h"

#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)

bool ChassisDirection_Solve(float direction_deg, float speed_mps,
                            TranslationSolution *solution)
{
    float theta;
    float mechanical_direction;
    float magnitude;

    if ((solution == NULL) || !isfinite(direction_deg)
        || !isfinite(speed_mps)) {
        return false;
    }

    theta = fmodf(direction_deg, 360.0f);
    if (theta < 0.0f) {
        theta += 360.0f;
    }
    if (theta >= 360.0f) {
        theta = 0.0f;
    }
    magnitude = fabsf(speed_mps);

    solution->normalized_direction_deg = theta;
    mechanical_direction =
        (theta - CHASSIS_DRIVE_DIRECTION_AT_STEERING_ZERO_DEG)
        / CHASSIS_DIRECTION_PER_STEERING_DEG;
    mechanical_direction = fmodf(mechanical_direction, 360.0f);
    if (mechanical_direction < 0.0f) {
        mechanical_direction += 360.0f;
    }

    if (mechanical_direction <= 180.0f) {
        solution->steering_angle_deg = mechanical_direction;
        solution->drive_direction = 1;
        solution->signed_speed_mps = (magnitude == 0.0f) ? 0.0f : magnitude;
    } else {
        solution->steering_angle_deg = mechanical_direction - 180.0f;
        solution->drive_direction = -1;
        solution->signed_speed_mps = (magnitude == 0.0f) ? 0.0f : -magnitude;
    }
    solution->steering_angle_rad =
        solution->steering_angle_deg * DEG_TO_RAD;
    return true;
}
