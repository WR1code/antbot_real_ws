#ifndef CHASSIS_DIRECTION_H
#define CHASSIS_DIRECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float normalized_direction_deg;
    float steering_angle_deg;
    float steering_angle_rad;
    int8_t drive_direction;
    float signed_speed_mps;
} TranslationSolution;

bool ChassisDirection_Solve(float direction_deg, float speed_mps,
                            TranslationSolution *solution);

#endif
