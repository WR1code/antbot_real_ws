#include "chassis_direction.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void check(float input, float angle, int direction)
{
    TranslationSolution solution;
    assert(ChassisDirection_Solve(input, 2.0f, &solution));
    assert(fabsf(solution.steering_angle_deg - angle) < 0.001f);
    assert(solution.drive_direction == direction);
    assert((solution.signed_speed_mps > 0.0f) == (direction > 0));
}

int main(void)
{
    TranslationSolution solution;
    check(0.0f, 0.0f, 1);
    check(45.0f, 135.0f, -1);
    check(90.0f, 90.0f, -1);
    check(135.0f, 45.0f, -1);
    check(180.0f, 0.0f, -1);
    check(181.0f, 179.0f, 1);
    check(225.0f, 135.0f, 1);
    check(270.0f, 90.0f, 1);
    check(315.0f, 45.0f, 1);
    check(359.0f, 1.0f, 1);
    check(360.0f, 0.0f, 1);
    check(-90.0f, 90.0f, 1);
    check(720.0f, 0.0f, 1);
    assert(ChassisDirection_Solve(0.0f, 0.0f, &solution));
    assert(solution.signed_speed_mps == 0.0f);
    assert(!signbit(solution.signed_speed_mps));
    assert(!ChassisDirection_Solve(NAN, 1.0f, &solution));
    assert(!ChassisDirection_Solve(0.0f, INFINITY, &solution));
    assert(!ChassisDirection_Solve(0.0f, 1.0f, NULL));
    puts("Chassis direction tests passed");
    return 0;
}
