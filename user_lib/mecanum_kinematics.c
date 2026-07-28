#include "mecanum_kinematics.h"

#include <stddef.h>


#define MECANUM_WHEEL_PATTERN(M1, M2, M3, M4) \
    { (M1), (M2), (M3), (M4) }

static const int8_t mecanum_direction_table
[MECANUM_DIRECTION_COUNT][MECANUM_WHEEL_COUNT] =
{
    [MECANUM_DIRECTION_FORWARD] = MECANUM_WHEEL_PATTERN( 1,  1,  1,  1),

    [MECANUM_DIRECTION_BACKWARD] = MECANUM_WHEEL_PATTERN(-1, -1, -1, -1),

    [MECANUM_DIRECTION_LATERAL_A] = MECANUM_WHEEL_PATTERN( 1, -1,  1, -1),

    [MECANUM_DIRECTION_LATERAL_B] = MECANUM_WHEEL_PATTERN(-1,  1, -1,  1),

    [MECANUM_DIRECTION_ROTATE_A] = MECANUM_WHEEL_PATTERN( 1, -1, -1,  1),

    [MECANUM_DIRECTION_ROTATE_B] = MECANUM_WHEEL_PATTERN(-1,  1,  1, -1),

	[MECANUM_DIRECTION_STOP] = MECANUM_WHEEL_PATTERN( 0,  0,  0,  0),
};

/*
 * Clear all calculated wheel target RPM values.
 */
static void MecanumKinematics_ClearTargets( float target_rpm[MECANUM_WHEEL_COUNT])
{
    for (uint8_t i = 0U; i < MECANUM_WHEEL_COUNT; i++)
    {
        target_rpm[i] = 0.0f;
    }
}


/**
 * @brief Convert a direction command into wheel target RPM values.
 */
MecanumKinematics_Status_t
MecanumKinematics_CalculateDirectionTargets( uint8_t direction, float speed_rpm, float target_rpm[MECANUM_WHEEL_COUNT])
{
    if (target_rpm == NULL)
    {
        return MECANUM_KINEMATICS_STATUS_INVALID_ARGUMENT;
    }

    if ((direction < MECANUM_DIRECTION_FORWARD) ||
        (direction > MECANUM_DIRECTION_ROTATE_B))
    {
        return MECANUM_KINEMATICS_STATUS_INVALID_DIRECTION;
    }
    /*
     * Remove any previous target values before
     * calculating a new movement pattern.
     */
    MecanumKinematics_ClearTargets(target_rpm);

    for (uint8_t i = 0U; i < MECANUM_WHEEL_COUNT; i++)
    {
        target_rpm[i] = speed_rpm * (float)mecanum_direction_table[direction][i];
    }

    return MECANUM_KINEMATICS_STATUS_OK;
}
