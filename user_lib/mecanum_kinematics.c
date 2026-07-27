#include "mecanum_kinematics.h"

#include <stddef.h>


/*
 * Clear all calculated wheel target RPM values.
 */
static void MecanumKinematics_ClearTargets(
    float target_rpm[MECANUM_WHEEL_COUNT]
)
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
MecanumKinematics_CalculateDirectionTargets(
    uint8_t direction,
    float speed_rpm,
    float target_rpm[MECANUM_WHEEL_COUNT]
)
{
    if (target_rpm == NULL)
    {
        return MECANUM_KINEMATICS_STATUS_INVALID_ARGUMENT;
    }

    /*
     * Remove any previous target values before
     * calculating a new movement pattern.
     */
    MecanumKinematics_ClearTargets(target_rpm);

    switch (direction)
    {
        case 1U:
            /*
             * Forward
             *
             * M1: +
             * M2: +
             * M3: +
             * M4: +
             */
            target_rpm[0] =  speed_rpm;
            target_rpm[1] =  speed_rpm;
            target_rpm[2] =  speed_rpm;
            target_rpm[3] =  speed_rpm;
            break;

        case 2U:
            /*
             * Backward
             *
             * M1: -
             * M2: -
             * M3: -
             * M4: -
             */
            target_rpm[0] = -speed_rpm;
            target_rpm[1] = -speed_rpm;
            target_rpm[2] = -speed_rpm;
            target_rpm[3] = -speed_rpm;
            break;

        case 3U:
            /*
             * Lateral direction A
             *
             * M1: +
             * M2: -
             * M3: +
             * M4: -
             */
            target_rpm[0] =  speed_rpm;
            target_rpm[1] = -speed_rpm;
            target_rpm[2] =  speed_rpm;
            target_rpm[3] = -speed_rpm;
            break;

        case 4U:
            /*
             * Lateral direction B
             *
             * M1: -
             * M2: +
             * M3: -
             * M4: +
             */
            target_rpm[0] = -speed_rpm;
            target_rpm[1] =  speed_rpm;
            target_rpm[2] = -speed_rpm;
            target_rpm[3] =  speed_rpm;
            break;

        case 5U:
            /*
             * Rotation direction A
             *
             * M1: +
             * M2: -
             * M3: -
             * M4: +
             */
            target_rpm[0] =  speed_rpm;
            target_rpm[1] = -speed_rpm;
            target_rpm[2] = -speed_rpm;
            target_rpm[3] =  speed_rpm;
            break;

        case 6U:
            /*
             * Rotation direction B
             *
             * M1: -
             * M2: +
             * M3: +
             * M4: -
             */
            target_rpm[0] = -speed_rpm;
            target_rpm[1] =  speed_rpm;
            target_rpm[2] =  speed_rpm;
            target_rpm[3] = -speed_rpm;
            break;

        default:
            return MECANUM_KINEMATICS_STATUS_INVALID_DIRECTION;
    }

    return MECANUM_KINEMATICS_STATUS_OK;
}
