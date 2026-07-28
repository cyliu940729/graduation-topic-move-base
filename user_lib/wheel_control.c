#include "wheel_control.h"

#include "motor.h"
#include "mecanum_kinematics.h"

/*
 * Signed target RPM values for motors M1 to M4.
 *
 * Positive:
 *     Forward wheel direction.
 *
 * Negative:
 *     Reverse wheel direction.
 */
static float wheel_target_rpm[MECANUM_WHEEL_COUNT] =
{
    0.0f,
    0.0f,
    0.0f,
    0.0f
};


/*
 * Clear all wheel target RPM values.
 */
static void WheelControl_ClearTargets(void)
{
    for ( uint8_t i = 0U; i < MECANUM_WHEEL_COUNT; i++)
    {
        wheel_target_rpm[i] = 0.0f;
    }
}


/**
 * @brief Initialize the wheel control module.
 */
void WheelControl_Init(void)
{
    WheelControl_ClearTargets();
}


/**
 * @brief Convert a chassis direction into wheel target RPM values.
 */
WheelControl_Status_t WheelControl_SetDirection( uint8_t direction, float speed_rpm)
{
    MecanumKinematics_Status_t kinematics_status;

    kinematics_status = MecanumKinematics_CalculateDirectionTargets( direction, speed_rpm, wheel_target_rpm);

    if (kinematics_status != MECANUM_KINEMATICS_STATUS_OK)
    {
        WheelControl_ClearTargets();

        return WHEEL_CONTROL_STATUS_INVALID_DIRECTION;
    }

    return WHEEL_CONTROL_STATUS_OK;
}


/**
 * @brief Run one speed PID update for all four wheels.
 */
void WheelControl_RunSpeedPID(float dt)
{
    for (uint8_t i = 0U; i < MECANUM_WHEEL_COUNT;i++)
    {
        (void)Motor_Speed_PID( (uint8_t)(i + 1U), wheel_target_rpm[i], dt);
    }
}

/**
 * @brief Stop all motors and clear all target RPM values.
 */
void WheelControl_Stop(void)
{
    Motor_Stop_All();
    WheelControl_ClearTargets();
}
