#include "motion_control.h"

#include <math.h>
#include <stddef.h>

#include "encoder.h"
#include "wheel_control.h"
#include "chassis_config.h"


#define MOTION_CONTROL_WHEEL_COUNT    4U


/*
 * Current motion execution state.
 */
static volatile uint8_t motion_running = 0U;


/*
 * Current chassis direction command.
 */
static volatile int motion_direction = 0;


/*
 * Current target distance or target angle.
 */
static volatile float motion_target_value = 0.0f;


/*
 * Current average wheel travel distance.
 */
static volatile float motion_distance_cm = 0.0f;


/*
 * Current calculated chassis rotation angle.
 */
static volatile float motion_angle_deg = 0.0f;

/*
 * Maximum speed requested for the current movement.
 */
static float motion_max_speed_rpm = 0.0f;

/*
 * Current motion-profile speed in wheel linear speed.
 */
static float motion_profile_speed_cm_s = 0.0f;

/*
 * Target wheel travel distance used by the motion profile.
 */
static float motion_target_wheel_distance_cm = 0.0f;

/*
 * Internal function declarations.
 */
static float MotionControl_UpdateProfileSpeed(
    float remaining_distance_cm,
    float control_dt
);
/*
 * Calculate the average absolute travel distance
 * of all four wheels.
 */
static float MotionControl_GetAverageWheelDistance(void)
{
    float distance_sum = 0.0f;

    for (uint8_t device = 1U; device <= MOTION_CONTROL_WHEEL_COUNT; device++)
    {
        distance_sum += fabsf( Encoder_Get_Distance_cm(device) );
    }

    return distance_sum / (float)MOTION_CONTROL_WHEEL_COUNT;
}

/*
 * Convert the requested chassis movement into
 * the equivalent wheel travel distance.
 */
static float MotionControl_GetTargetWheelDistance( int direction, float target_value)
{
    if ((direction >= 1) && (direction <= 4))
    {
        return fabsf(target_value);
    }

    if ((direction == 5) || (direction == 6))
    {
        return WheelDistance_From_CarAngle( fabsf(target_value) );
    }

    return 0.0f;
}

/**
 * @brief Initialize the motion control module.
 */
void MotionControl_Init(void)
{
    motion_running = 0U;
    motion_direction = 0;
    motion_target_value = 0.0f;
    motion_distance_cm = 0.0f;
    motion_angle_deg = 0.0f;

    motion_max_speed_rpm = 0.0f;
    motion_profile_speed_cm_s = 0.0f;
    motion_target_wheel_distance_cm = 0.0f;

    WheelControl_Stop();
}


/**
 * @brief Start a new chassis movement.
 */
MotionControl_StartResult_t MotionControl_Start(
    int direction,
    float target_value,
    float speed_rpm
)
{
    WheelControl_Status_t wheel_status;

    /*
     * Stop the previous movement before loading
     * a new motion command.
     */
    WheelControl_Stop();
    motion_running = 0U;

    motion_direction = direction;
    motion_target_value = fabsf(target_value);
    motion_distance_cm = 0.0f;
    motion_angle_deg = 0.0f;

    motion_max_speed_rpm = fabsf(speed_rpm);
    motion_profile_speed_cm_s = 0.0f;

    motion_target_wheel_distance_cm =
        MotionControl_GetTargetWheelDistance(
            motion_direction,
            motion_target_value
        );

    if ((motion_direction < 1) || (motion_direction > 7))
    {
        return MOTION_CONTROL_START_INVALID_DIRECTION;
    }

    /*
     * Direction 7 is an immediate stop command.
     */
    if (motion_direction == 7)
    {
        return MOTION_CONTROL_START_STOPPED;
    }

    /*
     * Reset encoder counters, distance values,
     * and RPM filter states for the new movement.
     */
    Encoder_Init();

    wheel_status = WheelControl_SetDirection( (uint8_t)motion_direction, 0.0f );

    if (wheel_status != WHEEL_CONTROL_STATUS_OK)
    {
        WheelControl_Stop();

        return MOTION_CONTROL_START_WHEEL_ERROR;
    }

    motion_running = 1U;

    return MOTION_CONTROL_START_STARTED;
}


/**
 * @brief Update the current movement.
 */
MotionControl_UpdateResult_t MotionControl_Update(
    float control_dt
)
{
    uint8_t movement_finished = 0U;

    float remaining_distance_cm;
    float target_speed_rpm;

    /*
     * Preserve the original behavior by updating
     * all encoders every control period, even when idle.
     */
    Encoder_Update_All();

    if (motion_running == 0U)
    {
        return MOTION_CONTROL_UPDATE_IDLE;
    }

    motion_distance_cm =
        MotionControl_GetAverageWheelDistance();

    motion_angle_deg =
        CarAngle_From_WheelDistance(
            motion_distance_cm
        );

    remaining_distance_cm = motion_target_wheel_distance_cm - motion_distance_cm;

    if (remaining_distance_cm < 0.0f)
    {
        remaining_distance_cm = 0.0f;
    }

    /*
     * Directions 1 to 4 use distance as the target.
     */
    if ((motion_direction >= 1) && (motion_direction <= 4))
    {
        if (motion_distance_cm >= motion_target_value)
        {
            movement_finished = 1U;
        }
    }

    /*
     * Directions 5 and 6 use rotation angle
     * as the target.
     */
    else if ((motion_direction == 5) || (motion_direction == 6))
    {
        if (motion_angle_deg >= motion_target_value)
        {
            movement_finished = 1U;
        }
    }

    if (movement_finished == 1U)
    {
        WheelControl_Stop();
        motion_running = 0U;
        motion_profile_speed_cm_s = 0.0f;

        return MOTION_CONTROL_UPDATE_COMPLETED;
    }

    /*
     * Run one closed-loop speed update
     * for all four wheels.
     */
    motion_profile_speed_cm_s =
        MotionControl_UpdateProfileSpeed(
            remaining_distance_cm,
            control_dt
        );

    target_speed_rpm =
        WheelRPM_From_LinearSpeed(
            motion_profile_speed_cm_s
        );

    (void)WheelControl_SetDirection(
        (uint8_t)motion_direction,
        target_speed_rpm
    );

    WheelControl_RunSpeedPID(control_dt);

    return MOTION_CONTROL_UPDATE_RUNNING;
}


/**
 * @brief Stop the current movement immediately.
 */
void MotionControl_Stop(void)
{
    WheelControl_Stop();
    motion_running = 0U;
    motion_profile_speed_cm_s = 0.0f;
}


/**
 * @brief Copy the current motion information.
 */
void MotionControl_GetInfo(
    MotionControl_Info_t *info
)
{
    if (info == NULL)
    {
        return;
    }

    info->running = motion_running;
    info->direction = motion_direction;
    info->target_value = motion_target_value;
    info->traveled_distance_cm = motion_distance_cm;
    info->rotated_angle_deg = motion_angle_deg;
}

/*
 * Update the trapezoidal motion-profile speed.
 *
 * The acceleration limit controls how quickly the
 * target speed may increase.
 *
 * The braking-speed limit is calculated from:
 *
 *     v^2 = 2 * a * d
 *
 * so the vehicle begins decelerating early enough
 * to approach zero speed at the target position.
 */
static float MotionControl_UpdateProfileSpeed( float remaining_distance_cm, float control_dt )
{
    float max_speed_cm_s;
    float acceleration_limited_speed;
    float braking_limited_speed;
    float next_speed;

    if ((remaining_distance_cm <= 0.0f) || (control_dt <= 0.0f))
    {
        return 0.0f;
    }

    max_speed_cm_s = fabsf( WheelLinearSpeed_From_RPM( motion_max_speed_rpm ) );

    acceleration_limited_speed = motion_profile_speed_cm_s + (CHASSIS_ACCELERATION_CM_S2 * control_dt);

    braking_limited_speed = sqrtf( 2.0f * CHASSIS_DECELERATION_CM_S2 * remaining_distance_cm );

    next_speed = max_speed_cm_s;

    if (acceleration_limited_speed < next_speed)
    {
        next_speed = acceleration_limited_speed;
    }

    if (braking_limited_speed < next_speed)
    {
        next_speed = braking_limited_speed;
    }

    if (next_speed < 0.0f)
    {
        next_speed = 0.0f;
    }

    return next_speed;
}
