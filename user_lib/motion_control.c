#include "motion_control.h"

#include <math.h>
#include <stddef.h>

#include "encoder.h"
#include "wheel_control.h"


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

    if ((motion_direction < 1) ||
        (motion_direction > 7))
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

    wheel_status = WheelControl_SetDirection(
        (uint8_t)motion_direction,
        speed_rpm
    );

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

    /*
     * Directions 1 to 4 use distance as the target.
     */
    if ((motion_direction >= 1) &&
        (motion_direction <= 4))
    {
        if (motion_distance_cm >=
            motion_target_value)
        {
            movement_finished = 1U;
        }
    }

    /*
     * Directions 5 and 6 use rotation angle
     * as the target.
     */
    else if ((motion_direction == 5) ||
             (motion_direction == 6))
    {
        if (motion_angle_deg >=
            motion_target_value)
        {
            movement_finished = 1U;
        }
    }

    if (movement_finished == 1U)
    {
        WheelControl_Stop();
        motion_running = 0U;

        return MOTION_CONTROL_UPDATE_COMPLETED;
    }

    /*
     * Run one closed-loop speed update
     * for all four wheels.
     */
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
    info->traveled_distance_cm =
        motion_distance_cm;
    info->rotated_angle_deg =
        motion_angle_deg;
}
