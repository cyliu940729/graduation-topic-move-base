#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/*
 * Result returned when a new motion command is started.
 */
typedef enum
{
    MOTION_CONTROL_START_STARTED = 0,
    MOTION_CONTROL_START_STOPPED,
    MOTION_CONTROL_START_INVALID_DIRECTION,
    MOTION_CONTROL_START_WHEEL_ERROR,
    MOTION_CONTROL_START_IMU_NOT_READY
} MotionControl_StartResult_t;


/*
 * Result returned by the periodic motion update.
 */
typedef enum
{
    MOTION_CONTROL_UPDATE_IDLE = 0,
    MOTION_CONTROL_UPDATE_RUNNING,
    MOTION_CONTROL_UPDATE_COMPLETED
} MotionControl_UpdateResult_t;


/*
 * Current motion control information.
 */
typedef struct
{
    uint8_t running;
    int direction;
    float target_value;
    float traveled_distance_cm;
    float rotated_angle_deg;
} MotionControl_Info_t;


/*
 * Initialize the motion control state.
 */
void MotionControl_Init(void);


/*
 * Start a new chassis movement.
 *
 * Directions:
 *
 *     1: Forward
 *     2: Backward
 *     3: Lateral direction A
 *     4: Lateral direction B
 *     5: Rotation direction A
 *     6: Rotation direction B
 *     7: Stop
 *
 * target_value:
 *
 *     Directions 1 to 4:
 *         Target distance in centimeters.
 *
 *     Directions 5 and 6:
 *         Target rotation angle in degrees.
 */
MotionControl_StartResult_t MotionControl_Start( int direction, float target_value, float speed_rpm );


/*
 * Update encoder state, check motion completion,
 * and run the four-wheel speed PID.
 *
 * This function should be called periodically.
 */
MotionControl_UpdateResult_t MotionControl_Update( float control_dt );


/*
 * Stop the current movement immediately.
 */
void MotionControl_Stop(void);


/*
 * Copy the current motion information.
 */
void MotionControl_GetInfo( MotionControl_Info_t *info );


#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CONTROL_H */
