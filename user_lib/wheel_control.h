#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/*
 * Wheel control operation result.
 */
typedef enum
{
    WHEEL_CONTROL_STATUS_OK = 0,
    WHEEL_CONTROL_STATUS_INVALID_DIRECTION
} WheelControl_Status_t;


/*
 * Initialize the wheel control module.
 *
 * All wheel target RPM values are reset to zero.
 */
void WheelControl_Init(void);


/*
 * Convert a chassis direction command into four
 * signed wheel target RPM values.
 *
 * Supported directions:
 *
 *     1: Forward
 *     2: Backward
 *     3: Lateral direction A
 *     4: Lateral direction B
 *     5: Rotation direction A
 *     6: Rotation direction B
 */
WheelControl_Status_t WheelControl_SetDirection(
    uint8_t direction,
    float speed_rpm
);


/*
 * Run one speed PID update for all four wheels.
 *
 * This function should be called at a fixed period.
 */
void WheelControl_RunSpeedPID(float dt);


/*
 * Stop all motors and clear all target RPM values.
 */
void WheelControl_Stop(void);


#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_CONTROL_H */
