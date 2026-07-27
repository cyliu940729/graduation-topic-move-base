#ifndef __MECANUM_KINEMATICS_H
#define __MECANUM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define MECANUM_WHEEL_COUNT    4U


/*
 * Mecanum kinematics calculation result.
 */
typedef enum
{
    MECANUM_KINEMATICS_STATUS_OK = 0,
    MECANUM_KINEMATICS_STATUS_INVALID_ARGUMENT,
    MECANUM_KINEMATICS_STATUS_INVALID_DIRECTION
} MecanumKinematics_Status_t;


/*
 * Convert a discrete chassis direction command into
 * four signed wheel target RPM values.
 *
 * Supported directions:
 *
 *     1: Forward
 *     2: Backward
 *     3: Lateral direction A
 *     4: Lateral direction B
 *     5: Rotation direction A
 *     6: Rotation direction B
 *
 * target_rpm order:
 *
 *     target_rpm[0]: Motor 1
 *     target_rpm[1]: Motor 2
 *     target_rpm[2]: Motor 3
 *     target_rpm[3]: Motor 4
 */
MecanumKinematics_Status_t
MecanumKinematics_CalculateDirectionTargets(
    uint8_t direction,
    float speed_rpm,
    float target_rpm[MECANUM_WHEEL_COUNT]
);


#ifdef __cplusplus
}
#endif

#endif /* __MECANUM_KINEMATICS_H */
