#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Chassis command settings.
 */
#define CHASSIS_COMMAND_QUEUE_SIZE           8U


/*
 * Periodic motor control settings.
 */
#define CHASSIS_CONTROL_PERIOD_MS            10U

#define CHASSIS_CONTROL_DT_S                 \
    ((float)CHASSIS_CONTROL_PERIOD_MS / 1000.0f)


/*
 * Default movement speed.
 */
#define CHASSIS_DEFAULT_SPEED_RPM            30.0f

/*
 * Motion profile acceleration limits.
 */
#define CHASSIS_ACCELERATION_CM_S2           10.0f
#define CHASSIS_DECELERATION_CM_S2           10.0f

/*
 * CMSIS-RTOS V2 task stack sizes are specified in bytes.
 */
#define CHASSIS_MOTOR_TASK_STACK_SIZE        2048U
#define CHASSIS_DEBUG_TASK_STACK_SIZE        1536U


/*
 * Debug output buffer size.
 */
#define CHASSIS_DEBUG_MESSAGE_SIZE           160U


#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_CONFIG_H */
