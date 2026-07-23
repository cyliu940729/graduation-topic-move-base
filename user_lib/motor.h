#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    /* PID internal states */
    float integral;
    float prev_measurement;
    float derivative_state;

    /* PWM output range, unit: percent */
    float output_min;
    float output_max;

    uint8_t initialized;
} PID_t;

extern PID_t motor_pid[4];

void Motor_Init(void);

void Motor_Speed_Percent(uint8_t device,
                         uint8_t percent,
                         int8_t direction);

/*
 * Execute one PID update. This function must be called every 10 ms.
 * target_rpm > 0: forward
 * target_rpm < 0: reverse
 * target_rpm = 0: stop
 * Return value: applied PWM percentage.
 */
float Motor_Speed_PID(uint8_t device,
                      float target_rpm,
                      float dt);

void Motor_Stop_All(void);

float PID_Calculate(PID_t *pid,
                    float target,
                    float current,
                    float dt);

void Motor_Set_PID(uint8_t device,
                   float kp,
                   float ki,
                   float kd);

void Motor_Get_PID(uint8_t device,
                   float *kp,
                   float *ki,
                   float *kd);

void Motor_Reset_PID(uint8_t device);
void Motor_Reset_All_PID(void);
void Motor_Reset_Derivative(uint8_t device);

float Motor_Get_Current_RPM(uint8_t device);
float Motor_Get_Output_Percent(uint8_t device);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
