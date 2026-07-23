#include "motor.h"
#include "encoder.h"

#include <math.h>

/* PWM timer --------------------------------------------------------------- */
extern TIM_HandleTypeDef htim1;

#define M_TIM       htim1
#define M1_SPEED    TIM_CHANNEL_1
#define M2_SPEED    TIM_CHANNEL_2
#define M3_SPEED    TIM_CHANNEL_3
#define M4_SPEED    TIM_CHANNEL_4

/* D-term low-pass filter time constant. Kd is currently 0, but keep this
 * structure so the controller is the same as the tuning project. */
#define PID_D_FILTER_TAU_S    0.05f

/* Final measured PI parameters. */
PID_t motor_pid[4] =
{
    {
        .kp = 0.6f,
        .ki = 6.0f,
        .kd = 0.0f,
        .integral = 0.0f,
        .prev_measurement = 0.0f,
        .derivative_state = 0.0f,
        .output_min = 10.0f,
        .output_max = 100.0f,
        .initialized = 0U
    },
    {
        .kp = 0.5f,
        .ki = 5.0f,
        .kd = 0.0f,
        .integral = 0.0f,
        .prev_measurement = 0.0f,
        .derivative_state = 0.0f,
        .output_min = 10.0f,
        .output_max = 100.0f,
        .initialized = 0U
    },
    {
        .kp = 0.5f,
        .ki = 5.0f,
        .kd = 0.0f,
        .integral = 0.0f,
        .prev_measurement = 0.0f,
        .derivative_state = 0.0f,
        .output_min = 10.0f,
        .output_max = 100.0f,
        .initialized = 0U
    },
    {
        .kp = 0.5f,
        .ki = 5.0f,
        .kd = 0.0f,
        .integral = 0.0f,
        .prev_measurement = 0.0f,
        .derivative_state = 0.0f,
        .output_min = 10.0f,
        .output_max = 100.0f,
        .initialized = 0U
    }
};

static float motor_current_rpm[4] =
{
    0.0f, 0.0f, 0.0f, 0.0f
};

static float motor_output_percent[4] =
{
    0.0f, 0.0f, 0.0f, 0.0f
};

static float motor_last_target_rpm[4] =
{
    0.0f, 0.0f, 0.0f, 0.0f
};

void Motor_Init(void)
{
    (void)HAL_TIM_PWM_Start(&M_TIM, M1_SPEED);
    (void)HAL_TIM_PWM_Start(&M_TIM, M2_SPEED);
    (void)HAL_TIM_PWM_Start(&M_TIM, M3_SPEED);
    (void)HAL_TIM_PWM_Start(&M_TIM, M4_SPEED);

    Motor_Stop_All();
}

void Motor_Speed_Percent(uint8_t device,
                         uint8_t percent,
                         int8_t direction)
{
    uint32_t timer_arr;
    uint32_t motor_ccr;

    if (percent > 100U)
    {
        percent = 100U;
    }

    /* Use the real TIM1 ARR instead of hard-coding 3599. */
    timer_arr = __HAL_TIM_GET_AUTORELOAD(&M_TIM);
    motor_ccr = (timer_arr * (uint32_t)percent) / 100U;

    switch (device)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&M_TIM, M1_SPEED, motor_ccr);

            if (direction == 1)
            {
                HAL_GPIO_WritePin(M1_I1_GPIO_Port, M1_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M1_I2_GPIO_Port, M1_I2_Pin, GPIO_PIN_SET);
            }
            else if (direction == -1)
            {
                HAL_GPIO_WritePin(M1_I1_GPIO_Port, M1_I1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M1_I2_GPIO_Port, M1_I2_Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(M1_I1_GPIO_Port, M1_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M1_I2_GPIO_Port, M1_I2_Pin, GPIO_PIN_RESET);
            }
            break;

        case 2:
            __HAL_TIM_SET_COMPARE(&M_TIM, M2_SPEED, motor_ccr);

            if (direction == 1)
            {
                HAL_GPIO_WritePin(M2_I1_GPIO_Port, M2_I1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M2_I2_GPIO_Port, M2_I2_Pin, GPIO_PIN_RESET);
            }
            else if (direction == -1)
            {
                HAL_GPIO_WritePin(M2_I1_GPIO_Port, M2_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M2_I2_GPIO_Port, M2_I2_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(M2_I1_GPIO_Port, M2_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M2_I2_GPIO_Port, M2_I2_Pin, GPIO_PIN_RESET);
            }
            break;

        case 3:
            __HAL_TIM_SET_COMPARE(&M_TIM, M3_SPEED, motor_ccr);

            if (direction == 1)
            {
                HAL_GPIO_WritePin(M3_I1_GPIO_Port, M3_I1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M3_I2_GPIO_Port, M3_I2_Pin, GPIO_PIN_RESET);
            }
            else if (direction == -1)
            {
                HAL_GPIO_WritePin(M3_I1_GPIO_Port, M3_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M3_I2_GPIO_Port, M3_I2_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(M3_I1_GPIO_Port, M3_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M3_I2_GPIO_Port, M3_I2_Pin, GPIO_PIN_RESET);
            }
            break;

        case 4:
            __HAL_TIM_SET_COMPARE(&M_TIM, M4_SPEED, motor_ccr);

            if (direction == 1)
            {
                HAL_GPIO_WritePin(M4_I1_GPIO_Port, M4_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M4_I2_GPIO_Port, M4_I2_Pin, GPIO_PIN_SET);
            }
            else if (direction == -1)
            {
                HAL_GPIO_WritePin(M4_I1_GPIO_Port, M4_I1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(M4_I2_GPIO_Port, M4_I2_Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(M4_I1_GPIO_Port, M4_I1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(M4_I2_GPIO_Port, M4_I2_Pin, GPIO_PIN_RESET);
            }
            break;

        default:
            __HAL_TIM_SET_COMPARE(&M_TIM, M1_SPEED, 0U);
            __HAL_TIM_SET_COMPARE(&M_TIM, M2_SPEED, 0U);
            __HAL_TIM_SET_COMPARE(&M_TIM, M3_SPEED, 0U);
            __HAL_TIM_SET_COMPARE(&M_TIM, M4_SPEED, 0U);
            break;
    }
}

float Motor_Speed_PID(uint8_t device,
                      float target_rpm,
                      float dt)
{
    uint8_t idx;
    float current_rpm;
    float current_abs;
    float target_abs;
    float output;
    uint8_t percent;
    int8_t direction;

    if ((device < 1U) || (device > 4U) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    idx = (uint8_t)(device - 1U);

    current_rpm = Encoder_Get_RPM(device);
    motor_current_rpm[idx] = current_rpm;

    if (target_rpm > 0.0f)
    {
        direction = 1;
        target_abs = target_rpm;
    }
    else if (target_rpm < 0.0f)
    {
        direction = -1;
        target_abs = -target_rpm;
    }
    else
    {
        Motor_Reset_PID(device);
        motor_last_target_rpm[idx] = 0.0f;
        motor_output_percent[idx] = 0.0f;
        Motor_Speed_Percent(device, 0U, 0);
        return 0.0f;
    }

    /* Reset the complete PID state when direction reverses. For a setpoint
     * change in the same direction, retain I and only reset D history. */
    if ((motor_last_target_rpm[idx] * target_rpm) < 0.0f)
    {
        Motor_Reset_PID(device);
    }
    else if (fabsf(target_rpm - motor_last_target_rpm[idx]) > 0.001f)
    {
        Motor_Reset_Derivative(device);
    }

    motor_last_target_rpm[idx] = target_rpm;

    current_abs = fabsf(current_rpm);

    output = PID_Calculate(&motor_pid[idx],
                           target_abs,
                           current_abs,
                           dt);

    if (output < 0.0f)
    {
        output = 0.0f;
    }
    else if (output > 100.0f)
    {
        output = 100.0f;
    }

    percent = (uint8_t)(output + 0.5f);

    motor_output_percent[idx] = output;
    Motor_Speed_Percent(device, percent, direction);

    return output;
}

void Motor_Stop_All(void)
{
    Motor_Speed_Percent(1U, 0U, 0);
    Motor_Speed_Percent(2U, 0U, 0);
    Motor_Speed_Percent(3U, 0U, 0);
    Motor_Speed_Percent(4U, 0U, 0);

    Motor_Reset_All_PID();

    for (uint32_t i = 0U; i < 4U; i++)
    {
        motor_current_rpm[i] = 0.0f;
        motor_output_percent[i] = 0.0f;
        motor_last_target_rpm[i] = 0.0f;
    }
}

float PID_Calculate(PID_t *pid,
                    float target,
                    float current,
                    float dt)
{
    float error;
    float p_term;
    float i_term;
    float d_term;
    float raw_derivative;
    float derivative_alpha;
    float candidate_integral;
    float candidate_output;
    float output;

    if ((pid == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    error = target - current;
    p_term = pid->kp * error;

    /* Derivative on measurement avoids derivative kick on a target step. */
    if (pid->initialized == 0U)
    {
        pid->prev_measurement = current;
        pid->derivative_state = 0.0f;
        pid->initialized = 1U;
    }

    raw_derivative = -(current - pid->prev_measurement) / dt;
    derivative_alpha = dt / (PID_D_FILTER_TAU_S + dt);

    pid->derivative_state +=
        derivative_alpha *
        (raw_derivative - pid->derivative_state);

    d_term = pid->kd * pid->derivative_state;

    /* Conditional-integration anti-windup. */
    candidate_integral = pid->integral + (error * dt);

    candidate_output =
        p_term +
        (pid->ki * candidate_integral) +
        d_term;

    if (((candidate_output < pid->output_max) &&
         (candidate_output > pid->output_min)) ||
        ((candidate_output >= pid->output_max) &&
         (error < 0.0f)) ||
        ((candidate_output <= pid->output_min) &&
         (error > 0.0f)))
    {
        pid->integral = candidate_integral;
    }

    i_term = pid->ki * pid->integral;
    output = p_term + i_term + d_term;

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    pid->prev_measurement = current;

    return output;
}

void Motor_Set_PID(uint8_t device,
                   float kp,
                   float ki,
                   float kd)
{
    uint8_t idx;

    if ((device < 1U) || (device > 4U))
    {
        return;
    }

    idx = (uint8_t)(device - 1U);

    motor_pid[idx].kp = kp;
    motor_pid[idx].ki = ki;
    motor_pid[idx].kd = kd;

    Motor_Reset_PID(device);
}

void Motor_Get_PID(uint8_t device,
                   float *kp,
                   float *ki,
                   float *kd)
{
    uint8_t idx;

    if ((device < 1U) || (device > 4U))
    {
        return;
    }

    idx = (uint8_t)(device - 1U);

    if (kp != NULL)
    {
        *kp = motor_pid[idx].kp;
    }

    if (ki != NULL)
    {
        *ki = motor_pid[idx].ki;
    }

    if (kd != NULL)
    {
        *kd = motor_pid[idx].kd;
    }
}

void Motor_Reset_PID(uint8_t device)
{
    uint8_t idx;

    if ((device < 1U) || (device > 4U))
    {
        return;
    }

    idx = (uint8_t)(device - 1U);

    motor_pid[idx].integral = 0.0f;
    motor_pid[idx].prev_measurement = 0.0f;
    motor_pid[idx].derivative_state = 0.0f;
    motor_pid[idx].initialized = 0U;
}

void Motor_Reset_Derivative(uint8_t device)
{
    uint8_t idx;

    if ((device < 1U) || (device > 4U))
    {
        return;
    }

    idx = (uint8_t)(device - 1U);

    motor_pid[idx].derivative_state = 0.0f;
    motor_pid[idx].initialized = 0U;
}

void Motor_Reset_All_PID(void)
{
    for (uint8_t i = 1U; i <= 4U; i++)
    {
        Motor_Reset_PID(i);
    }
}

float Motor_Get_Current_RPM(uint8_t device)
{
    if ((device < 1U) || (device > 4U))
    {
        return 0.0f;
    }

    return motor_current_rpm[device - 1U];
}

float Motor_Get_Output_Percent(uint8_t device)
{
    if ((device < 1U) || (device > 4U))
    {
        return 0.0f;
    }

    return motor_output_percent[device - 1U];
}
