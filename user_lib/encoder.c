#include "encoder.h"

/* Encoder timers ---------------------------------------------------------- */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

/* Must match the 10 ms motor-control task period in main.c. */
#define ENCODER_DT                    0.01f
#define ENCODER_CPR  				  61440.0f
#define WHEEL_DIAMETER_CM             6.8f

#define CAR_HALF_WHEELBASE_CM         5.5f
#define CAR_HALF_TRACK_CM             9.5f
#define CAR_TURN_RADIUS_CM            \
        (CAR_HALF_WHEELBASE_CM + CAR_HALF_TRACK_CM)

#define PI_F                          3.1415926f

/* This is the same RPM filter used while tuning the final PID values. */
#define ENCODER_RPM_FILTER_ALPHA      0.25f

/* M1->TIM5, M2->TIM3, M3->TIM4, M4->TIM2 */
static TIM_HandleTypeDef *encoder_tim[4] =
{
    &htim5,
    &htim3,
    &htim4,
    &htim2
};

/* A/B phases of M2 and M4 are opposite to M1 and M3. */
static const int8_t encoder_direction[4] =
{
    -1,     /* M1 */
     1,     /* M2 */
    -1,     /* M3 */
     1      /* M4 */
};

static uint16_t encoder_last_cnt[4] =
{
    0U, 0U, 0U, 0U
};

static int32_t encoder_total_cnt[4] =
{
    0, 0, 0, 0
};

static float encoder_rpm[4] =
{
    0.0f, 0.0f, 0.0f, 0.0f
};

void Encoder_Init(void)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        (void)HAL_TIM_Encoder_Start(encoder_tim[i], TIM_CHANNEL_ALL);

        __HAL_TIM_SET_COUNTER(encoder_tim[i], 0U);

        encoder_last_cnt[i] =
            (uint16_t)__HAL_TIM_GET_COUNTER(encoder_tim[i]);

        encoder_total_cnt[i] = 0;
        encoder_rpm[i] = 0.0f;
    }
}

void Encoder_Update(uint8_t device)
{
    uint8_t idx;
    uint16_t now_cnt;
    int16_t raw_diff;
    int32_t corrected_diff;
    float raw_rpm;

    if ((device < 1U) || (device > 4U))
    {
        return;
    }

    idx = (uint8_t)(device - 1U);

    now_cnt =
        (uint16_t)__HAL_TIM_GET_COUNTER(encoder_tim[idx]);

    /* int16_t conversion handles normal 16-bit timer wrap-around. */
    raw_diff =
        (int16_t)(now_cnt - encoder_last_cnt[idx]);

    encoder_last_cnt[idx] = now_cnt;

    corrected_diff =
        (int32_t)raw_diff *
        (int32_t)encoder_direction[idx];

    encoder_total_cnt[idx] += corrected_diff;

    raw_rpm =
        ((float)corrected_diff * 60.0f) /
        (ENCODER_CPR * ENCODER_DT);

    encoder_rpm[idx] +=
        ENCODER_RPM_FILTER_ALPHA *
        (raw_rpm - encoder_rpm[idx]);
}

void Encoder_Update_All(void)
{
    Encoder_Update(1U);
    Encoder_Update(2U);
    Encoder_Update(3U);
    Encoder_Update(4U);
}

float Encoder_Get_Angle(uint8_t device)
{
    uint8_t idx;

    if ((device < 1U) || (device > 4U))
    {
        return 0.0f;
    }

    idx = (uint8_t)(device - 1U);

    return ((float)encoder_total_cnt[idx] /
            ENCODER_CPR) *
           360.0f;
}

float Encoder_Get_RPM(uint8_t device)
{
    if ((device < 1U) || (device > 4U))
    {
        return 0.0f;
    }

    return encoder_rpm[device - 1U];
}

int32_t Encoder_Get_TotalCount(uint8_t device)
{
    if ((device < 1U) || (device > 4U))
    {
        return 0;
    }

    return encoder_total_cnt[device - 1U];
}

float Encoder_Get_Distance_cm(uint8_t device)
{
    uint8_t idx;
    float wheel_circumference_cm;

    if ((device < 1U) || (device > 4U))
    {
        return 0.0f;
    }

    idx = (uint8_t)(device - 1U);
    wheel_circumference_cm = PI_F * WHEEL_DIAMETER_CM;

    return ((float)encoder_total_cnt[idx] /
            ENCODER_CPR) *
           wheel_circumference_cm;
}

float CarAngle_From_WheelDistance(float wheel_cm)
{
    return (wheel_cm / CAR_TURN_RADIUS_CM) *
           (180.0f / PI_F);
}


