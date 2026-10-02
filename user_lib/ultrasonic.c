#include "ultrasonic.h"

#include "cmsis_os2.h"
#include "tim.h"

#include <stddef.h>

#define ULTRASONIC_TRIGGER_PULSE_US    10U
#define ULTRASONIC_ECHO_TIMEOUT_MS     35U
#define ULTRASONIC_US_PER_CM           58.0f

#define ULTRASONIC_WAIT_RISING         0U
#define ULTRASONIC_WAIT_FALLING        1U
#define ULTRASONIC_CAPTURE_COMPLETE    2U

typedef struct
{
    volatile uint8_t active;
    volatile uint8_t state;
    volatile uint16_t rising_capture;
    volatile uint16_t pulse_width_us;
} Ultrasonic_Capture_t;

static volatile Ultrasonic_Capture_t ultrasonic_capture[2];

static void Ultrasonic_DelayUs(uint16_t delay_us);

static void Ultrasonic_PrepareCapture(
    uint8_t capture_index,
    uint32_t channel,
    uint32_t capture_flag
);

static void Ultrasonic_HandleCapture(
    uint8_t capture_index,
    uint32_t channel
);

Ultrasonic_Status_t Ultrasonic_Init(void)
{
    HAL_GPIO_WritePin(
        TR1_GPIO_Port,
        TR1_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        TR2_GPIO_Port,
        TR2_Pin,
        GPIO_PIN_RESET
    );

    ultrasonic_capture[0].active = 0U;
    ultrasonic_capture[0].state = ULTRASONIC_WAIT_RISING;
    ultrasonic_capture[0].rising_capture = 0U;
    ultrasonic_capture[0].pulse_width_us = 0U;

    ultrasonic_capture[1].active = 0U;
    ultrasonic_capture[1].state = ULTRASONIC_WAIT_RISING;
    ultrasonic_capture[1].rising_capture = 0U;
    ultrasonic_capture[1].pulse_width_us = 0U;

    __HAL_TIM_SET_CAPTUREPOLARITY(
        &htim8,
        TIM_CHANNEL_1,
        TIM_INPUTCHANNELPOLARITY_RISING
    );

    __HAL_TIM_SET_CAPTUREPOLARITY(
        &htim8,
        TIM_CHANNEL_2,
        TIM_INPUTCHANNELPOLARITY_RISING
    );

    if (HAL_TIM_IC_Start_IT(
        &htim8,
        TIM_CHANNEL_1
    ) != HAL_OK)
    {
        return ULTRASONIC_STATUS_TIMER_ERROR;
    }

    if (HAL_TIM_IC_Start_IT(
        &htim8,
        TIM_CHANNEL_2
    ) != HAL_OK)
    {
        (void)HAL_TIM_IC_Stop_IT(
            &htim8,
            TIM_CHANNEL_1
        );

        return ULTRASONIC_STATUS_TIMER_ERROR;
    }

    return ULTRASONIC_STATUS_OK;
}

Ultrasonic_Status_t Ultrasonic_ReadDistance(
    uint8_t sensor,
    float *distance_cm
)
{
    GPIO_TypeDef *trigger_port;
    uint16_t trigger_pin;

    uint8_t capture_index;
    uint32_t channel;
    uint32_t capture_flag;

    uint32_t start_tick;
    uint16_t pulse_width_us;

    if (distance_cm == NULL)
    {
        return ULTRASONIC_STATUS_INVALID_ARGUMENT;
    }

    if (sensor == 1U)
    {
        trigger_port = TR1_GPIO_Port;
        trigger_pin = TR1_Pin;

        capture_index = 0U;
        channel = TIM_CHANNEL_1;
        capture_flag = TIM_FLAG_CC1;
    }
    else if (sensor == 2U)
    {
        trigger_port = TR2_GPIO_Port;
        trigger_pin = TR2_Pin;

        capture_index = 1U;
        channel = TIM_CHANNEL_2;
        capture_flag = TIM_FLAG_CC2;
    }
    else
    {
        return ULTRASONIC_STATUS_INVALID_ARGUMENT;
    }

    Ultrasonic_PrepareCapture(
        capture_index,
        channel,
        capture_flag
    );

    /*
     * Generate HC-SR04 trigger pulse.
     */
    HAL_GPIO_WritePin(
        trigger_port,
        trigger_pin,
        GPIO_PIN_RESET
    );

    Ultrasonic_DelayUs(2U);

    HAL_GPIO_WritePin(
        trigger_port,
        trigger_pin,
        GPIO_PIN_SET
    );

    Ultrasonic_DelayUs(
        ULTRASONIC_TRIGGER_PULSE_US
    );

    HAL_GPIO_WritePin(
        trigger_port,
        trigger_pin,
        GPIO_PIN_RESET
    );

    start_tick = HAL_GetTick();

    /*
     * Wait for the interrupt routine to capture
     * both rising and falling edges.
     */
    while (ultrasonic_capture[capture_index].state !=
           ULTRASONIC_CAPTURE_COMPLETE)
    {
        if ((HAL_GetTick() - start_tick) >=
            ULTRASONIC_ECHO_TIMEOUT_MS)
        {
            ultrasonic_capture[capture_index].active = 0U;
            ultrasonic_capture[capture_index].state =
                ULTRASONIC_WAIT_RISING;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                &htim8,
                channel,
                TIM_INPUTCHANNELPOLARITY_RISING
            );

            return ULTRASONIC_STATUS_TIMEOUT;
        }

        osDelay(1U);
    }

    pulse_width_us =
        ultrasonic_capture[capture_index].pulse_width_us;

    ultrasonic_capture[capture_index].active = 0U;

    *distance_cm =
        (float)pulse_width_us /
        ULTRASONIC_US_PER_CM;

    return ULTRASONIC_STATUS_OK;
}

static void Ultrasonic_DelayUs(uint16_t delay_us)
{
    uint16_t start_count;

    start_count =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

    while ((uint16_t)(
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim8) -
        start_count
    ) < delay_us)
    {
    }
}

static void Ultrasonic_PrepareCapture(
    uint8_t capture_index,
    uint32_t channel,
    uint32_t capture_flag
)
{
    ultrasonic_capture[capture_index].active = 0U;
    ultrasonic_capture[capture_index].state =
        ULTRASONIC_WAIT_RISING;

    ultrasonic_capture[capture_index].rising_capture = 0U;
    ultrasonic_capture[capture_index].pulse_width_us = 0U;

    __HAL_TIM_SET_CAPTUREPOLARITY(
        &htim8,
        channel,
        TIM_INPUTCHANNELPOLARITY_RISING
    );

    __HAL_TIM_CLEAR_FLAG(
        &htim8,
        capture_flag
    );

    ultrasonic_capture[capture_index].active = 1U;
}

static void Ultrasonic_HandleCapture(
    uint8_t capture_index,
    uint32_t channel
)
{
    uint16_t captured_value;

    if (ultrasonic_capture[capture_index].active == 0U)
    {
        return;
    }

    captured_value =
        (uint16_t)HAL_TIM_ReadCapturedValue(
            &htim8,
            channel
        );

    if (ultrasonic_capture[capture_index].state ==
        ULTRASONIC_WAIT_RISING)
    {
        ultrasonic_capture[capture_index].rising_capture =
            captured_value;

        ultrasonic_capture[capture_index].state =
            ULTRASONIC_WAIT_FALLING;

        __HAL_TIM_SET_CAPTUREPOLARITY(
            &htim8,
            channel,
            TIM_INPUTCHANNELPOLARITY_FALLING
        );
    }
    else if (ultrasonic_capture[capture_index].state ==
             ULTRASONIC_WAIT_FALLING)
    {
        /*
         * uint16_t subtraction also handles one timer wrap.
         */
        ultrasonic_capture[capture_index].pulse_width_us =
            (uint16_t)(
                captured_value -
                ultrasonic_capture[capture_index].rising_capture
            );

        ultrasonic_capture[capture_index].state =
            ULTRASONIC_CAPTURE_COMPLETE;

        ultrasonic_capture[capture_index].active = 0U;

        __HAL_TIM_SET_CAPTUREPOLARITY(
            &htim8,
            channel,
            TIM_INPUTCHANNELPOLARITY_RISING
        );
    }
}

void HAL_TIM_IC_CaptureCallback(
    TIM_HandleTypeDef *htim
)
{
    if (htim->Instance != TIM8)
    {
        return;
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        Ultrasonic_HandleCapture(
            0U,
            TIM_CHANNEL_1
        );
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        Ultrasonic_HandleCapture(
            1U,
            TIM_CHANNEL_2
        );
    }
}
