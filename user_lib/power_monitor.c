#include "power_monitor.h"

#include "adc.h"

#define POWER_MONITOR_ADC_MAX_VALUE          4095.0f
#define POWER_MONITOR_ADC_REFERENCE_VOLTAGE  3.3f
#define POWER_MONITOR_ADC_TIMEOUT_MS         10U

#define POWER_MONITOR_R_TOP_OHM              27000.0f
#define POWER_MONITOR_R_BOTTOM_OHM           10000.0f

PowerMonitor_Status_t PowerMonitor_Init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        return POWER_MONITOR_STATUS_ADC_ERROR;
    }

    return POWER_MONITOR_STATUS_OK;
}


PowerMonitor_Status_t PowerMonitor_ReadPercentage(
    uint8_t *battery_percentage
)
{
    uint32_t adc_raw;
    float adc_voltage;
    float battery_voltage;
    float percentage;

    if (battery_percentage == NULL)
    {
        return POWER_MONITOR_STATUS_ADC_ERROR;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return POWER_MONITOR_STATUS_ADC_ERROR;
    }

    if (HAL_ADC_PollForConversion(
        &hadc1,
        POWER_MONITOR_ADC_TIMEOUT_MS
    ) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);

        return POWER_MONITOR_STATUS_ADC_ERROR;
    }

    adc_raw = HAL_ADC_GetValue(&hadc1);

    (void)HAL_ADC_Stop(&hadc1);

    adc_voltage =
        ((float)adc_raw /
         POWER_MONITOR_ADC_MAX_VALUE) *
        POWER_MONITOR_ADC_REFERENCE_VOLTAGE;

    battery_voltage =
        adc_voltage *
        ((POWER_MONITOR_R_TOP_OHM +
          POWER_MONITOR_R_BOTTOM_OHM) /
         POWER_MONITOR_R_BOTTOM_OHM);

    percentage =
        (battery_voltage - 11.6f) * 100.0f;

    if (percentage > 100.0f)
    {
        percentage = 100.0f;
    }
    else if (percentage < 0.0f)
    {
        percentage = 0.0f;
    }

    *battery_percentage =
        (uint8_t)(percentage + 0.5f);

    return POWER_MONITOR_STATUS_OK;
}
