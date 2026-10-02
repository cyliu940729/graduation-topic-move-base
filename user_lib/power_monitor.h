#ifndef __POWER_MONITOR_H
#define __POWER_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    POWER_MONITOR_STATUS_OK = 0,
    POWER_MONITOR_STATUS_ADC_ERROR
} PowerMonitor_Status_t;

PowerMonitor_Status_t PowerMonitor_Init(void);

PowerMonitor_Status_t PowerMonitor_ReadPercentage( uint8_t *battery_percentage );

#ifdef __cplusplus
}
#endif

#endif /* __POWER_MONITOR_H */
