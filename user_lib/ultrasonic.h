#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef enum
{
    ULTRASONIC_STATUS_OK = 0,
    ULTRASONIC_STATUS_INVALID_ARGUMENT,
    ULTRASONIC_STATUS_TIMEOUT,
    ULTRASONIC_STATUS_TIMER_ERROR
} Ultrasonic_Status_t;

Ultrasonic_Status_t Ultrasonic_Init(void);

Ultrasonic_Status_t Ultrasonic_ReadDistance(
    uint8_t sensor,
    float *distance_cm
);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H */
