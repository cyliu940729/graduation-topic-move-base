#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>


typedef enum
{
    IMU_STATUS_OK = 0,
    IMU_STATUS_NO_DATA,
    IMU_STATUS_INVALID_ARGUMENT,
    IMU_STATUS_NOT_INITIALIZED,
    IMU_STATUS_DMP_ERROR
} IMU_Status_t;


IMU_Status_t IMU_Init(I2C_HandleTypeDef *hi2c);

IMU_Status_t IMU_Update(void);

uint8_t IMU_HasValidData(void);

float IMU_GetYawDeg(void);

float IMU_GetPitchDeg(void);

float IMU_GetRollDeg(void);

int16_t IMU_GetGyroXRaw(void);
int16_t IMU_GetGyroYRaw(void);
int16_t IMU_GetGyroZRaw(void);

float IMU_GetHeadingDeg(void);
void IMU_ResetHeading(void);
uint8_t IMU_IsHealthy(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
