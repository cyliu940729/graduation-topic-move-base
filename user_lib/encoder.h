#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"  
#include <stdint.h>
#include <stdbool.h>


void Encoder_Init(void);
void Encoder_Update(uint8_t device);
void Encoder_Update_All(void);

float Encoder_Get_Angle(uint8_t device);
float Encoder_Get_RPM(uint8_t device);
int32_t Encoder_Get_TotalCount(uint8_t device);
float Encoder_Get_Distance_cm(uint8_t device);
float CarAngle_From_WheelDistance(float wheel_cm);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
