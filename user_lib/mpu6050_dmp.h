#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_DMP_DEFAULT_ADDRESS  0x68U
#define MPU6050_DMP_PACKET_SIZE      42U

typedef enum
{
    MPU6050_DMP_OK = 0,
    MPU6050_DMP_NO_DATA = 1,
    MPU6050_DMP_I2C_ERROR = 2,
    MPU6050_DMP_FIRMWARE_ERROR = 3,
    MPU6050_DMP_VERIFY_ERROR = 4,
    MPU6050_DMP_FIFO_OVERFLOW = 5,
    MPU6050_DMP_INVALID_ARGUMENT = 6,
    MPU6050_DMP_NOT_INITIALIZED = 7
} MPU6050_DMP_Status_t;

typedef struct
{
    float qw;
    float qx;
    float qy;
    float qz;

    float yaw_rad;
    float pitch_rad;
    float roll_rad;

    float yaw_deg;
    float pitch_deg;
    float roll_deg;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    uint16_t fifo_count_before_read;
} MPU6050_DMP_Data_t;

/*
 * Load MotionApps 2.0 firmware and prepare the 42-byte DMP FIFO packet.
 * This function leaves the DMP disabled. Call MPU6050_DMP_Start() next.
 */
MPU6050_DMP_Status_t MPU6050_DMP_Init(I2C_HandleTypeDef *hi2c,
                                      uint8_t address_7bit);

/* Enable FIFO and DMP output. */
MPU6050_DMP_Status_t MPU6050_DMP_Start(void);

/* Disable DMP and FIFO. */
MPU6050_DMP_Status_t MPU6050_DMP_Stop(void);

/* Clear FIFO contents. */
MPU6050_DMP_Status_t MPU6050_DMP_ResetFIFO(void);

/*
 * Read the newest complete 42-byte packet.
 * Returns MPU6050_DMP_NO_DATA until a packet is available.
 */
MPU6050_DMP_Status_t MPU6050_DMP_Read(MPU6050_DMP_Data_t *data);

/* Current FIFO byte count. Returns 0 when the I2C read fails. */
uint16_t MPU6050_DMP_GetFIFOCount(void);

/* Last STM32 HAL I2C status returned by this driver. */
HAL_StatusTypeDef MPU6050_DMP_GetLastHALStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_DMP_H */
