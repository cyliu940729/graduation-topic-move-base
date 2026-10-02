#include "imu.h"

#include "mpu6050_dmp.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

static MPU6050_DMP_Data_t imu_dmp_data;

static uint8_t imu_initialized = 0U;
static uint8_t imu_data_valid = 0U;

#define IMU_RAD_TO_DEG       57.2957795131f
#define IMU_HEADING_SIGN     1.0f
#define IMU_GYRO_LSB_PER_DPS          16.4f
#define IMU_GYRO_CALIBRATION_SAMPLES  200U
#define IMU_GYRO_DEADBAND_DPS         0.3f

static float imu_heading_deg = 0.0f;

static float imu_previous_qw = 1.0f;
static float imu_previous_qx = 0.0f;
static float imu_previous_qy = 0.0f;
static float imu_previous_qz = 0.0f;
static int32_t imu_gyro_x_bias_sum = 0;
static float imu_gyro_x_bias_raw = 0.0f;
static uint16_t imu_gyro_calibration_count = 0U;
static uint8_t imu_gyro_calibrated = 0U;
static uint32_t imu_last_update_tick = 0U;


static uint8_t imu_heading_initialized = 0U;

IMU_Status_t IMU_Init(I2C_HandleTypeDef *hi2c)
{
    MPU6050_DMP_Status_t dmp_status;

    if (hi2c == NULL)
    {
        return IMU_STATUS_INVALID_ARGUMENT;
    }

    memset(&imu_dmp_data, 0, sizeof(imu_dmp_data));

    imu_initialized = 0U;
    imu_data_valid = 0U;
    imu_heading_deg = 0.0f;
    imu_heading_initialized = 0U;
    imu_gyro_x_bias_sum = 0;
    imu_gyro_x_bias_raw = 0.0f;
    imu_gyro_calibration_count = 0U;
    imu_gyro_calibrated = 0U;
    imu_last_update_tick = 0U;

    dmp_status = MPU6050_DMP_Init( hi2c, MPU6050_DMP_DEFAULT_ADDRESS );

    if (dmp_status != MPU6050_DMP_OK)
    {
        return IMU_STATUS_DMP_ERROR;
    }

    dmp_status = MPU6050_DMP_Start();

    if (dmp_status != MPU6050_DMP_OK)
    {
        return IMU_STATUS_DMP_ERROR;
    }

    imu_initialized = 1U;

    return IMU_STATUS_OK;
}


IMU_Status_t IMU_Update(void)
{
    MPU6050_DMP_Status_t dmp_status;

    if (imu_initialized == 0U)
    {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    dmp_status = MPU6050_DMP_Read(&imu_dmp_data);

    if (dmp_status == MPU6050_DMP_NO_DATA)
    {
        return IMU_STATUS_NO_DATA;
    }

    if (dmp_status != MPU6050_DMP_OK)
    {
        return IMU_STATUS_DMP_ERROR;
    }

    /*
     * Calibrate the sensor X gyro while the chassis is stationary.
     * Sensor X corresponds to the chassis vertical rotation axis.
     */
    if (imu_gyro_calibrated == 0U)
    {
        imu_gyro_x_bias_sum += imu_dmp_data.gyro_x;
        imu_gyro_calibration_count++;

        if (imu_gyro_calibration_count >=
            IMU_GYRO_CALIBRATION_SAMPLES)
        {
            imu_gyro_x_bias_raw =
                (float)imu_gyro_x_bias_sum /
                (float)imu_gyro_calibration_count;

            imu_gyro_calibrated = 1U;

            imu_heading_deg = 0.0f;

            imu_previous_qw = imu_dmp_data.qw;
            imu_previous_qx = imu_dmp_data.qx;
            imu_previous_qy = imu_dmp_data.qy;
            imu_previous_qz = imu_dmp_data.qz;

            imu_heading_initialized = 1U;
        }

        return IMU_STATUS_NO_DATA;
    }

    if (imu_heading_initialized == 0U)
    {
        imu_previous_qw = imu_dmp_data.qw;
        imu_previous_qx = imu_dmp_data.qx;
        imu_previous_qy = imu_dmp_data.qy;
        imu_previous_qz = imu_dmp_data.qz;

        imu_heading_deg = 0.0f;
        imu_heading_initialized = 1U;
    }
    else
    {
        float delta_w;
        float delta_x;
        float twist_norm;
        float delta_angle_rad;
        float gyro_x_dps;

        /*
         * Calculate the relative quaternion:
         * inverse(previous) * current.
         */
        delta_w =
            (imu_previous_qw * imu_dmp_data.qw) +
            (imu_previous_qx * imu_dmp_data.qx) +
            (imu_previous_qy * imu_dmp_data.qy) +
            (imu_previous_qz * imu_dmp_data.qz);

        delta_x =
            (imu_previous_qw * imu_dmp_data.qx) -
            (imu_previous_qx * imu_dmp_data.qw) -
            (imu_previous_qy * imu_dmp_data.qz) +
            (imu_previous_qz * imu_dmp_data.qy);

        /*
         * Quaternion q and -q represent the same orientation.
         * Keep the shortest incremental rotation.
         */
        if (delta_w < 0.0f)
        {
            delta_w = -delta_w;
            delta_x = -delta_x;
        }

        /*
         * Chassis vertical axis corresponds to sensor X.
         * Extract only the twist around sensor X.
         */
        twist_norm = sqrtf(
            (delta_w * delta_w) +
            (delta_x * delta_x)
        );

        /*
         * Remove the measured stationary gyro bias.
         */
        gyro_x_dps =
            ((float)imu_dmp_data.gyro_x -
             imu_gyro_x_bias_raw) /
            IMU_GYRO_LSB_PER_DPS;

        if (twist_norm > 0.000001f)
        {
            delta_w /= twist_norm;
            delta_x /= twist_norm;

            delta_angle_rad =
                2.0f * atan2f(delta_x, delta_w);

            /*
             * Ignore very small gyro motion to reduce heading drift.
             */
            if (fabsf(gyro_x_dps) >=
                IMU_GYRO_DEADBAND_DPS)
            {
                imu_heading_deg +=
                    IMU_HEADING_SIGN *
                    delta_angle_rad *
                    IMU_RAD_TO_DEG;
            }
        }

        imu_previous_qw = imu_dmp_data.qw;
        imu_previous_qx = imu_dmp_data.qx;
        imu_previous_qy = imu_dmp_data.qy;
        imu_previous_qz = imu_dmp_data.qz;
    }

    imu_last_update_tick = HAL_GetTick();
    imu_data_valid = 1U;

    return IMU_STATUS_OK;
}


uint8_t IMU_HasValidData(void)
{
    return imu_data_valid;
}


float IMU_GetYawDeg(void)
{
    return imu_dmp_data.yaw_deg;
}


float IMU_GetPitchDeg(void)
{
    return imu_dmp_data.pitch_deg;
}


float IMU_GetRollDeg(void)
{
    return imu_dmp_data.roll_deg;
}

int16_t IMU_GetGyroXRaw(void)
{
    return imu_dmp_data.gyro_x;
}

int16_t IMU_GetGyroYRaw(void)
{
    return imu_dmp_data.gyro_y;
}

int16_t IMU_GetGyroZRaw(void)
{
    return imu_dmp_data.gyro_z;
}
float IMU_GetHeadingDeg(void)
{
    return imu_heading_deg;
}


void IMU_ResetHeading(void)
{
    imu_heading_deg = 0.0f;

    if (imu_data_valid != 0U)
    {
        imu_previous_qw = imu_dmp_data.qw;
        imu_previous_qx = imu_dmp_data.qx;
        imu_previous_qy = imu_dmp_data.qy;
        imu_previous_qz = imu_dmp_data.qz;

        imu_heading_initialized = 1U;
    }
    else
    {
        imu_heading_initialized = 0U;
    }
}

uint8_t IMU_IsHealthy(uint32_t timeout_ms)
{
    uint32_t current_tick;

    if (imu_data_valid == 0U)
    {
        return 0U;
    }

    current_tick = HAL_GetTick();

    if ((current_tick - imu_last_update_tick) > timeout_ms)
    {
        return 0U;
    }

    return 1U;
}
