#ifndef _DEVICE_IMU_H_
#define _DEVICE_IMU_H_

#include "zf_common_headfile.h"

typedef enum {
    IMU_DEVICE_660RA,
    IMU_DEVICE_660RB,
    IMU_DEVICE_963RA,
    IMU_DEVICE_ADIS16505,
} imu_device_enum;

typedef struct {
    float accel_x, accel_y, accel_z;    // 加速度 (m/s²)
    float gyro_x, gyro_y, gyro_z;       // 角速度 (°/s)
} imu_data_t;

void imu_init(imu_device_enum device);
imu_data_t imu_get_data(void);

#endif
