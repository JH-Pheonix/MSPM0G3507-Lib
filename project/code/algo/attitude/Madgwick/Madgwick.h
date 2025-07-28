#ifndef _ATTITUDE_MADGWICK_H
#define _ATTITUDE_MADGWICK_H

#include "zf_common_headfile.h"
#include "../../device/imu/imu.h"

// Madgwick 使用​​梯度下降法​​最小化误差，Beta是步长
#define MADGWICK_TAU (0.001f)
#define MADGWICK_BETA_DEF 0.001f

typedef struct
{
    float beta;
    float q0, q1, q2, q3; // quaternion components
    float sampleFreq;

    float pitch, roll, yaw;
} MadgwickAHRS_INFO;

void MadgwickAHRS_init();
void MadgwickAHRS_calibrate(imu_data_t imu_data);
void MadgwickAHRS_update(imu_data_t imu_data);

float MadgwickAHRS_get_roll(void);
float MadgwickAHRS_get_pitch(void);
float MadgwickAHRS_get_yaw(void);

#endif