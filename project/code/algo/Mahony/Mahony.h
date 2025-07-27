#ifndef _ATTITUDE_MAHONY_H
#define _ATTITUDE_MAHONY_H

#include "zf_common_headfile.h"
#include "../../device/imu/imu.h"

// 在时间τ内，系统会达到系统响应时间所需的63.2%
// 在时间2τ内，会达到约86.5%
// 在时间3τ内，会达到约95%

// Kp = 1 / (2 × τ)
// Ki / Kp = 0.10

// #define twoKpDef (2.0f * 250.0f) // 2 * proportional gain
// #define twoKiDef (2.0f * 5.0f)   // 2 * integral gain

#define TauDef (0.0005f)

#define twoKpDef ((1.0f / (2.0f * TauDef)) * 2.0f)
#define twoKiDef ((twoKpDef * 0.1f) * 2.0f)

typedef struct
{
    float twoKi;                                 // 2 * integral gain (Ki)
    float q0, q1, q2, q3;                        // quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki
    float invSampleFreq;
    float roll, pitch, yaw;
} MahonyAHRS_INFO;

void MahonyAHRS_init();
void MahonyAHRS_calibrate(imu_data_t imu_data);
void MahonyAHRS_update(imu_data_t imu_data);
float MahonyAHRS_get_roll(void);
float MahonyAHRS_get_pitch(void);
float MahonyAHRS_get_yaw(void);

extern uint32 mahony_cnt;
#endif