#include "Madgwick.h"

static MadgwickAHRS_INFO s_madgwickAHRS_info;

static float Madgwick_invSqrt(float x);

void MadgwickAHRS_init()
{
    s_madgwickAHRS_info.beta = MADGWICK_BETA_DEF; // 2 * proportional gain
    s_madgwickAHRS_info.q0 = 1.0f;
    s_madgwickAHRS_info.q1 = 0.0f;
    s_madgwickAHRS_info.q2 = 0.0f;
    s_madgwickAHRS_info.q3 = 0.0f;
    s_madgwickAHRS_info.sampleFreq = 1 / MADGWICK_TAU; // Default sample frequency
}

void MadgwickAHRS_update(imu_data_t imu_data)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-s_madgwickAHRS_info.q1 * imu_data.gyro_x - s_madgwickAHRS_info.q2 * imu_data.gyro_y - s_madgwickAHRS_info.q3 * imu_data.gyro_z);
    qDot2 = 0.5f * (s_madgwickAHRS_info.q0 * imu_data.gyro_x + s_madgwickAHRS_info.q2 * imu_data.gyro_z - s_madgwickAHRS_info.q3 * imu_data.gyro_y);
    qDot3 = 0.5f * (s_madgwickAHRS_info.q0 * imu_data.gyro_y - s_madgwickAHRS_info.q1 * imu_data.gyro_z + s_madgwickAHRS_info.q3 * imu_data.gyro_x);
    qDot4 = 0.5f * (s_madgwickAHRS_info.q0 * imu_data.gyro_z + s_madgwickAHRS_info.q1 * imu_data.gyro_y - s_madgwickAHRS_info.q2 * imu_data.gyro_x);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((imu_data.accel_x == 0.0f) && (imu_data.accel_y == 0.0f) && (imu_data.accel_z == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = Madgwick_invSqrt(imu_data.accel_x * imu_data.accel_x + imu_data.accel_y * imu_data.accel_y + imu_data.accel_z * imu_data.accel_z);
        imu_data.accel_x *= recipNorm;
        imu_data.accel_y *= recipNorm;
        imu_data.accel_z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * s_madgwickAHRS_info.q0;
        _2q1 = 2.0f * s_madgwickAHRS_info.q1;
        _2q2 = 2.0f * s_madgwickAHRS_info.q2;
        _2q3 = 2.0f * s_madgwickAHRS_info.q3;
        _4q0 = 4.0f * s_madgwickAHRS_info.q0;
        _4q1 = 4.0f * s_madgwickAHRS_info.q1;
        _4q2 = 4.0f * s_madgwickAHRS_info.q2;
        _8q1 = 8.0f * s_madgwickAHRS_info.q1;
        _8q2 = 8.0f * s_madgwickAHRS_info.q2;
        q0q0 = s_madgwickAHRS_info.q0 * s_madgwickAHRS_info.q0;
        q1q1 = s_madgwickAHRS_info.q1 * s_madgwickAHRS_info.q1;
        q2q2 = s_madgwickAHRS_info.q2 * s_madgwickAHRS_info.q2;
        q3q3 = s_madgwickAHRS_info.q3 * s_madgwickAHRS_info.q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * imu_data.accel_x + _4q0 * q1q1 - _2q1 * imu_data.accel_y;
        s1 = _4q1 * q3q3 - _2q3 * imu_data.accel_x + 4.0f * q0q0 * s_madgwickAHRS_info.q1 - _2q0 * imu_data.accel_y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * imu_data.accel_z;
        s2 = 4.0f * q0q0 * s_madgwickAHRS_info.q2 + _2q0 * imu_data.accel_x + _4q2 * q3q3 - _2q3 * imu_data.accel_y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * imu_data.accel_z;
        s3 = 4.0f * q1q1 * s_madgwickAHRS_info.q3 - _2q1 * imu_data.accel_x + 4.0f * q2q2 * s_madgwickAHRS_info.q3 - _2q2 * imu_data.accel_y;
        recipNorm = Madgwick_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= s_madgwickAHRS_info.beta * s0;
        qDot2 -= s_madgwickAHRS_info.beta * s1;
        qDot3 -= s_madgwickAHRS_info.beta * s2;
        qDot4 -= s_madgwickAHRS_info.beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    s_madgwickAHRS_info.q0 += qDot1 * (1.0f / s_madgwickAHRS_info.sampleFreq);
    s_madgwickAHRS_info.q1 += qDot2 * (1.0f / s_madgwickAHRS_info.sampleFreq);
    s_madgwickAHRS_info.q2 += qDot3 * (1.0f / s_madgwickAHRS_info.sampleFreq);
    s_madgwickAHRS_info.q3 += qDot4 * (1.0f / s_madgwickAHRS_info.sampleFreq);

    // Normalise quaternion
    recipNorm = Madgwick_invSqrt(s_madgwickAHRS_info.q0 * s_madgwickAHRS_info.q0 + s_madgwickAHRS_info.q1 * s_madgwickAHRS_info.q1 + s_madgwickAHRS_info.q2 * s_madgwickAHRS_info.q2 + s_madgwickAHRS_info.q3 * s_madgwickAHRS_info.q3);
    s_madgwickAHRS_info.q0 *= recipNorm;
    s_madgwickAHRS_info.q1 *= recipNorm;
    s_madgwickAHRS_info.q2 *= recipNorm;
    s_madgwickAHRS_info.q3 *= recipNorm;

    s_madgwickAHRS_info.roll = atan2(s_madgwickAHRS_info.q0 * s_madgwickAHRS_info.q1 + s_madgwickAHRS_info.q2 * s_madgwickAHRS_info.q3, 0.5f - s_madgwickAHRS_info.q1 * s_madgwickAHRS_info.q1 - s_madgwickAHRS_info.q2 * s_madgwickAHRS_info.q2); // 英飞凌的加速试一下  用了英飞凌的底层支持库 ，不是英飞凌的换成atan就行
    s_madgwickAHRS_info.pitch = asinf(-2.0f * (s_madgwickAHRS_info.q1 * s_madgwickAHRS_info.q3 - s_madgwickAHRS_info.q0 * s_madgwickAHRS_info.q2));
    s_madgwickAHRS_info.yaw = atan2(s_madgwickAHRS_info.q1 * s_madgwickAHRS_info.q2 + s_madgwickAHRS_info.q0 * s_madgwickAHRS_info.q3, 0.5f - s_madgwickAHRS_info.q2 * s_madgwickAHRS_info.q2 - s_madgwickAHRS_info.q3 * s_madgwickAHRS_info.q3); // 加速我来试一下
}

void MadgwickAHRS_calibrate(imu_data_t imu_data)
{
    // 简单的校准实现：使用加速度计数据初始化姿态
    float recipNorm;
    float init_pitch, init_roll;
    
    // 归一化加速度计数据
    recipNorm = Madgwick_invSqrt(imu_data.accel_x * imu_data.accel_x + 
                                 imu_data.accel_y * imu_data.accel_y + 
                                 imu_data.accel_z * imu_data.accel_z);
    imu_data.accel_x *= recipNorm;
    imu_data.accel_y *= recipNorm;
    imu_data.accel_z *= recipNorm;
    
    // 计算初始俯仰角和横滚角
    init_pitch = asinf(-imu_data.accel_x);
    init_roll = atan2f(imu_data.accel_y, imu_data.accel_z);
    
    // 初始化四元数（假设偏航角为0）
    float cp2 = cosf(init_pitch * 0.5f);
    float sp2 = sinf(init_pitch * 0.5f);
    float cr2 = cosf(init_roll * 0.5f);
    float sr2 = sinf(init_roll * 0.5f);
    
    s_madgwickAHRS_info.q0 = cp2 * cr2;
    s_madgwickAHRS_info.q1 = cp2 * sr2;
    s_madgwickAHRS_info.q2 = sp2 * cr2;
    s_madgwickAHRS_info.q3 = -sp2 * sr2;
}

float MadgwickAHRS_get_roll()
{
    return s_madgwickAHRS_info.roll * 57.29578f; // Convert to degrees
}

float MadgwickAHRS_get_pitch()
{
    return s_madgwickAHRS_info.pitch * 57.29578f; // Convert to degrees
}

float MadgwickAHRS_get_yaw()
{
    return s_madgwickAHRS_info.yaw * 57.29578f + 180.0f; // Convert to degrees and adjust yaw
}

static float Madgwick_invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}