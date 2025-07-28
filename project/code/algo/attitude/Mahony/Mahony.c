#include "Mahony.h"

// static variables
static MahonyAHRS_INFO s_mahonyAHRS_info;

// func
static float Mahony_invSqrt(float x);

uint32 mahony_cnt = 0;
void MahonyAHRS_init()
{
    s_mahonyAHRS_info.twoKi = twoKiDef; // 2 * integral gain (Ki)
    s_mahonyAHRS_info.q0 = 1.0f;
    s_mahonyAHRS_info.q1 = 0.0f;
    s_mahonyAHRS_info.q2 = 0.0f;
    s_mahonyAHRS_info.q3 = 0.0f;
    s_mahonyAHRS_info.integralFBx = 0.0f;
    s_mahonyAHRS_info.integralFBy = 0.0f;
    s_mahonyAHRS_info.integralFBz = 0.0f;
    s_mahonyAHRS_info.invSampleFreq = TauDef;
}

void MahonyAHRS_calibrate(imu_data_t imu_data)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = Mahony_invSqrt(imu_data.accel_x * imu_data.accel_x + imu_data.accel_y * imu_data.accel_y + imu_data.accel_z * imu_data.accel_z);
    imu_data.accel_x *= recipNorm;
    imu_data.accel_y *= recipNorm;
    imu_data.accel_z *= recipNorm;

    init_pitch = atan2f(-imu_data.accel_x, imu_data.accel_z);
    init_roll = atan2f(imu_data.accel_y, imu_data.accel_z);

    sin_roll = sinf(init_roll);
    cos_roll = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    // 没有磁力计
    init_yaw = 0.0f;

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    s_mahonyAHRS_info.q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    s_mahonyAHRS_info.q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    s_mahonyAHRS_info.q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    s_mahonyAHRS_info.q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = Mahony_invSqrt(s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q0 + s_mahonyAHRS_info.q1 * s_mahonyAHRS_info.q1 + s_mahonyAHRS_info.q2 * s_mahonyAHRS_info.q2 + s_mahonyAHRS_info.q3 * s_mahonyAHRS_info.q3);
    s_mahonyAHRS_info.q0 *= recipNorm;
    s_mahonyAHRS_info.q1 *= recipNorm;
    s_mahonyAHRS_info.q2 *= recipNorm;
    s_mahonyAHRS_info.q3 *= recipNorm;
}

void MahonyAHRS_update(imu_data_t imu_data)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // imu_data.gyro_x *= 0.0174533f;
    // imu_data.gyro_y *= 0.0174533f;
    // imu_data.gyro_z *= 0.0174533f;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((imu_data.accel_x == 0.0f) && (imu_data.accel_y == 0.0f) && (imu_data.accel_z == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = Mahony_invSqrt(imu_data.accel_x * imu_data.accel_x + imu_data.accel_y * imu_data.accel_y + imu_data.accel_z * imu_data.accel_z);
        imu_data.accel_x *= recipNorm;
        imu_data.accel_y *= recipNorm;
        imu_data.accel_z *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = s_mahonyAHRS_info.q1 * s_mahonyAHRS_info.q3 - s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q2;
        halfvy = s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q1 + s_mahonyAHRS_info.q2 * s_mahonyAHRS_info.q3;
        halfvz = s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q0 - 0.5f + s_mahonyAHRS_info.q3 * s_mahonyAHRS_info.q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (imu_data.accel_y * halfvz - imu_data.accel_z * halfvy);
        halfey = (imu_data.accel_z * halfvx - imu_data.accel_x * halfvz);
        halfez = (imu_data.accel_x * halfvy - imu_data.accel_y * halfvx);

        // Compute and apply integral feedback if enabled
        if (s_mahonyAHRS_info.twoKi > 0.0f)
        {
            s_mahonyAHRS_info.integralFBx += s_mahonyAHRS_info.twoKi * halfex * s_mahonyAHRS_info.invSampleFreq; // integral error scaled by Ki
            s_mahonyAHRS_info.integralFBy += s_mahonyAHRS_info.twoKi * halfey * s_mahonyAHRS_info.invSampleFreq;
            s_mahonyAHRS_info.integralFBz += s_mahonyAHRS_info.twoKi * halfez * s_mahonyAHRS_info.invSampleFreq;
            imu_data.gyro_x += s_mahonyAHRS_info.integralFBx; // apply integral feedback
            imu_data.gyro_y += s_mahonyAHRS_info.integralFBy;
            imu_data.gyro_z += s_mahonyAHRS_info.integralFBz;
        }
        else
        {
            s_mahonyAHRS_info.integralFBx = 0.0f; // prevent integral windup
            s_mahonyAHRS_info.integralFBy = 0.0f;
            s_mahonyAHRS_info.integralFBz = 0.0f;
        }

        // Apply proportional feedback
        imu_data.gyro_x += twoKpDef * halfex;
        imu_data.gyro_y += twoKpDef * halfey;
        imu_data.gyro_z += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    imu_data.gyro_x *= (0.5f * s_mahonyAHRS_info.invSampleFreq); // pre-multiply common factors
    imu_data.gyro_y *= (0.5f * s_mahonyAHRS_info.invSampleFreq);
    imu_data.gyro_z *= (0.5f * s_mahonyAHRS_info.invSampleFreq);
    qa = s_mahonyAHRS_info.q0;
    qb = s_mahonyAHRS_info.q1;
    qc = s_mahonyAHRS_info.q2;
    s_mahonyAHRS_info.q0 += (-qb * imu_data.gyro_x - qc * imu_data.gyro_y - s_mahonyAHRS_info.q3 * imu_data.gyro_z);
    s_mahonyAHRS_info.q1 += (qa * imu_data.gyro_x + qc * imu_data.gyro_z - s_mahonyAHRS_info.q3 * imu_data.gyro_y);
    s_mahonyAHRS_info.q2 += (qa * imu_data.gyro_y - qb * imu_data.gyro_z + s_mahonyAHRS_info.q3 * imu_data.gyro_x);
    s_mahonyAHRS_info.q3 += (qa * imu_data.gyro_z + qb * imu_data.gyro_y - qc * imu_data.gyro_x);

    // Normalise quaternion
    recipNorm = Mahony_invSqrt(s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q0 + s_mahonyAHRS_info.q1 * s_mahonyAHRS_info.q1 + s_mahonyAHRS_info.q2 * s_mahonyAHRS_info.q2 + s_mahonyAHRS_info.q3 * s_mahonyAHRS_info.q3);
    s_mahonyAHRS_info.q0 *= recipNorm;
    s_mahonyAHRS_info.q1 *= recipNorm;
    s_mahonyAHRS_info.q2 *= recipNorm;
    s_mahonyAHRS_info.q3 *= recipNorm;

    s_mahonyAHRS_info.roll = atan2(s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q1 + s_mahonyAHRS_info.q2 * s_mahonyAHRS_info.q3, 0.5f - s_mahonyAHRS_info.q1 * s_mahonyAHRS_info.q1 - s_mahonyAHRS_info.q2 * s_mahonyAHRS_info.q2); // 英飞凌的加速试一下  用了英飞凌的底层支持库 ，不是英飞凌的换成atan就行
    s_mahonyAHRS_info.pitch = asinf(-2.0f * (s_mahonyAHRS_info.q1 * s_mahonyAHRS_info.q3 - s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q2));
    s_mahonyAHRS_info.yaw = atan2(s_mahonyAHRS_info.q1 * s_mahonyAHRS_info.q2 + s_mahonyAHRS_info.q0 * s_mahonyAHRS_info.q3, 0.5f - s_mahonyAHRS_info.q2 * s_mahonyAHRS_info.q2 - s_mahonyAHRS_info.q3 * s_mahonyAHRS_info.q3); // 加速我来试一下

    mahony_cnt++;
}

float MahonyAHRS_get_roll()
{
    return s_mahonyAHRS_info.roll * 57.29578f;
}

float MahonyAHRS_get_pitch()
{
    return s_mahonyAHRS_info.pitch * 57.29578f;
}

float MahonyAHRS_get_yaw()
{
    return s_mahonyAHRS_info.yaw * 57.29578f;
}

static float Mahony_invSqrt(float x)
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