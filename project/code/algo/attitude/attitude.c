#include "attitude.h"
#include "EKF/QuaternionEKF.h"
#include "Madgwick/Madgwick.h"
#include "Mahony/Mahony.h"
#include "Integral/Integral.h"

euler_angle_t g_euler_angle;
euler_angle_t g_euler_angle_bias;

static attitude_algorithm_enum current_algorithm;

void attitude_init(attitude_algorithm_enum algo)
{
    current_algorithm = algo;

    switch (current_algorithm)
    {
    case ATTITUDE_EKF:
        IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0);
        break;
    case ATTITUDE_MADGWICK:
        MadgwickAHRS_init();
        break;
    case ATTITUDE_MAHONY:
        MahonyAHRS_init();
        // 获取一个初始的acc信息加速收敛
        {
            imu_data_t init_data = imu_get_data();
            MahonyAHRS_calibrate(init_data);
        }
        break;
    case ATTITUDE_INTEGRAL:
        integral_init(0.001f);  // 1ms采样周期
        break;
    default:
        MahonyAHRS_init();
        {
            imu_data_t init_data = imu_get_data();
            MahonyAHRS_calibrate(init_data);
        }
        break;
    }
}

void attitude_update(imu_data_t imu_data)
{
    switch (current_algorithm)
    {
    case ATTITUDE_EKF:
        IMU_QuaternionEKF_Update(imu_data);
        break;
    case ATTITUDE_MADGWICK:
        MadgwickAHRS_update(imu_data);
        break;
    case ATTITUDE_MAHONY:
        MahonyAHRS_update(imu_data);
        break;
    case ATTITUDE_INTEGRAL:
        integral_update(&imu_data);
        break;
    default:
        MahonyAHRS_update(imu_data);
        break;
    }

    // 更新全局姿态数据
    switch (current_algorithm)
    {
    case ATTITUDE_EKF:
        // 需要在EKF中添加这些获取函数
        g_euler_angle.roll = 0.0f;   // EKF_get_roll();
        g_euler_angle.pitch = 0.0f;  // EKF_get_pitch();
        g_euler_angle.yaw = 0.0f;    // EKF_get_yaw();
        break;
    case ATTITUDE_MADGWICK:
        g_euler_angle.roll = MadgwickAHRS_get_roll();
        g_euler_angle.pitch = MadgwickAHRS_get_pitch();
        g_euler_angle.yaw = MadgwickAHRS_get_yaw();
        break;
    case ATTITUDE_MAHONY:
        g_euler_angle.roll = MahonyAHRS_get_roll();
        g_euler_angle.pitch = MahonyAHRS_get_pitch();
        g_euler_angle.yaw = MahonyAHRS_get_yaw();
        break;
    case ATTITUDE_INTEGRAL:
        g_euler_angle.roll = integral_get_roll() * 57.29578f;    // 弧度转角度
        g_euler_angle.pitch = integral_get_pitch() * 57.29578f;
        g_euler_angle.yaw = integral_get_yaw() * 57.29578f;
        break;
    default:
        g_euler_angle.roll = MahonyAHRS_get_roll();
        g_euler_angle.pitch = MahonyAHRS_get_pitch();
        g_euler_angle.yaw = MahonyAHRS_get_yaw();
        break;
    }

    // 处理yaw角度范围 0~360
    g_euler_angle.yaw = 360.0f - g_euler_angle.yaw;
    if (g_euler_angle.yaw > 360.0f) {
        g_euler_angle.yaw -= 360.0f;
    }
    if (g_euler_angle.yaw < 0.0f) {
        g_euler_angle.yaw += 360.0f;
    }
}

euler_angle_t attitude_get_data(void)
{
    return g_euler_angle;
}