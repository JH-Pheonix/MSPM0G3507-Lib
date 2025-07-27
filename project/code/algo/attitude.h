#ifndef _ALGO_ATTITUDE_H_
#define _ALGO_ATTITUDE_H_

#include "zf_common_headfile.h"
#include "../device/imu/imu.h"

// 姿态算法类型枚举
typedef enum
{
    ATTITUDE_EKF = 0,       // 扩展卡尔曼滤波
    ATTITUDE_MADGWICK,      // Madgwick 算法
    ATTITUDE_MAHONY,        // Mahony 算法
    ATTITUDE_INTEGRAL,      // 简单积分
} attitude_algorithm_enum;

// 欧拉角结构体
typedef struct
{
    float roll;     // 横滚角 (度)
    float pitch;    // 俯仰角 (度)
    float yaw;      // 偏航角 (度)
} euler_angle_t;

// 全局变量声明
extern euler_angle_t g_euler_angle;
extern euler_angle_t g_euler_angle_bias;

// 函数声明
void attitude_init(attitude_algorithm_enum algo);
void attitude_update(imu_data_t imu_data);
euler_angle_t attitude_get_data(void);

#endif
