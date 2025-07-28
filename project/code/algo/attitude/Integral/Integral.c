#include "Integral.h"
// #include "control.h"  // 暂时注释掉，需要实现control相关函数

// 临时实现restrictValueF函数
static void restrictValueF(float *value, float max_val, float min_val) {
    if (*value > max_val) *value = max_val;
    if (*value < min_val) *value = min_val;
}

static Integral_info s_integral;

void integral_init(float dt)
{
    s_integral.roll = 0.0f;
    s_integral.pitch = 0.0f;
    s_integral.yaw = 0.0f;
    s_integral.dt = dt;
}

void integral_update(imu_data_t *data)
{
    // 对三个轴的角速度进行积分
    s_integral.roll += data->gyro_x * s_integral.dt;
    s_integral.pitch += data->gyro_y * s_integral.dt;
    s_integral.yaw += data->gyro_z * s_integral.dt;

    // 限制积分值的范围，防止溢出或累积过大的误差
    restrictValueF(&s_integral.roll, 1000.0f, -1000.0f);
    restrictValueF(&s_integral.pitch, 1000.0f, -1000.0f);
    restrictValueF(&s_integral.yaw, 1000.0f, -1000.0f);
}

float integral_get_roll(void)
{
    return s_integral.roll;
}

float integral_get_pitch(void)
{
    return s_integral.pitch;
}

float integral_get_yaw(void)
{
    return s_integral.yaw;
}

void integral_reset(void)
{
    s_integral.roll = 0.0f;
    s_integral.pitch = 0.0f;
    s_integral.yaw = 0.0f;
}
