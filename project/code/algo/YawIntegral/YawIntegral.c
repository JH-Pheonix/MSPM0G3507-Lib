#include "YawIntegral.h"
// #include "control.h"  // 暂时注释掉，需要实现control相关函数

// 临时实现restrictValueF函数
static void restrictValueF(float *value, float max_val, float min_val) {
    if (*value > max_val) *value = max_val;
    if (*value < min_val) *value = min_val;
}

static Integral_info s_integral;

void integral_init(float dt)
{
    s_integral.yaw = 0.0f;
    s_integral.dt = dt;
}

void integral_update(imu_data_t *data)
{

    s_integral.yaw += data->gyro_z;

    // 限制积分值的范围，防止溢出或累积过大的误差
    restrictValueF(&s_integral.yaw, 1000.0f, -1000.0f);
}

float integral_get_yaw(void)
{
    return s_integral.yaw;
}

void integral_reset(void)
{
    s_integral.yaw = 0.0f;
}