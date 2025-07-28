#ifndef _ATTITUDE_INTEGRAL_H
#define _ATTITUDE_INTEGRAL_H

#include "zf_common_headfile.h"
#include "../../device/imu/imu.h"

typedef struct
{
    float roll;   // X轴积分值
    float pitch;  // Y轴积分值
    float yaw;    // Z轴积分值

    float dt;
} Integral_info;

void integral_init(float dt);
void integral_update(imu_data_t *data);
float integral_get_roll(void);
float integral_get_pitch(void);
float integral_get_yaw(void);
void integral_reset(void);

#endif
