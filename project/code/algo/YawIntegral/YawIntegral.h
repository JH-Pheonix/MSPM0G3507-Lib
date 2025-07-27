#ifndef YAW_INTEGRAL_H
#define YAW_INTEGRAL_H

#include "zf_common_headfile.h"
#include "../../device/imu/imu.h"

typedef struct
{
    float yaw;

    float dt;
} Integral_info;

void integral_init(float dt);
void integral_update(imu_data_t *data);
float integral_get_yaw(void);
void integral_reset(void);

#endif