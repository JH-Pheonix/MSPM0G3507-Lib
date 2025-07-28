#include "init.h"
#include "imu.h"

void system_init(void)
{
    imu_init(IMU_DEVICE_660RA); 
}