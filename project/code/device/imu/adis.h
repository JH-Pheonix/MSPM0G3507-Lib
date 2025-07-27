#ifndef _DEVICE_IMU_ADIS_H
#define _DEVICE_IMU_ADIS_H

#include "zf_common_headfile.h"
#include "imu.h"

#define filter_set_high          0xDC06
#define filter_set_low           0xDD00   //滤波器B=6
#define filter_read              0x5C00
#define GYZ_LOW                  0x0C00  //Z轴陀螺仪低16位
#define GYZ_OUT                  0x0E00  //Z轴陀螺仪高16位
#define GYX_LOW                  0x0400
#define GYX_OUT                  0x0600
#define GYY_LOW                  0x0800
#define GYY_OUT                  0x0A00
#define ACC_X_LOW                0x1000
#define ACC_X_HIGH               0x1200
#define ACC_Y_LOW                0x1400
#define ACC_Y_HIGH               0x1600
#define ACC_Z_LOW                0x1800
#define ACC_Z_HIGH               0x1A00
#define TEMP                     0x1C00
#define RST1                     0xE880
#define RST2                     0xE900
#define Kg                       0.025
#define ACC_Kg                   0.00245


#endif