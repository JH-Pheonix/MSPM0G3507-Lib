#ifndef _ALGO_LASER_COORD_H_
#define _ALGO_LASER_COORD_H_

#include "zf_common_headfile.h"

#define GIMBAL_X_RESOLUTION 60.0f  // 云台X方向分辨率
#define GIMBAL_Y_RESOLUTION 60.0f  // 云台Y方向分辨率

typedef struct {
    gimbal_angle_2d_t left_top;     // 左上角测量角度数据
    gimbal_angle_2d_t right_top;    // 右上角测量角度数据
    gimbal_angle_2d_t left_bottom;  // 左下角测量角度数据
    gimbal_angle_2d_t right_bottom; // 右下角测量角度数据
} gimbal_calibration_data_t;

// 函数声明
gimbal_angle_2d_t coord_to_angle(float target_x, float target_y);
uint8_t is_coord_in_area(float target_x, float target_y);

#endif