#include "coord.h"

static gimbal_calibration_data_t cal_data = {
    .left_top = {.pan = 10.0f, .tilt = 15.0f},
    .right_top = {.pan = 20.0f, .tilt = 18.0f},
    .left_bottom = {.pan = 8.0f, .tilt = 5.0f},
    .right_bottom = {.pan = 22.0f, .tilt = 7.0f}
}; // 示例值，按需更改

gimbal_angle_2d_t coord_to_angle(float target_x, float target_y)
{
    gimbal_angle_2d_t result = {0.0f, 0.0f};

    float x1 = 0.0f;                  // 左边界
    float x2 = GIMBAL_X_RESOLUTION;   // 右边界
    float y1 = 0.0f;                  // 下边界
    float y2 = GIMBAL_Y_RESOLUTION;   // 上边界

    // 计算归一化坐标 (0-1范围)
    float u = (target_x - x1) / (x2 - x1);  // X方向插值参数
    float v = (target_y - y1) / (y2 - y1);  // Y方向插值参数
    
    // 限制插值参数在0-1范围内
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    
    // 双线性插值计算云台俯仰角 (pan)
    // 下边插值：左下角到右下角
    float pan_bottom = cal_data.left_bottom.pan * (1.0f - u) + cal_data.right_bottom.pan * u;
    // 上边插值：左上角到右上角  
    float pan_top = cal_data.left_top.pan * (1.0f - u) + cal_data.right_top.pan * u;
    // 垂直方向插值
    result.pan = pan_bottom * (1.0f - v) + pan_top * v;
    
    // 双线性插值计算云台倾斜角 (tilt)
    // 下边插值：左下角到右下角
    float tilt_bottom = cal_data.left_bottom.tilt * (1.0f - u) + cal_data.right_bottom.tilt * u;
    // 上边插值：左上角到右上角
    float tilt_top = cal_data.left_top.tilt * (1.0f - u) + cal_data.right_top.tilt * u;
    // 垂直方向插值
    result.tilt = tilt_bottom * (1.0f - v) + tilt_top * v;
    
    return result;
}

// 检查坐标是否在区域内
uint8_t is_coord_in_area(float target_x, float target_y)
{
    // 定义区域边界（实际应用中应根据具体数据确定）
    float min_x = 0.0f;
    float max_x = GIMBAL_X_RESOLUTION;
    float min_y = 0.0f;
    float max_y = GIMBAL_Y_RESOLUTION;

    // 检查是否在区域内
    if (target_x >= min_x && target_x <= max_x && 
        target_y >= min_y && target_y <= max_y) {
        return 1;
    }
    
    return 0;
}
