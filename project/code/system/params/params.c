#include "params.h"

float bottom_velocity_target = 0.0f; // 底部速度目标值

float base_pwm = 1000.0f;                   // 基准PWM值
float weight_list[5] = {-10, -5, 0, 5, 10}; // 权重列表
uint32 turn_delay_cnt = 50;                 // 转弯延时计数

// 纯编码器控制参数
const float control_only_encoder_base_pwm = 1000.0f;
const int32 control_only_encoder_target_distance = 1000;
const int control_only_encoder_turn_delay_times = 20;
const float control_only_encoder_diff = 30.0f;

// 正常循迹，开环转弯
const float control_tracking_open_base_pwm = 1000.0f; // 开环转弯基准PWM值
const float control_tracking_open_turn_diff = 30.0f;
const int control_tracking_open_turn_delay_times = 20;

// 速度开环控制
const float control_without_vel_base_pwm = 1000.0f;

float bottom_velocity_pid[3] = {0.0, 0.0, 0.0};
float turn_angle_velocity_pid[3] = {0.0, 0.0, 0.0};
float turn_err_pid[3] = {0.0, 0.0, 0.0};

uint8 turn_angle_velocity_time = 10;
uint8 turn_err_time = 10;
uint8 bottom_velocity_time = 10;