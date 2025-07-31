#include "params.h"

float bottom_velocity_target = 0.0f; // 底部速度目标值

float base_pwm = 1000.0f;                   // 基准PWM值
float weight_list[5] = {-10, -5, 0, 5, 10}; // 权重列表
uint32 turn_delay_cnt = 50;                 // 转弯延时计数

uint8 turn_angle_velocity_time = 10;
uint8 turn_err_time = 10;
uint8 bottom_velocity_time = 10;

float bottom_velocity_pid[3] = {0.0, 0.0, 0.0};
float turn_angle_velocity_pid[3] = {0.0, 0.0, 0.0};
float turn_err_pid[3] = {0.0, 0.0, 0.0};