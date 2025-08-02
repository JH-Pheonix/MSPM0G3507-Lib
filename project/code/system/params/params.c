#include "params.h"

float bottom_velocity_target = 0.0f; // 底部速度目标值

float base_pwm = 1000.0f;                         // 基准PWM值
float weight_list[5] = {-8000, -80, 0, 80, 8000}; // 权重列表
uint32 turn_delay_cnt = 50;                       // 转弯延时计数

// velocity only
const float left_vel_target = 100;
const float right_vel_target = 100;

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
const float control_without_left_vel_base_pwm = 1800.0f;
const float control_without_right_vel_base_pwm = 2100.0f;
const int control_without_vel_turn_delay = 2;      // 持续时间，可根据实际调整
const int control_start_delay_time = 50;           // 启动延时周期，可根据实际调整
const int control_start_left_vel_base_pwm = 2200;  // 启动阶段左PWM
const int control_start_right_vel_base_pwm = 1500; // 启动阶段右PWM

// PID控制参数
float bottom_velocity_pid[3] = {0.0, 0.0, 0.0};
float bottom_velocity_left_pid[5] = {0.750, 0.0, 0.0, 0.30, 40.0};
// kp,ki,kd,kp2,confine
float bottom_velocity_right_pid[5] = {0.50, 0.0, 0.0, 0.30, 40.0};

float turn_angle_velocity_pid[3] = {0.0, 0.0, 0.0};
float turn_err_pid[3] = {7, 0.0, 1.0};

uint8 turn_angle_velocity_time = 10;
uint8 turn_err_time = 1;
uint8 bottom_velocity_time = 10;

bool RA_flag = 0;
bool RA_flag_R = 0;
bool RA_flag_L = 0;
uint16 RA_cnt = 0;
uint16 RA_cnt_n = 7;