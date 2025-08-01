#ifndef _SYSTEM_PARAMS_H_
#define _SYSTEM_PARAMS_H_

#include "zf_common_headfile.h"

extern float bottom_velocity_target;

extern float base_pwm;
extern float weight_list[5];  // 权重列表
extern uint32 turn_delay_cnt; // 转弯延时计数

extern uint8 turn_angle_velocity_time;
extern uint8 turn_err_time;
extern uint8 bottom_velocity_time;

extern float bottom_velocity_pid[3];
extern float turn_angle_velocity_pid[3];
extern float turn_err_pid[3];

extern int32 single_line_encoder_cnt;

extern const float control_only_encoder_base_pwm;
extern const int32 control_only_encoder_target_distance;
extern const int control_only_encoder_turn_delay_times;
extern const float control_only_encoder_diff;

extern const float control_tracking_open_base_pwm; // 开环转弯基准PWM值
extern const float control_tracking_open_turn_diff;
extern const int control_tracking_open_turn_delay_times;

extern const float control_without_vel_base_pwm;
#endif