#ifndef _SYSTEM_PARAMS_H_
#define _SYSTEM_PARAMS_H_

#include "zf_common_headfile.h"

extern float base_pwm;
extern float weight_list[5];  // 权重列表
extern uint32 turn_delay_cnt; // 转弯延时计数

extern uint8 turn_angle_velocity_time;
extern uint8 turn_err_time;
extern uint8 bottom_velocity_time;

extern float bottom_velocity_pid[3];
extern float turn_angle_velocity_pid[3];
extern float turn_err_pid[3];

#endif