#ifndef _SYSTEM_PID_CONTROL_H_
#define _SYSTEM_PID_CONTROL_H_

#include "zf_common_headfile.h"

float pid_turn_control(float turn_err_target, float z_velocity);
float pid_bottom_control(float bottom_velocity_target, float bottom_velocity);
float pid_turn_control_without_vel(float turn_err_target);

#endif