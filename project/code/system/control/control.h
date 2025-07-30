#ifndef _SYSTEM_CONTROL_H_
#define _SYSTEM_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"

void control_init();
void control_pid_params_init();
uint8 control_check_turn();
void main_control_novel(float z_velocity);
void main_control_open();
void main_control_pid(float z_velocity, float bottom_velocity_target, float bottom_velocity);

extern pid_type_def turn_angle_velocity_PID;
extern pid_type_def turn_err_PID;
extern pid_type_def bottom_velocity_PID;

#endif