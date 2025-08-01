#ifndef _SYSTEM_CONTROL_H_
#define _SYSTEM_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"

void control_init();
void control_pid_params_init();
uint8 control_check_turn();
uint8 control_check_line();

void main_control_pid(float z_velocity, float bottom_velocity_target, float bottom_velocity);

void main_control_pid_without_angle_vel();
void main_control_tracking_open_turn();
void main_control_without_vel(float lvel, float rvel);
void main_control_encoder_only();
void main_control_pid_vel_only();
void main_control_turn_until_line();

void control_callback_func(uint32 event, void *ptr);

extern pid_type_def turn_angle_velocity_PID;
extern pid_type_def turn_err_PID;
extern pid_type_def bottom_velocity_PID;

extern pid_type_def bottom_velocity_left_PID;
extern pid_type_def bottom_velocity_right_PID;

#endif