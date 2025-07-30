#ifndef _SYSTEM_CONTROL_H_
#define _SYSTEM_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"

void control_pid_params_init();
void control_init();

extern pid_type_def turn_angle_velocity_PID;
extern pid_type_def turn_err_PID;
extern pid_type_def bottom_velocity_PID;

extern uint8 turn_angle_velocity_time;
extern uint8 turn_err_time;
extern uint8 bottom_velocity_time;

#endif