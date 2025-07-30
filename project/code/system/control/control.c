#include "control.h"

pid_type_def turn_angle_velocity_PID;
pid_type_def turn_err_PID;
pid_type_def bottom_velocity_PID;

uint8 turn_angle_velocity_time;
uint8 turn_err_time;
uint8 bottom_velocity_time;

static void control_params_handler(pid_type_def *pid,
                                   const float para[3],
                                   float kp_coefficient,
                                   float ki_coefficient,
                                   float kd_coefficient,
                                   float max_out,
                                   float max_i_out)
{
    float temp_pid[3];
    temp_pid[0] = (float)para[0] / kp_coefficient;
    temp_pid[1] = (float)para[1] / ki_coefficient;
    temp_pid[2] = (float)para[2] / kd_coefficient;
    pid_init(pid, temp_pid, max_out, max_i_out);
}

void control_pid_params_init()
{
    float bottom_velocity_pid[3] = {0.0, 0.0, 0.0};
    control_params_handler(&bottom_velocity_PID,
                           bottom_velocity_pid,
                           10, 10, 10, 20, 10.0f);

    float turn_angle_velocity_pid[3] = {0.0, 0.0, 0.0};
    control_params_handler(&turn_angle_velocity_PID,
                           turn_angle_velocity_pid,
                           10, 10, 10, 20, 10.0f);

    float turn_err_pid[3] = {0.0, 0.0, 0.0};
    control_params_handler(&turn_err_PID,
                           turn_err_pid,
                           10, 10, 10, 20, 10.0f);
}

void control_init()
{
    turn_angle_velocity_PID = (pid_type_def){0};
    turn_err_PID = (pid_type_def){0};
    bottom_velocity_PID = (pid_type_def){0};
}

void main_control_pid(float turn_err_target, float z_velocity, float bottom_velocity_target, float bottom_velocity)
{
    float turn_diff = pid_turn_control(turn_err_target, z_velocity);
    float bottom_velocity_out = pid_bottom_control(bottom_velocity_target, bottom_velocity);

    // set pwm
}

void main_control_novel(float turn_err_target, float z_velocity)
{
    float turn_diff = pid_turn_control(turn_err_target, z_velocity);

    // set pwm
}

void main_control_open(float z_velocity)
{
    float turn_diff = pid_turn_control(0.0f, z_velocity);
    grey_tracking_scan();

    // handler
}