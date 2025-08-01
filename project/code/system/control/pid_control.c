#include "pid_control.h"
#include "control.h"
#include "params.h"

float pid_turn_control(float turn_err_target, float z_velocity)
{
    static uint8 turn_angle_velocity_cnt = 0;
    static uint8 turn_err_cnt = 0;

    float turn_err_out = 0.0f;
    float turn_diff = 0.0f;

    if (turn_angle_velocity_cnt % turn_angle_velocity_time == 0)
    {
        turn_diff = PID_calc_Position(&turn_angle_velocity_PID, z_velocity, turn_err_out);
        turn_angle_velocity_cnt = 0;
    }
    else
    {
        turn_angle_velocity_cnt++;
    }

    if (turn_err_cnt % turn_err_time == 0)
    {
        turn_err_out = PID_calc_Position(&turn_err_PID, turn_err_target, 0.0f);
        turn_err_cnt = 0;
    }
    else
    {
        turn_err_cnt++;
    }

    return turn_diff;
}

float pid_turn_control_without_angle_vel(float turn_err_target)
{
    static uint8 turn_angle_velocity_cnt = 0;
    static uint8 turn_err_cnt = 0;

    float turn_diff = 0.0f;

    // 误差环
    if (turn_err_cnt % turn_err_time == 0)
    {
        turn_diff = PID_calc_Position(&turn_err_PID, turn_err_target, 0.0f);
        turn_err_cnt = 0;
    }
    else
    {
        turn_err_cnt++;
    }

    return turn_diff;
}

float pid_bottom_control(float bottom_velocity_target, float bottom_velocity)
{
    static uint8 bottom_velocity_cnt = 0;
    float bottom_velocity_out = 0.0f;

    if (bottom_velocity_cnt % bottom_velocity_time == 0)
    {
        bottom_velocity_out = PID_calc_Position(&bottom_velocity_PID, bottom_velocity, bottom_velocity_target);
        bottom_velocity_cnt = 0;
    }
    else
    {
        bottom_velocity_cnt++;
    }
    return bottom_velocity_out;
}
