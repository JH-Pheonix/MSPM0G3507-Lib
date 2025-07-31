#include "control.h"
#include "zfmotor.h"
#include "pin.h"
#include "grey_tracking.h"
#include "params.h"
#include "pid_control.h"
#include "encoder.h"
#include "absolute_encoder.h"

pid_type_def turn_angle_velocity_PID;
pid_type_def turn_err_PID;
pid_type_def bottom_velocity_PID;

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
    control_params_handler(&bottom_velocity_PID,
                           bottom_velocity_pid,
                           10, 10, 10, 20, 10.0f);

    control_params_handler(&turn_angle_velocity_PID,
                           turn_angle_velocity_pid,
                           10, 10, 10, 20, 10.0f);

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

void main_control_pid(float z_velocity, float bottom_velocity_target, float bottom_velocity)
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    float turn_err_target = weight_list[0] * left_side +
                            weight_list[1] * left +
                            weight_list[2] * mid +
                            weight_list[3] * right +
                            weight_list[4] * right_side;

    float turn_diff = pid_turn_control(turn_err_target, z_velocity);
    float bottom_velocity_out = pid_bottom_control(bottom_velocity_target, bottom_velocity);

    // set pwm
    motor_set_left_pwm(bottom_velocity_out - turn_diff);
    motor_set_right_pwm(bottom_velocity_out + turn_diff);
}

void main_control_pid_without_vel()
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    float turn_err_target = weight_list[0] * left_side +
                            weight_list[1] * left +
                            weight_list[2] * mid +
                            weight_list[3] * right +
                            weight_list[4] * right_side;

    int16 vel_right = encoder_absolute_encoder_get_offset(0);
    int16 vel_left = encoder_absolute_encoder_get_offset(1);

    float turn_diff = pid_turn_control_without_vel(turn_err_target);
    float bottom_velocity_out = pid_bottom_control(bottom_velocity_target, vel_left + vel_right);

    // set pwm
    motor_set_left_pwm(bottom_velocity_out - turn_diff);
    motor_set_right_pwm(bottom_velocity_out + turn_diff);
}

void main_control_novel(float z_velocity)
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    float turn_err_target = weight_list[0] * left_side +
                            weight_list[1] * left +
                            weight_list[2] * mid +
                            weight_list[3] * right +
                            weight_list[4] * right_side;

    float turn_diff = pid_turn_control(turn_err_target, z_velocity);

    motor_set_left_pwm(base_pwm - turn_diff);  // set left pwm
    motor_set_right_pwm(base_pwm + turn_diff); // set right pwm
}

void main_control_open()
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    float turn_diff = weight_list[0] * left_side +
                      weight_list[1] * left +
                      weight_list[2] * mid +
                      weight_list[3] * right +
                      weight_list[4] * right_side;

    motor_set_left_pwm(base_pwm - turn_diff);
    motor_set_right_pwm(base_pwm + turn_diff);
}

uint8 control_check_turn()
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    if ((left_side && left) || (right_side && right))
    {
        return 1;
    }
    return 0;
}

void control_callback_func(uint32 event, void *ptr)
{
    *((uint8 *)ptr) = 1;

    static uint32 cnt = 0;
    if (control_check_turn()) // 检测到转弯
    {
        // lcd_show_uint(0, 0, 1, 3);
        if (cnt < turn_delay_cnt)
        {
            cnt++;
            return;
        }
        else
        {
            cnt = 0;
        }
    }
    // lcd_show_uint(0, 0, 0, 3);
    main_control_pid_without_vel(); // 执行开放式控制
}
