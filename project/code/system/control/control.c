#include "control.h"
#include "zfmotor.h"
#include "pin.h"
#include "grey_tracking.h"
#include "params.h"
#include "pid_control.h"
#include "encoder.h"
#include "absolute_encoder.h"
#include "lcd.h"

pid_type_def turn_angle_velocity_PID;
pid_type_def turn_err_PID;
pid_type_def bottom_velocity_PID;

pid_type_def bottom_velocity_left_PID;
pid_type_def bottom_velocity_right_PID;

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
                           1, 1, 1, 9999, 9999);

    // control_params_handler(&bottom_velocity_left_PID,
    //                        bottom_velocity_left_pid,
    //                        1, 1, 1, 9999, 5000.0f);

    // control_params_handler(&bottom_velocity_right_PID,
    //                        bottom_velocity_right_pid,
    //                        1, 1, 1, 9999, 5000.0f);

    pid_init_double_k(&bottom_velocity_left_PID,
                      bottom_velocity_left_pid,
                      9999, 5000.0f);
    pid_init_double_k(&bottom_velocity_right_PID,
                      bottom_velocity_right_pid,
                      9999, 5000.0f);

    control_params_handler(&turn_angle_velocity_PID,
                           turn_angle_velocity_pid,
                           1, 1, 1, 9999, 10.0f);

    control_params_handler(&turn_err_PID,
                           turn_err_pid,
                           1, 1, 1, 9999, 5000.0f);
}

void control_init()
{
    turn_angle_velocity_PID = (pid_type_def){0};
    turn_err_PID = (pid_type_def){0};
    bottom_velocity_PID = (pid_type_def){0};

    bottom_velocity_left_PID = (pid_type_def){0};
    bottom_velocity_right_PID = (pid_type_def){0};

    control_pid_params_init();
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

// 无角速度环控制
void main_control_pid_without_angle_vel()
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

    float turn_diff = pid_turn_control_without_angle_vel(turn_err_target);
    float bottom_velocity_out = pid_bottom_control(bottom_velocity_target, vel_left + vel_right);

    // set pwm
    motor_set_left_pwm(bottom_velocity_out - turn_diff);
    motor_set_right_pwm(bottom_velocity_out + turn_diff);
}

// 开环速度控制
void main_control_without_vel(float lvel, float rvel)
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    float turn_err_target = 0;

    if (RA_out_cnt >= 200)
    {
        RA_flag_L = (left_side) ? 1 : RA_flag_L;
        RA_flag_R = (right_side) ? 1 : RA_flag_R;
    }

    if (!RA_flag_L && !RA_flag_R)
    {
        RA_out_cnt++;

        if (left_side && !right_side)
        {
            turn_err_target = weight_list[0] * left_side +
                              weight_list[1] * left +
                              weight_list[2] * mid;
        }
        else if (right_side && !left_side)
        {
            turn_err_target = weight_list[3] * right +
                              weight_list[4] * right_side +
                              weight_list[2] * mid;
        }
        else
        {
            turn_err_target = weight_list[0] * left_side +
                              weight_list[1] * left +
                              weight_list[2] * mid +
                              weight_list[3] * right +
                              weight_list[4] * right_side;
        }
    }
    else
    {
        RA_out_cnt = 0;

        turn_err_target = weight_list[0] * (int)RA_flag_L +
                          weight_list[4] * (int)RA_flag_R;
        RA_cnt++;
        if (RA_cnt >= RA_cnt_n)
        {
            RA_flag_R = 0;
            RA_flag_L = 0;
        }
    }

    // printf("%d, %d, %d, %d, %d\n",
    //        left_side, left, mid, right, right_side);
    // float turn_err_target = weight_list[0] * left_side +
    //                         weight_list[1] * left +
    //                         weight_list[2] * mid +
    //                         weight_list[3] * right +
    //                         weight_list[4] * right_side;

    float turn_diff = 0.9 * pid_turn_control_without_angle_vel(turn_err_target);

    motor_set_left_pwm(lvel - turn_diff);  // set left pwm
    motor_set_right_pwm(rvel + turn_diff); // set right pwm
}

// 正常循迹，开环转向
void main_control_tracking_open_turn()
{
    static int turning = 0;
    static int turn_cnt = 0;
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    if (!turning && control_check_turn())
    {
        turning = 1;
        turn_cnt = 0;
    }

    if (turning)
    {
        // 固定差速转弯
        motor_set_left_pwm(control_tracking_open_base_pwm - control_tracking_open_turn_diff);
        motor_set_right_pwm(control_tracking_open_base_pwm + control_tracking_open_turn_diff);
        turn_cnt++;
        if (turn_cnt >= control_tracking_open_turn_delay_times)
        {
            turning = 0;
        }
    }
    else
    {
        // 正常循迹
        float turn_diff = weight_list[0] * left_side +
                          weight_list[1] * left +
                          weight_list[2] * mid +
                          weight_list[3] * right +
                          weight_list[4] * right_side;
        motor_set_left_pwm(control_tracking_open_base_pwm - turn_diff);
        motor_set_right_pwm(control_tracking_open_base_pwm + turn_diff);
    }
}

void main_control_encoder_only()
{
    static int32 encoder_sum = 0;
    static int turning = 0;
    static int turn_cnt = 0;

    int16 vel_right = encoder_absolute_encoder_get_offset(0);
    int16 vel_left = encoder_absolute_encoder_get_offset(1);

    encoder_sum += vel_right + vel_left;

    if (!turning && encoder_sum >= control_only_encoder_target_distance)
    {
        turning = 1;
        turn_cnt = 0;
        encoder_sum = 0; // 重置累计
    }

    if (turning)
    {
        motor_set_left_pwm(control_only_encoder_base_pwm - control_only_encoder_diff);
        motor_set_right_pwm(control_only_encoder_base_pwm + control_only_encoder_diff);
        turn_cnt++;
        if (turn_cnt >= control_only_encoder_turn_delay_times)
        {
            turning = 0;
        }
    }
}

void main_control_pid_vel_only()
{
    // float vel_left = encoder_absolute_encoder_get_offset(0);
    // float vel_right = -encoder_absolute_encoder_get_offset(1);
    static float last_left_vel = 0;
    static float last_right_vel = 0;

    encoder_data_t right_encoder_data = encoder_read(0);
    encoder_data_t left_encoder_data = encoder_read(1);

    float vel_left = left_encoder_data.velocity;
    float vel_right = -right_encoder_data.velocity;

    if (vel_left < 0)
    {
        vel_left = last_left_vel;
    }
    else
    {
        last_left_vel = vel_left;
    }

    if (vel_right < 0)
    {
        vel_right = last_right_vel;
    }
    else
    {
        last_right_vel = vel_right;
    }

    // absolute_encoder_get_location(0);
    // absolute_encoder_get_location(1);

    // float vel_left = absolute_encoder_get_offset(1);
    // float vel_right = -absolute_encoder_get_offset(0);

    float vel_left_out = pid_bottom_control_left(left_vel_target, vel_left);
    float vel_right_out = pid_bottom_control_right(right_vel_target, vel_right);
    // printf("%.2f\n", vel_right);
    printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", vel_left_out, vel_right_out, vel_left, vel_right, left_vel_target, right_vel_target);

    motor_set_left_pwm(vel_left_out);
    motor_set_right_pwm(vel_right_out);
}

uint8 control_check_turn()
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    if (left_side)
    {
        return 1;
    }
    else if (right_side)
    {
        return 2;
    }
    return 0;
}

uint8 control_check_mid()
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    if (left || right || mid)
    {
        return 1;
    }
    return 0;
}

uint8 control_check_line()
{
    uint8 left_side = grey_tracking_get_status(GREY_LEFT_SIDE);
    uint8 left = grey_tracking_get_status(GREY_LEFT);
    uint8 mid = grey_tracking_get_status(GREY_MID);
    uint8 right = grey_tracking_get_status(GREY_RIGHT);
    uint8 right_side = grey_tracking_get_status(GREY_RIGHT_SIDE);

    if (left_side && left && mid && right && right_side)
    {
        return 1; // 有线
    }
    return 0; // 无线
}

void control_callback_func(uint32 event, void *ptr)
{
    *((uint8 *)ptr) = 1;
    // printf("test\n");
    // static int turning = 0;
    // static int turn_cnt = 0;
    // static int start_delay_cnt = 0;

    // // 启动延时阶段
    // if (start_delay_cnt < control_start_delay_time)
    // {
    //     main_control_without_vel(control_start_left_vel_base_pwm, control_start_right_vel_base_pwm);
    //     start_delay_cnt++;
    //     return;
    // }

    // if (!turning && control_check_turn())
    // {
    //     turning = 1;
    //     turn_cnt = 0;
    // }

    // if (turning)
    // {
    //     main_control_without_vel(0, 0); // 转弯控制
    //     turn_cnt++;
    //     if (turn_cnt >= control_without_vel_turn_delay)
    //     {
    //         turning = 0;
    //     }
    // }
    // else
    // {

    // static bool flag = 0;
    // static uint8 turn_dir = 0;
    // if (flag == 1)
    // {
    //     if (control_check_mid())
    //     {
    //         flag = 0;
    //     }
    //     else
    //     {
    //         if (turn_dir == 1)
    //         {
    //             // 左转
    //             float diff = pid_turn_control_without_angle_vel(weight_list[0]);
    //             motor_set_left_pwm(diff);
    //         }
    //         else if (turn_dir == 2)
    //         {
    //             // 右转
    //             float diff = pid_turn_control_without_angle_vel(weight_list[4]);
    //             motor_set_right_pwm(diff);
    //         }
    //     }
    // }

    // turn_dir = control_check_turn();
    // if (turn_dir)
    // {
    //     flag = 1;
    //     return;
    // }
    // if (control_check_line())
    // {
    //     float diff = pid_turn_control_without_angle_vel(weight_list[1]);
    //     motor_set_left_pwm(control_without_left_vel_base_pwm - diff);
    //     motor_set_right_pwm(control_without_right_vel_base_pwm + diff);
    //     return;
    // }
    main_control_without_vel(control_without_left_vel_base_pwm, control_without_right_vel_base_pwm); // 正常循迹

    // 无线状态，使用开环速度控制
    // main_control_pid_without_angle_vel();
    // }
}