#ifndef _ALGO_PID_H_
#define _ALGO_PID_H_

#include "zf_common_headfile.h"

typedef struct PIDparam_st
{
    uint32 P;
    uint32 I;
    uint32 D;
    uint32 T;
    uint32 Coeff;
    float ki;
} PIDparam_st;

typedef struct
{
    float SumError;
    int32 LastError;
    int32 PrevError;
    int32 LastData;
} PIDcal_st;

typedef struct
{
    // PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    float error[3]; // 误差项 0最新 1上一次 2上上次

} pid_type_def;

void pid_init(pid_type_def *pid,
              const float PID[3],
              float max_out,
              float max_iout);

float PID_calc_Position(pid_type_def *pid, float ref, float set);

float PID_calc_DELTA(pid_type_def *pid, float ref, float set);

float PID_calc_Position_Gyro_D(pid_type_def *pid, float ref, float set, float Dbuf);

void PID_clear(pid_type_def *pid);

static inline void Change_Pid_Para(pid_type_def *pid, float *pid_para)
{
    pid->Kp = pid_para[0];
    pid->Ki = pid_para[1];
    pid->Kd = pid_para[2];
}

void Set_Pid_Limit(pid_type_def *pid, float max_out, float max_iout);
void Set_Pid_Para(pid_type_def *pid, float PID[3]);

float PID_Realize_Curvature(pid_type_def *pid,
                            float NowPiont,
                            float TarPoint,
                            int32 speed);

float Speed_Control_turn(float encoder, const float pid_para[3]);

float PID_calc_Position_LowPassD(pid_type_def *pid, float ref, float set);
float PID_calc_Position_DynamicI(pid_type_def *pid,
                                 float ref,
                                 float set,
                                 float range,
                                 float iMax);
#endif