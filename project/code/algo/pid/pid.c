#include "pid.h"

void pid_init(pid_type_def *pid,
              const float PID[3],
              float max_out,
              float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];

    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout =
        pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid calculate Position
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算 位置式PID
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
float PID_calc_Position(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}

float PID_calc_Position_Gyro_D(pid_type_def *pid, float ref, float set, float Dbuf)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dout = -Dbuf * pid->Kd;
    pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}

/**
 * @brief          pid calculate PID_DELTA
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算 增量式
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
float PID_calc_DELTA(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    // LimitMax(pid->Iout, pid->max_iout);
    return pid->out;
}

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

// float Speed_Control(float encoder,const float pid_para[3])
// {
//     static float error_i=0;
//     static float last_error=0;
//     float error;
//     error=encoder*0.1f;  //等效/10 乘法运算更快
//     error_i += error;
//     LimitMax(error_i,200);
//     float temp_out;
//     temp_out = error*pid_para[0] + error_i*pid_para[1];
//     temp_out*=0.01f;   //等效为 /100 乘法运算更快
//     LimitMax(temp_out,9999);
//     return temp_out;
// }

/**********************************************************************************
 * @brief  PID参数设置  快速更改pid参数
 * @param pid
 * @param PID
 *********************************************************************************/
void Set_Pid_Para(pid_type_def *pid, float PID[3])
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}

/**********************************************************************************
 * @brief  PID参数设置  快速更改pid限幅
 * @param pid
 * @param max_out  //最大输出限幅
 * @param max_iout //最大积分限幅
 *********************************************************************************/
void Set_Pid_Limit(pid_type_def *pid, float max_out, float max_iout)
{
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}

float PID_Realize_Curvature(pid_type_def *pid,
                            float NowPoint,
                            float TarPoint,
                            int32 speed)
{
    // 定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
    float iError,                 // 当前误差
        Actual;                   // 最后得出的实际输出值
    float Kp;                     // 动态P
    iError = TarPoint - NowPoint; // 计算当前误差
    Kp = pid->Kp;
    Actual = (float)(Kp * iError);
    return Actual;
}

/// @brief 位置式平方误差PID计算
/// @param pid
/// @param ref 参考值
/// @param set 实际值
/// @return
float PID_calc_Position_Square(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->error[0] >= 0)
    {
        pid->error[0] = pid->error[0] * pid->error[0];
    }
    else
    {
        pid->error[0] = -pid->error[0] * pid->error[0];
    }
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}
/// @brief 增量式平方误差PID计算
/// @param pid
/// @param ref 参考值
/// @param set 实际值
/// @return
float PID_calc_DELTA_Square(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->error[0] >= 0)
    {
        pid->error[0] = pid->error[0] * pid->error[0];
    }
    else
    {
        pid->error[0] = -pid->error[0] * pid->error[0];
    }
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
    return pid->out;
}

/// @brief 位置式变速积分PID计算
/// @param pid
/// @param ref
/// @param set
/// @return
float PID_calc_Position_DynamicI(pid_type_def *pid,
                                 float ref,
                                 float set,
                                 float range,
                                 float iMax)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    // pid->Ki = (pid->error[0] > range) ? 0.0f : pid->Ki;

    pid->Ki = (fabsf(pid->error[0]) > range)
                  ? 0.0f
                  : pid->Ki * (1 - fabsf(pid->error[0]) / range);

    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Iout = LIMIT(pid->Iout, -iMax, iMax);
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}
float PID_calc_Position_LowPassD(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * (0.5f) * pid->Dbuf[0] + 0.5f * pid->Dbuf[1];
    pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = LIMIT(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}