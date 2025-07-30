#ifndef _DEVICE_ZFMOTOR_H_
#define _DEVICE_ZFMOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR_HZ 15 * 1000 // 15kHz
#define MOTOR_HZ_RANGE 1000

#define MOTOR_MAX 9999

void motor_init();
void motor_set_left_pwm(int32 pwm);
void motor_set_right_pwm(int32 pwm);
void motor_stop();

#endif
