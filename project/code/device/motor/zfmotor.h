#ifndef _DEVICE_ZFMOTOR_H_
#define _DEVICE_ZFMOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR_HZ 15 * 1000 // 15kHz
#define MOTOR_HZ_RANGE 1000

#define MOTOR_MAX 9999

void motor_init();
void set_left_motor_pwm(int32 pwm);
void set_right_motor_pwm(int32 pwm);
void stop_motor();

#endif
