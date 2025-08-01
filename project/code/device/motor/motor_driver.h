#ifndef _DEVICE_MOTOR_DRIVER_H_
#define _DEVICE_MOTOR_DRIVER_H_

#include "zf_common_headfile.h"

#define MOTOR_DRIVER_HZ 15 * 1000 // 15kHz
#define MOTOR_DRIVER_MAX 1000

void motor_driver_init(void);
void motor_driver_set_left_pwm(int16 pwm);
void motor_driver_set_right_pwm(int16 pwm);
void motor_driver_stop(void);

#endif