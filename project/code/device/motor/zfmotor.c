#include "zfmotor.h"
#include "pin.h"

void motor_init()
{
    pwm_init(MOTOR_LEFT, MOTOR_HZ, 0);
    gpio_init(MOTOR_LEFT_DIR, GPO, 1, GPO_PUSH_PULL);

    pwm_init(MOTOR_RIGHT, MOTOR_HZ, 0);
    gpio_init(MOTOR_RIGHT_DIR, GPO, 1, GPO_PUSH_PULL);
}

void motor_set_left_pwm(int32 pwm)
{
    pwm = LIMIT(pwm, -MOTOR_MAX, MOTOR_MAX);
    if (pwm >= 0)
    {
        gpio_set_level(MOTOR_LEFT_DIR, 0);
        pwm_set_duty(MOTOR_LEFT, pwm);
    }
    else
    {
        gpio_set_level(MOTOR_LEFT_DIR, 1);
        pwm_set_duty(MOTOR_LEFT, -pwm);
    }
}

void motor_set_right_pwm(int32 pwm)
{
    pwm = LIMIT(pwm, -MOTOR_MAX, MOTOR_MAX);
    if (pwm >= 0)
    {
        gpio_set_level(MOTOR_RIGHT_DIR, 0);
        pwm_set_duty(MOTOR_RIGHT, pwm);
    }
    else
    {
        gpio_set_level(MOTOR_RIGHT_DIR, 1);
        pwm_set_duty(MOTOR_RIGHT, -pwm);
    }
}

void motor_stop()
{
    pwm_set_duty(MOTOR_LEFT, 0);
    pwm_set_duty(MOTOR_RIGHT, 0);
    gpio_set_level(MOTOR_LEFT_DIR, 1);
    gpio_set_level(MOTOR_RIGHT_DIR, 1);
}