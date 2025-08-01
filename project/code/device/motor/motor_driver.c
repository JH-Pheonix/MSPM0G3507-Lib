#include "motor_driver.h"

void motor_driver_init(void)
{
    pwm_init(MOTOR_DRIVER_LEFT_PWM, MOTOR_DRIVER_HZ, 0);
    gpio_init(MOTOR_DRIVER_LEFT_IN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_DRIVER_LEFT_IN2, GPO, 0, GPO_PUSH_PULL);

    pwm_init(MOTOR_DRIVER_RIGHT_PWM, MOTOR_DRIVER_HZ, 0);
    gpio_init(MOTOR_DRIVER_RIGHT_IN1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_DRIVER_RIGHT_IN2, GPO, 0, GPO_PUSH_PULL);
}

void motor_driver_set_left_pwm(int16 pwm)
{
    pwm = LIMIT(pwm, -MOTOR_DRIVER_MAX, MOTOR_DRIVER_MAX);
    if (pwm >= 0)
    {
        gpio_set_level(MOTOR_DRIVER_LEFT_IN1, 0);
        gpio_set_level(MOTOR_DRIVER_LEFT_IN2, 1);
        pwm_set_duty(MOTOR_DRIVER_LEFT_PWM, pwm);
    }
    else
    {
        gpio_set_level(MOTOR_DRIVER_LEFT_IN1, 1);
        gpio_set_level(MOTOR_DRIVER_LEFT_IN2, 0);
        pwm_set_duty(MOTOR_DRIVER_LEFT_PWM, -pwm);
    }
}

void motor_driver_set_right_pwm(int16 pwm)
{
    pwm = LIMIT(pwm, -MOTOR_DRIVER_MAX, MOTOR_DRIVER_MAX);
    if (pwm >= 0)
    {
        gpio_set_level(MOTOR_DRIVER_RIGHT_IN1, 0);
        gpio_set_level(MOTOR_DRIVER_RIGHT_IN2, 1);
        pwm_set_duty(MOTOR_DRIVER_RIGHT_PWM, pwm);
    }
    else
    {
        gpio_set_level(MOTOR_DRIVER_RIGHT_IN1, 1);
        gpio_set_level(MOTOR_DRIVER_RIGHT_IN2, 0);
        pwm_set_duty(MOTOR_DRIVER_RIGHT_PWM, -pwm);
    }
}

void motor_driver_stop(void)
{
    pwm_set_duty(MOTOR_DRIVER_LEFT_PWM, 0);

    gpio_set_level(MOTOR_DRIVER_LEFT_IN1, 0);
    gpio_set_level(MOTOR_DRIVER_LEFT_IN2, 0);

    pwm_set_duty(MOTOR_DRIVER_RIGHT_PWM, 0);
    gpio_set_level(MOTOR_DRIVER_RIGHT_IN1, 0);
    gpio_set_level(MOTOR_DRIVER_RIGHT_IN2, 0);
}