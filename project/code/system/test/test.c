#include "test.h"
#include "key.h"
#include "zfmotor.h"
#include "imu.h"
#include "lcd.h"

void test_key()
{
    lcd_clear();

    while (1)
    {
        lcd_show_string(0, 2, "Key pressed:");
        lcd_show_int(10, 2, keymsg.key, 2);
        lcd_show_string(0, 3, "Status:");
        lcd_show_int(10, 3, keymsg.status, 2);

        system_delay_ms(1); // 减少刷新频率
    }
    lcd_clear();
}

void test_motor()
{
    motor_init();
    motor_set_left_pwm(1000);
    motor_set_right_pwm(1000);
}

void test_imu()
{
    imu_init(IMU_DEVICE_660RA);
    while (1)
    {
        imu_data_t data = imu_get_data();
        lcd_show_float(0, 0, data.accel_x, 3, 3);
        lcd_show_float(0, 1, data.accel_y, 3, 3);
        lcd_show_float(0, 2, data.accel_z, 3, 3);
        lcd_show_float(0, 3, data.gyro_x, 3, 3);
        lcd_show_float(0, 4, data.gyro_y, 3, 3);
        lcd_show_float(0, 5, data.gyro_z, 3, 3);
    }
}