#include "test.h"
#include "key.h"
#include "zfmotor.h"
#include "imu.h"
#include "lcd.h"
#include "encoder.h"
#include "grey_tracking.h"
#include "zf_device_absolute_encoder copy.h"
#include "absolute_encoder.h"
#include "madc.h"

// volatile uint8_t pit_flag = 0;

// void test_key()
// {
//     lcd_clear();

//     while (1)
//     {
//         lcd_show_string(0, 2, "Key pressed:");
//         lcd_show_int(10, 2, keymsg.key, 2);
//         lcd_show_string(0, 3, "Status:");
//         lcd_show_int(10, 3, keymsg.status, 2);

//         system_delay_ms(1); // 减少刷新频率
//     }
//     lcd_clear();
// }

// void test_motor()
// {
//     motor_init();
//     motor_set_left_pwm(4000);
//     motor_set_right_pwm(2000);
// }

// void test_imu()
// {
//     imu_init(IMU_DEVICE_660RA);
//     while (1)
//     {
//         imu_data_t data = imu_get_data();
//         lcd_show_float(0, 0, data.accel_x, 3, 3);
//         lcd_show_float(0, 1, data.accel_y, 3, 3);
//         lcd_show_float(0, 2, data.accel_z, 3, 3);
//         lcd_show_float(0, 3, data.gyro_x, 3, 3);
//         lcd_show_float(0, 4, data.gyro_y, 3, 3);
//         lcd_show_float(0, 5, data.gyro_z, 3, 3);
//     }
// }

// void test_encoder()
// {
//     // if (absolute_encoder_init(0)) // 初始化4个编码器
//     // {

//     //     lcd_show_string(0, 1, "failed");
//     // }
//     // else
//     // {
//     //     lcd_show_string(0, 1, "successfully");
//     // }

//     // if (absolute_encoder1_init(0)) // 初始化4个编码器
//     // {

//     //     lcd_show_string(0, 2, "failed");
//     // }
//     // else
//     // {
//     //     lcd_show_string(0, 2, "successfully");
//     // }
//     // encoder_init(ENCODER_ABS);
//     absolute_encoder_init(0);
//     absolute_encoder_init(1);
//     pit_ms_init(PIT_TIM_G12, 5, encoder_absolute_encoder_pit_handler, (void *)&pit_flag);
//     while (1)
//     {
//         system_delay_ms(10);
//         encoder_data_t data1 = encoder_read(0);
//         lcd_show_float(0, 0, data1.position, 3, 3);
//         lcd_show_float(0, 1, -data1.velocity, 3, 3);
//         encoder_data_t data2 = encoder_read(1);
//         lcd_show_float(0, 2, data2.position, 3, 3);
//         lcd_show_float(0, 3, data2.velocity, 3, 3);
//         // printf("Encoder 1: Position: %.2f, Velocity: %.2f\n", data1.position, data1.velocity);
//         // printf("Encoder 1: Position: %.2f, Velocity: %.2f, Encoder 2: Position: %.2f, Velocity: %.2f\n", data1.position, data1.velocity, data2.position, data2.velocity);
//     }
// }

// void test_grey()
// {
//     grey_tracking_init(GREY_NUM);
//     while (1)
//     {
//         lcd_clear();

//         lcd_show_uint(0, 1, grey_tracking_get_status(GREY_LEFT_SIDE), 1);
//         lcd_show_uint(0, 2, grey_tracking_get_status(GREY_LEFT), 1);
//         lcd_show_uint(0, 3, grey_tracking_get_status(GREY_MID), 1);
//         lcd_show_uint(0, 4, grey_tracking_get_status(GREY_RIGHT), 1);
//         lcd_show_uint(0, 5, grey_tracking_get_status(GREY_RIGHT_SIDE), 1);

//         system_delay_ms(50);
//     }
// }

// void test_adc_grey()
// {
//     madc_init(MADC_NUM);
//     while (1)
//     {
//         lcd_clear();
//         // printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
//         //        madc_read(MADC_1), madc_read(MADC_2), 0,
//         //        madc_read(MADC_4), madc_read(MADC_5), madc_read(MADC_6),
//         //        madc_read(MADC_7), madc_read(MADC_8));
//         lcd_show_int(0, 0, madc_read(MADC_1), 4);
//         lcd_show_int(0, 1, madc_read(MADC_2), 4);
//         lcd_show_int(0, 2, madc_read(MADC_3), 4);
//         lcd_show_int(0, 3, madc_read(MADC_4), 4);
//         lcd_show_int(0, 4, madc_read(MADC_5), 4);
//         lcd_show_int(0, 5, madc_read(MADC_6), 4);
//         lcd_show_int(0, 6, madc_read(MADC_7), 4);
//         lcd_show_int(0, 7, madc_read(MADC_8), 4);

//         system_delay_ms(50);
//     }
// }

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

    lcd_clear();
    // system_delay_ms(200);

    static uint16 cnt = 0;
    uint8 mode = 0;
    while (1)
    {
        // cnt++;
        // if (cnt >= 1000)
        // {
        //     cnt = 0;
        //     mode++;
        //     if (mode > 3)
        //         mode = 0;
        // }

        // switch (mode)
        // {
        // case 0: // 左电机向前
        //     motor_set_right_pwm(2000);
        //     motor_set_left_pwm(0);
        //     break;
        // case 1: // 左电机向后
        //     motor_set_right_pwm(-2000);
        //     motor_set_left_pwm(0);
        //     break;
        // case 2: // 右电机向前
        //     motor_set_left_pwm(2000);
        //     motor_set_right_pwm(0);
        //     break;
        // case 3: // 右电机向后
        //     motor_set_left_pwm(-2000);
        //     motor_set_right_pwm(0);
        //     break;
        // }
        // lcd_clear();
        motor_set_right_pwm(1500);
        motor_set_left_pwm(1500);
        encoder_data_t data1 = encoder_read(0);
        encoder_data_t data2 = encoder_read(1);
        printf("Encoder 1: Position: %.2f, Velocity: %.2f, Encoder 2: Position: %.2f, Velocity: %.2f\n",
               data1.position, data1.velocity, data2.position, data2.velocity);
        // lcd_show_float(0, 0, data1.position, 3, 3);
        // lcd_show_float(0, 1, -data1.velocity, 3, 3);
        // lcd_show_float(0, 2, data2.position, 3, 3);
        // lcd_show_float(0, 3, data2.velocity, 3, 3);

        system_delay_ms(50);
    }

    motor_set_left_pwm(0);
    motor_set_right_pwm(0);
    lcd_clear();
}

void test_grey()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        // lcd_show_uint(0, 1, grey_tracking_get_status(GREY_LEFT_SIDE), 1);
        // lcd_show_uint(0, 2, grey_tracking_get_status(GREY_LEFT), 1);
        // lcd_show_uint(0, 3, grey_tracking_get_status(GREY_MID), 1);
        // lcd_show_uint(0, 4, grey_tracking_get_status(GREY_RIGHT), 1);
        // lcd_show_uint(0, 5, grey_tracking_get_status(GREY_RIGHT_SIDE), 1);

        printf("Grey Left Side: %d, Grey Left: %d, Grey Mid: %d, Grey Right: %d, Grey Right Side: %d\n",
               grey_tracking_get_status(GREY_LEFT_SIDE),
               grey_tracking_get_status(GREY_LEFT),
               grey_tracking_get_status(GREY_MID),
               grey_tracking_get_status(GREY_RIGHT),
               grey_tracking_get_status(GREY_RIGHT_SIDE));
        system_delay_ms(20);
    }
}

// void test_adc_grey()
// {
//     lcd_clear();
//     madc_init(MADC_NUM);
//     while (keymsg.key != KEY_L)
//     {
//         lcd_clear();
//         // printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
//         //        madc_read(MADC_1), madc_read(MADC_2), 0,
//         //        madc_read(MADC_4), madc_read(MADC_5), madc_read(MADC_6),
//         //        madc_read(MADC_7), madc_read(MADC_8));
//         lcd_show_int(0, 0, madc_read(MADC_1), 4);
//         lcd_show_int(0, 1, madc_read(MADC_2), 4);
//         lcd_show_int(0, 2, madc_read(MADC_3), 4);
//         lcd_show_int(0, 3, madc_read(MADC_4), 4);
//         lcd_show_int(0, 4, madc_read(MADC_5), 4);
//         lcd_show_int(0, 5, madc_read(MADC_6), 4);
//         lcd_show_int(0, 6, madc_read(MADC_7), 4);
//         lcd_show_int(0, 7, madc_read(MADC_8), 4);

//         system_delay_ms(50);
//     }
// }