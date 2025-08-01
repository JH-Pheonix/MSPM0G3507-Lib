#include "init.h"
#include "imu.h"
#include "key.h"
#include "grey_tracking.h"
#include "menu.h"
#include "lcd.h"
#include "encoder.h"
#include "control.h"
#include "absolute_encoder.h"

volatile uint8_t control_pit_flag = 0;
volatile uint8_t key_pit_flag = 0;
volatile uint8_t encoder_pit_flag = 0;

void system_init(void)
{
    lcd_init();
    absolute_encoder_init(0);
    absolute_encoder_init(1);
    key_init_rewrite(KEY_NUM);
    grey_tracking_init(GREY_MID);

    pit_ms_init(PIT_TIM_G6, 5, key_callback_func, (void *)&key_pit_flag);
    pit_ms_init(PIT_TIM_G12, 5, encoder_absolute_encoder_callback_func, (void *)&encoder_pit_flag);

    MainMenu_Set();

    // pit_ms_init(PIT_TIM_G12, 1, control_callback_func, (void *)&control_pit_flag);
    // imu_init(IMU_DEVICE_660RA);
    // key_init_rewrite(KEY_NUM);
    // encoder_init(ENCODER_ABS);
    // grey_tracking_init(GREY_MID);
    // MainMenu_Set();
}