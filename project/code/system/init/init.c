#include "init.h"
#include "imu.h"
#include "key.h"
#include "grey_tracking.h"
#include "menu.h"
#include "lcd.h"
#include "encoder.h"
#include "control.h"

volatile uint8_t control_pit_flag = 0;

void system_init(void)
{
    lcd_init();
    absolute_encoder_init(0);
    absolute_encoder_init(1);
    pit_ms_init(PIT_TIM_G12, 1, control_callback_func, (void *)&control_pit_flag);
    // imu_init(IMU_DEVICE_660RA);
    // key_init_rewrite(KEY_NUM);
    // encoder_init(ENCODER_ABS);
    // grey_tracking_init(GREY_MID);
    // MainMenu_Set();
}