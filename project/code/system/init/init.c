#include "init.h"
#include "imu.h"
#include "key.h"
#include "grey_tracking.h"
#include "menu.h"
#include "lcd.h"
#include "encoder.h"

void system_init(void)
{
    lcd_init();
    lcd_show_string(0, 0, "System Init");
    // imu_init(IMU_DEVICE_660RA);
    // key_init_rewrite(KEY_NUM);
    // encoder_init(ENCODER_ABS);
    // grey_tracking_init(GREY_MID);
    // MainMenu_Set();
}