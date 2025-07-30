#include "init.h"
#include "imu.h"
#include "key.h"
#include "grey_tracking.h"
#include "menu.h"
#include "lcd.h"

void system_init(void)
{
    lcd_init();
    imu_init(IMU_DEVICE_660RA);
    key_init_rewrite(KEY_NUM);
    grey_tracking_init(GREY_MID);
    MainMenu_Set();
}