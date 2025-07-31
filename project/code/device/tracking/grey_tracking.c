#include "grey_tracking.h"

static const gpio_pin_enum GREY_PTxn[GREY_NUM] = {GREY_TRACKING_LIST};
uint8 grey_tracking_msg[GREY_NUM] = {0};

void grey_tracking_init(grey_tracking_e pin)
{
    if (pin < GREY_NUM)
    {
        gpio_init(GREY_PTxn[pin], GPI, 0, GPO_PUSH_PULL);
    }
    else
    {
        pin = GREY_NUM;
        while (pin--)
        {
            gpio_init(GREY_PTxn[pin], GPI, 0, GPO_PUSH_PULL);
        }
    }
}

uint8 grey_tracking_get_status(grey_tracking_e pin)
{
    if (pin < GREY_NUM)
    {
        return gpio_get_level(GREY_PTxn[pin]);
    }
    return 0;
}

// 扫描函数
void grey_tracking_scan(void)
{
    for (grey_tracking_e pin = 0; pin < GREY_NUM; pin++)
    {
        grey_tracking_msg[pin] = grey_tracking_get_status(pin);
    }
}