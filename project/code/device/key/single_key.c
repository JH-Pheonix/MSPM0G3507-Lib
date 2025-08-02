#include "single_key.h"
#include "pin.h"

static uint32 single_key_press_cnt = 0;

uint32 single_key_get_cnt()
{
    return single_key_press_cnt;
}

void single_key_init()
{
    gpio_init(SINGLE_KEY_PIN, GPI, 0, GPI_PULL_UP);
}

void single_key_callback_func()
{
    // 检测按键，直到按键被释放，计数加一
    if (gpio_get_level(SINGLE_KEY_PIN) == 0)
    {
        system_delay_ms(20);
        if (gpio_get_level(SINGLE_KEY_PIN) == 0)
        {
            while (gpio_get_level(SINGLE_KEY_PIN) == 0)
            {
            }
            single_key_press_cnt++;
        }
    }
}