#include "madc.h"

static const adc_pin_enum ADC_PTxn[MADC_NUM] = {MADC_LIST};

void madc_init(madc_e channel)
{
    if (channel < MADC_NUM)
    {
        gpio_init(ADC_PTxn[channel], GPI, 0, GPO_PUSH_PULL);
    }
    else
    {
        channel = MADC_NUM;
        while (channel--)
        {
            adc_init(ADC_PTxn[channel], ADC_8BIT);
        }
    }
}

uint16 madc_read(madc_e channel)
{
    if (channel < MADC_NUM)
    {
        return adc_convert(ADC_PTxn[channel]);
    }
    return 0;
}