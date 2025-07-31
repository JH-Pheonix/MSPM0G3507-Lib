#ifndef _DEVICE_ADC_H_
#define _DEVICE_ADC_H_

#include "zf_common_headfile.h"

typedef enum
{
    MADC_NAME,
    MADC_NUM,
} madc_e;

void madc_init(madc_e channel);
uint16_t madc_read(madc_e channel);

#endif
