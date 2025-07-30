#ifndef _DEVICE_GREY_TRACKING_H_
#define _DEVICE_GREY_TRACKING_H_

#include "zf_common_headfile.h"

typedef enum
{
    GREY_TRACKING_NAME,
    GREY_NUM
} grey_tracking_e;

typedef struct {
    grey_tracking_e pin;
    uint8 status;
} grey_tracking_msg_t;

void grey_tracking_init(grey_tracking_e pin);
uint8 grey_tracking_get_status(grey_tracking_e pin);
void grey_tracking_scan(void);

#endif

