#ifndef _DEVICE_ENCODER_INTERRUPT_H_
#define _DEVICE_ENCODER_INTERRUPT_H_

#include "encoder.h"

typedef struct {
    gpio_pin_enum pin_a;
    gpio_pin_enum pin_b;
    volatile int32 counter;
    volatile uint8 last_state;
    volatile uint32 last_time;
} encoder_interrupt_state_t;

uint8 encoder_interrupt_init(uint8 encoder_index);
void encoder_interrupt_handler(uint8 encoder_index);
void encoder_interrupt_update(void);
encoder_data_t encoder_interrupt_read(uint8 encoder_index);
void encoder_interrupt_clear(uint8 encoder_index);

#endif
