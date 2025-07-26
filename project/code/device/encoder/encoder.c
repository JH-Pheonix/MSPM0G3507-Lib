#include "encoder.h"
#include "pin.h"
#include "absolute_encoder.h"
#include "interrupt.h"

static bool encoder_state = false;

// 编码器引脚
static gpio_pin_enum encoder_pins[MAX_ENCODER_COUNT][2] = {
    {ENCODER_1_PIN},
    {ENCODER_2_PIN},
    {ENCODER_3_PIN},
    {ENCODER_4_PIN}
};

static encoder_mode_t curr_mode;

void encoder_get_pin(uint8 encoder_index, gpio_pin_enum *pin_a, gpio_pin_enum *pin_b) {
    if (encoder_index < MAX_ENCODER_COUNT) {
        *pin_a = encoder_pins[encoder_index][0];
        *pin_b = encoder_pins[encoder_index][1];
    } else {
        *pin_a = A0; // 默认值
        *pin_b = A1; // 默认值
    }
}

void encoder_init(encoder_mode_t mode) {
    curr_mode = mode;
    for(uint8 i = 0; i < MAX_ENCODER_COUNT; i++) {
        switch (curr_mode) {
        case ENCODER_ABS:
            encoder_state = encoder_absolute_encoder_init(i);
            break;
        case ENCODER_INCR:
            encoder_state = encoder_interrupt_init(i);
            break;
        default:
            break;
        }
    }
}
