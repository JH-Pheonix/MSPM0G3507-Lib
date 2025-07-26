#include "encoder.h"
#include "pin.h"
#include "absolute_encoder.h"
#include "interrupt.h"

static bool encoder_state = false;
static encoder_mode_t curr_mode;

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

encoder_data_t encoder_read(uint8 encoder_index) {
    encoder_data_t data = {0, 0};

    if (encoder_index >= MAX_ENCODER_COUNT) {
        return data; // 返回默认值
    }

    switch (curr_mode) {
    case ENCODER_ABS:
        data = encoder_absolute_encoder_read(encoder_index);
        break;
    case ENCODER_INCR:
        data = encoder_interrupt_read(encoder_index);
        break;
    default:
        break;
    }

    return data;
}

void encoder_clear(uint8 encoder_index) {
    if (encoder_index >= MAX_ENCODER_COUNT) return;

    switch (curr_mode) {
    case ENCODER_ABS:
        // 这里只有增量式编码器需要清除
        break;
    case ENCODER_INCR:
        encoder_interrupt_clear(encoder_index);
        break;
    default:
        break;
    }
}