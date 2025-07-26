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
