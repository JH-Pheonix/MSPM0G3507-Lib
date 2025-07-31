#ifndef _DEVICE_ENCODER_ABSOLUTE_H_
#define _DEVICE_ENCODER_ABSOLUTE_H_

#include "encoder.h"

bool encoder_absolute_encoder_init(uint8 encoder_index);
void encoder_absolute_encoder_pit_handler(uint32 event, void *ptr);
encoder_data_t encoder_absolute_encoder_read(uint8 encoder_index);
int16 encoder_absolute_encoder_get_offset(uint8 index);

#endif // _DEVICE_ENCODER_ABSOLUTE_H_
