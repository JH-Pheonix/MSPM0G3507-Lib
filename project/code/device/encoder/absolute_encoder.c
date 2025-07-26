#include "absolute_encoder.h"

int16 location_data[MAX_ENCODER_COUNT] = {0, 0, 0, 0};
int16 offset_data[MAX_ENCODER_COUNT]   = {0, 0, 0, 0};

uint8 encoder_absolute_encoder_init(uint8 encoder_index) {
    return absolute_encoder_init(encoder_index);
}

void encoder_absolute_encoder_pit_handler(uint32 event, void *ptr) {
    *((uint8 *)ptr)  = 1;

    for(uint8 i = 0; i < MAX_ENCODER_COUNT;i++) {
        // 获取编码器当前大角度信息
        location_data[i] = absolute_encoder_get_location(i);
        // 通过两次角度对比得到当前的旋转速度
        offset_data[i] = absolute_encoder_get_offset(i);
    }
}

encoder_data_t encoder_absolute_encoder_read(uint8 encoder_index) {
    encoder_data_t data = {0, 0};

    if (encoder_index >= MAX_ENCODER_COUNT) {
        return data; // 返回默认值
    }

    // 获取当前编码器位置
    data.position = (float)location_data[encoder_index];
    
    // 获取当前编码器速度
    data.velocity = (float)offset_data[encoder_index];

    return data;
}