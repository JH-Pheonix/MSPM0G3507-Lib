#include "absolute_encoder.h"

int16 location_data[ENCODER_CNT] = {0};
int16 offset_data[ENCODER_CNT] = {0};

bool encoder_absolute_encoder_init(uint8 encoder_index)
{
    return absolute_encoder_init(encoder_index) == 0 ? true : false;
}

void encoder_absolute_encoder_callback_func(uint32 event, void *ptr)
{
    *((uint8 *)ptr) = 1;

    for (uint8 i = 0; i < ENCODER_CNT; i++)
    {
        // 获取编码器当前大角度信息
        location_data[i] = absolute_encoder_get_location(i);
        // 通过两次角度对比得到当前的旋转速度
        offset_data[i] = absolute_encoder_get_offset_fix(i);
    }
}

int16 encoder_absolute_encoder_get_offset(uint8 index)
{
    if (index >= ENCODER_CNT)
        return 0;
    return absolute_encoder_get_offset(index);
}

encoder_data_t encoder_absolute_encoder_read(uint8 encoder_index)
{
    encoder_data_t data = {0, 0};

    if (encoder_index >= ENCODER_CNT)
    {
        return data; // 返回默认值
    }

    // 获取当前编码器位置
    data.position = (float)location_data[encoder_index];

    // 获取当前编码器速度
    data.velocity = (float)offset_data[encoder_index];

    return data;
}