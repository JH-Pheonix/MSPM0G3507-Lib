#include "interrupt.h"
#include "encoder.h"
#include "pin.h"

// 批处理标志
static volatile uint32 encoder_flags = 0;
static encoder_interrupt_state_t int_encoders[MAX_ENCODER_COUNT] = {
    {ENCODER_1_PIN, 0, 0, 0},
    {ENCODER_2_PIN, 0, 0, 0},
    {ENCODER_3_PIN, 0, 0, 0},
    {ENCODER_4_PIN, 0, 0, 0}
};

static const int8 state_table[16] = {
    0, -1, 1, 0,    // 00->xx
    1, 0, 0, -1,    // 01->xx  
    -1, 0, 0, 1,    // 10->xx
    0, 1, -1, 0     // 11->xx
};
static encoder_data_t encoder_data[MAX_ENCODER_COUNT];
static encoder_state_t encoder_state[MAX_ENCODER_COUNT];

uint8 encoder_interrupt_init(uint8 encoder_index) {
    if(encoder_index >= MAX_ENCODER_COUNT) return 1;

    encoder_interrupt_state_t* enc = &int_encoders[encoder_index];

    // 配置GPIO（启用滤波功能）
    gpio_init(enc->pin_a, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(enc->pin_b, GPI, GPIO_LOW, GPI_PULL_UP);

    // 只在A相配置中断，降低中断频率
    exti_init(enc->pin_a, EXTI_TRIGGER_BOTH, NULL, NULL);

    uint8 state_a = gpio_get_level(enc->pin_a);
    uint8 state_b = gpio_get_level(enc->pin_b);

    enc->last_state = (state_a << 1) | state_b;
    enc->last_time = 0; // 初始化为0，使用系统延时代替时间戳
    
    encoder_state[encoder_index].initialized = 1;
    return 0;
}

// 中断处理函数（在外部中断中调用）
void encoder_interrupt_handler(uint8 encoder_index) {
    if(encoder_index >= MAX_ENCODER_COUNT) return;
    
    // 设置处理标志，延迟到主循环处理
    encoder_flags |= (1 << encoder_index);
}

// 批处理更新（在主循环中定期调用）
void encoder_interrupt_update(void) {
    if(encoder_flags == 0) return;
    
    for(uint8 i = 0; i < MAX_ENCODER_COUNT; i++) {
        if(!(encoder_flags & (1 << i))) continue;
        
        encoder_interrupt_state_t *enc = &int_encoders[i];
        
        // 读取当前状态
        uint8 state_a = gpio_get_level(enc->pin_a);
        uint8 state_b = gpio_get_level(enc->pin_b);
        uint8 current_state = (state_a << 1) | state_b;
        
        // 状态表解码
        uint8 table_index = (enc->last_state << 2) | current_state;
        int8 delta = state_table[table_index];
        
        if(delta != 0) {
            int32 old_position = encoder_data[i].position;
            enc->counter += delta;
            encoder_data[i].position = enc->counter;
            
            // 简化速度计算，直接使用位置变化量
            encoder_data[i].velocity = encoder_data[i].position - old_position;
            
        } else if(enc->last_state != current_state) {
            // 错误状态转换
            encoder_state[i].error_count++;
        }
        
        enc->last_state = current_state;
    }
    
    encoder_flags = 0;
}

encoder_data_t encoder_interrupt_read(uint8 encoder_index) {
    if(encoder_index >= MAX_ENCODER_COUNT) return (encoder_data_t){0, 0};
    return encoder_data[encoder_index];
}

void encoder_interrupt_clear(uint8 encoder_index) {
    if(encoder_index >= MAX_ENCODER_COUNT) return;
    
    int_encoders[encoder_index].counter = 0;
    encoder_data[encoder_index].position = 0;
    encoder_data[encoder_index].velocity = 0;
    encoder_state[encoder_index].error_count = 0;
}