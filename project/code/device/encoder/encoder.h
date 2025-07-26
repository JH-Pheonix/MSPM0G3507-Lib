#ifndef _DEVICE_ENCODER_H_
#define _DEVICE_ENCODER_H_

#include "zf_common_headfile.h"

// 由于msp芯片条件限制，我们只有一个TIMG8支持QEI模式
// 因此我们这里如果使用多增量式编码器的时候就需要使用其他的方案来实现。这里提供三种方案
// 方案一：LS7366R硬件编码器芯片（我们手头目前暂时没有，我也就懒得写了）
// 方案二：优化的软件中断方案
// 方案三：混合方案（QEI + 其他方案）

#define SOLUTION_LS7366R 0
#define SOLUTION_INTERRUPT 1
#define SOLUTION_MIXED 0

#define MAX_ENCODER_COUNT 4
#define ENCODER_RESOLUTION 1024  // 编码器线数
  
typedef struct {
    float position;
    float velocity;
} encoder_data_t;

typedef struct {
    uint32 error_count;     // 错误计数
    uint8 initialized;      // 初始化标志
} encoder_state_t;

typedef enum {
    ENCODER_ABS, 
    ENCODER_INCR,
} encoder_mode_t;

#endif
