#ifndef _COMMON_H_
#define _COMMON_H_

#include "zf_common_headfile.h"

#define LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))


typedef struct 
{
    float x;
    float y;
} point_2d_f_t;

typedef struct 
{
    int32 x; // 俯仰角
    int32 y;   // 偏航角
} point_2d_i32_t;

typedef struct {
    float pan;
    float tilt;
} gimbal_angle_2d_t;


#endif