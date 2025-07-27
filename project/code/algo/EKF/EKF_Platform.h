/**
 ******************************************************************************
 * @file    ekf platform.h
 * @author  xiang yankai  项彦凯
 * @version V1.2.2
 * @date    2024/2/28
 * @brief   当时尝试的 没想到成功了
 ******************************************************************************
 * @attention
 * 如果智能车感觉好用 欢迎支持支持我
 ******************************************************************************
 */
#ifndef __EKF_Platform_H
#define __EKF_Platform_H

//============================================================================
#include "zf_common_headfile.h"
// #include "arm_math.h"  // ARM DSP库暂时不使用，不知道为什么会报错
#include "matrix.h"   //arm平台的不需要引用 直接使用dsp库的 我自己实现的矩阵库
#include <math.h>     // 使用标准数学库

//这个是针对不同平台的三角函数 下面的是英飞凌的 其他大家自己补充
//=============================================================================

//==========================================================================
//平台相关的数学函数定义 - 统一使用标准数学库
#define arm_cos_f32(x)      cosf(x)
#define arm_sin_f32(x)      sinf(x)
#define arm_sqrt_f32(x)     sqrtf(x)
#define arm_atan2_f32(y,x)  atan2f(y,x)

//====================================================================================

//=====================================================================
//下面是矩阵库映射，使用matrix.h中定义的函数
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
//==================================================================================================================
#endif //__EKF_Platform_H
