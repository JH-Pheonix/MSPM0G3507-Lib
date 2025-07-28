#ifndef _ALGO_INS_H_
#define _ALGO_INS_H_

#include "zf_common_headfile.h"
#include "../device/imu/imu.h"
#include "../device/encoder/encoder.h"
#include <math.h>
#include <stdbool.h>
#include <float.h>

// 数学常数定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * ======= 纯跟踪算法可调参数区域 =======
 */
// 最近点查找范围（前后各多少个点）
#define LOOKAHEAD_SEARCH_RANGE 30

// 预瞄点最大搜索步数（从最近点向前搜索的最大点数）
#define LOOKAHEAD_MAX_STEPS 20

// 跳跃步长估算的平均点间距（单位：cm）
#define LOOKAHEAD_AVG_DIST 1.0f

// 输出差速的经验比例系数
#define SCALE_FACTOR 80

// 小车物理参数
#define DISTANCE_INTERVAL 1.0f  // 每间隔1cm记录一次点位
#define WHEELBASE 17            // 轮距（左右轮距离，单位cm）
#define WHEEL_BASE_CM 17        // 轮距（用于打滑检测）

// 存储路径点的最大数量
#define MAX_PATH_POINTS (1024 * 6)

// 编码器相关参数
#define ENCODER_PULSES_PER_REVOLUTION 366  // 每厘米脉冲数
#define SAMPLING_INTERVAL_MS 5             // 采样间隔（毫秒）

// 打滑检测阈值
#define MIN_LATERAL_DETECT_SPEED_CMPS 5.0f        // 横向打滑检测最小速度（厘米/秒）
#define MIN_LONGITUDINAL_DETECT_SPEED_CMPS 5.0f   // 纵向打滑检测最小速度（厘米/秒）
#define LATERAL_SLIP_YAW_RATE_THRESHOLD 0.5f      // 横向打滑偏航率阈值（弧度/秒）
#define LONGITUDINAL_ACCEL_DIFF_THRESHOLD 0.3f    // 纵向打滑加速度差异阈值（米/秒²）

/**
 * 打滑标志位枚举
 */
typedef enum {
    SLIP_FLAG_NONE = 0x00,                  // 无打滑
    SLIP_FLAG_LATERAL_LEFT = 0x01,          // 左轮横向打滑
    SLIP_FLAG_LATERAL_RIGHT = 0x02,         // 右轮横向打滑
    SLIP_FLAG_LONGITUDINAL_ACCEL = 0x04,    // 纵向加速打滑
    SLIP_FLAG_LONGITUDINAL_DECEL = 0x08,    // 纵向减速打滑
} SlipFlags;

/**
 * 轮子数据结构体，封装单个轮子的编码器相关数据
 */
typedef struct {
    int32_t total_pulses;        // 累计脉冲数，用于虚拟清零功能
    int32_t last_total_pulses;   // 上一次的累计脉冲数
    int16_t encoder_last;        // 上一次读取的编码器值，用于计算增量
    int16_t encoder_prev_speed;  // 速度计算基准值，保存上一次中断时的编码器值
    int32_t target_pulses;       // 目标脉冲数，达到此值后触发虚拟清零
} WheelData;

/**
 * 惯导系统状态结构体
 */
typedef struct {
    // 位置信息（东天北坐标系，单位：cm）
    float x_position;
    float y_position;
    
    // 姿态信息
    float yaw_angle;        // 偏航角（弧度）
    
    // IMU数据
    imu_data_t imu_data;
    
    // 编码器速度（脉冲/采样周期）
    int32_t speed_left;
    int32_t speed_right;
    
    // 轮速信息（厘米/秒）
    float left_wheel_speed_cmps;
    float right_wheel_speed_cmps;
    float current_avg_speed_cmps;
    
    // 加速度积分速度（米/秒）
    float acc_y_speed;
    
    // 打滑检测标志
    SlipFlags wheel_slip_flags;
    
    // 轮子数据
    WheelData wheel_left;
    WheelData wheel_right;
    
    // 控制参数
    int16_t err_guandao_plus;    // 惯导差速
    float e_lat;                 // 惯导横向误差
    int locate_index;            // 小车定位点
    
} ins_system_t;

/**
 * 路径记忆系统结构体
 */
typedef struct {
    // 路径存储数组
    float x_memory[MAX_PATH_POINTS];
    float y_memory[MAX_PATH_POINTS];
    float x_memory_store[MAX_PATH_POINTS];  // 存储的路径点（东天北坐标系）
    float y_memory_store[MAX_PATH_POINTS];
    
    // 控制标志
    uint8_t road_memory_finish_flag;    // 路径记忆完成标志位
    uint8_t road_memory_start_flag;     // 路径记忆开始标志位
    uint8_t road_recurrent_flag;        // 路径复现标志位
    
    // 计数器
    uint16_t num_l;                     // 左轮计数器
    uint16_t num_r;                     // 右轮计数器
    uint16_t road_destination;          // 路径终点
    
} path_memory_t;

// 全局变量声明
extern ins_system_t ins_system;
extern path_memory_t path_memory;

// 函数声明

/**
 * @brief 惯导系统初始化
 */
void ins_init(void);

/**
 * @brief 更新惯导系统状态
 * @param dt 时间间隔（秒）
 */
void ins_update(float dt);

/**
 * @brief 路径记忆函数
 */
void distance_get_plus(void);

/**
 * @brief 纯跟踪控制算法
 * @return 差速控制量
 */
int16_t pure_pursuit_control(void);

/**
 * @brief 查找预瞄点索引
 * @param L 预瞄距离（厘米）
 */
void find_lookahead_index(float L);

/**
 * @brief 计算横向误差
 * @param lookahead_index 预瞄点索引
 * @return 横向误差（厘米）
 */
float calculate_e_lat(int lookahead_index);

/**
 * @brief 打滑检测
 */
void slip_check(void);

/**
 * @brief 更新轮子脉冲累计值
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return 前进厘米数
 */
int32_t update_wheel_pulses(WheelData *wheel, int16_t current_encoder);

/**
 * @brief 获取轮子速度
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return 速度（脉冲/采样周期）
 */
int32_t get_wheel_speed(WheelData *wheel, int16_t current_encoder);

/**
 * @brief 获取当前实时位置
 * @param x_pos 输出X坐标指针（单位：厘米）
 * @param y_pos 输出Y坐标指针（单位：厘米）
 */
void ins_get_position(float *x_pos, float *y_pos);

/**
 * @brief 获取当前偏航角
 * @return 偏航角（弧度）
 */
float ins_get_yaw(void);

/**
 * @brief 获取当前速度信息
 * @param left_speed 左轮速度指针（厘米/秒）
 * @param right_speed 右轮速度指针（厘米/秒）
 * @param avg_speed 平均速度指针（厘米/秒）
 */
void ins_get_velocity(float *left_speed, float *right_speed, float *avg_speed);

/**
 * @brief 重置位置到原点
 */
void ins_reset_position(void);

/**
 * @brief 设置当前位置
 * @param x_pos X坐标（厘米）
 * @param y_pos Y坐标（厘米）
 */
void ins_set_position(float x_pos, float y_pos);

#endif  // _ALGO_INS_H_