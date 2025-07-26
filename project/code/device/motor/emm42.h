#ifndef _DEVICE_MOTOR_EMM42_H_
#define _DEVICE_MOTOR_EMM42_H_

#include "zf_common_headfile.h"

//====================================================用户配置区域====================================================
// EMM42通讯参数配置
#define EMM42_DEFAULT_BAUDRATE          115200          // 默认波特率
#define EMM42_BROADCAST_ADDRESS         0               // 广播地址
#define EMM42_TIMEOUT_MS                100             // 通讯超时时间(ms)
#define EMM42_RECEIVE_BUFFER_SIZE       64              // 接收缓冲区大小
#define EMM42_MAX_RETRY_COUNT           3               // 最大重试次数

// EMM42电机参数配置
#define EMM42_MOTOR_STEP_ANGLE          1.8             // 步进角度(度)
#define EMM42_DEFAULT_SUBDIVISION       16              // 默认细分数
#define EMM42_PULSES_PER_REVOLUTION     (360.0 / EMM42_MOTOR_STEP_ANGLE * EMM42_DEFAULT_SUBDIVISION) // 每圈脉冲数

// EMM42功能使能配置
#define EMM42_ENABLE_DEBUG_OUTPUT       1               // 使能调试输出 1-使能 0-关闭
#define EMM42_ENABLE_AUTO_RETRY         1               // 使能自动重试 1-使能 0-关闭

// 校验方式
#define EMM42_CHECKSUM_0X6B             0x00            // 固定0x6B校验
#define EMM42_CHECKSUM_XOR              0x01            // XOR校验
#define EMM42_CHECKSUM_CRC8             0x02            // CRC-8校验

// 命令状态码
#define EMM42_CMD_SUCCESS               0x02            // 命令执行成功
#define EMM42_CMD_CONDITION_ERROR       0xE2            // 条件不满足
#define EMM42_CMD_ERROR                 0xEE            // 错误命令

//====================================================错误码定义====================================================
#define EMM42_ERROR_NONE                0x00            // 无错误
#define EMM42_ERROR_PARAM               0x01            // 参数错误
#define EMM42_ERROR_TIMEOUT             0x02            // 超时错误
#define EMM42_ERROR_CHECKSUM            0x03            // 校验错误
#define EMM42_ERROR_RESPONSE            0x04            // 响应错误
#define EMM42_ERROR_CONDITION           0x05            // 条件不满足
#define EMM42_ERROR_COMMUNICATION       0x06            // 通讯错误

//====================================================调试宏定义====================================================
#if EMM42_ENABLE_DEBUG_OUTPUT
    #define EMM42_DEBUG_PRINTF(format, ...)    printf("[EMM42 DEBUG] " format, ##__VA_ARGS__)
    #define EMM42_ERROR_PRINTF(format, ...)    printf("[EMM42 ERROR] " format, ##__VA_ARGS__)
    #define EMM42_INFO_PRINTF(format, ...)     printf("[EMM42 INFO] " format, ##__VA_ARGS__)
#else
    #define EMM42_DEBUG_PRINTF(format, ...)
    #define EMM42_ERROR_PRINTF(format, ...)
    #define EMM42_INFO_PRINTF(format, ...)
#endif

//====================================================常用宏定义====================================================
// 角度转脉冲数宏
#define EMM42_ANGLE_TO_PULSES(angle)    ((uint32)((angle) * EMM42_PULSES_PER_REVOLUTION / 360.0))

// 转数转脉冲数宏
#define EMM42_REVOLUTIONS_TO_PULSES(rev) ((uint32)((rev) * EMM42_PULSES_PER_REVOLUTION))

// 脉冲数转角度宏
#define EMM42_PULSES_TO_ANGLE(pulses)   ((float)(pulses) * 360.0 / EMM42_PULSES_PER_REVOLUTION)

// 脉冲数转转数宏
#define EMM42_PULSES_TO_REVOLUTIONS(pulses) ((float)(pulses) / EMM42_PULSES_PER_REVOLUTION)

// 数组长度宏
#define EMM42_ARRAY_SIZE(arr)           (sizeof(arr) / sizeof((arr)[0]))

// 最小值和最大值宏
#define EMM42_MIN(a, b)                 ((a) < (b) ? (a) : (b))
#define EMM42_MAX(a, b)                 ((a) > (b) ? (a) : (b))

// 限制值在范围内的宏
#define EMM42_CLAMP(value, min, max)    EMM42_MAX(min, EMM42_MIN(value, max))

//====================================================预定义常量====================================================
// 常用转速定义 (RPM)
#define EMM42_SPEED_VERY_SLOW           50
#define EMM42_SPEED_SLOW                200
#define EMM42_SPEED_MEDIUM              500
#define EMM42_SPEED_FAST                1000
#define EMM42_SPEED_VERY_FAST           2000
#define EMM42_SPEED_MAX                 3000

// 常用加速度定义
#define EMM42_ACCELERATION_VERY_SLOW    1
#define EMM42_ACCELERATION_SLOW         5
#define EMM42_ACCELERATION_MEDIUM       10
#define EMM42_ACCELERATION_FAST         50
#define EMM42_ACCELERATION_VERY_FAST    100
#define EMM42_ACCELERATION_MAX          255

// 常用角度定义
#define EMM42_ANGLE_90_DEG              90.0f
#define EMM42_ANGLE_180_DEG             180.0f
#define EMM42_ANGLE_270_DEG             270.0f
#define EMM42_ANGLE_360_DEG             360.0f

//====================================================函数式宏定义====================================================
// 检查设备指针有效性
#define EMM42_CHECK_DEVICE(device) \
    do { \
        if((device) == NULL) { \
            EMM42_ERROR_PRINTF("Device pointer is NULL!\r\n"); \
            return EMM42_ERROR_PARAM; \
        } \
    } while(0)

// 检查参数有效性
#define EMM42_CHECK_PARAM(param) \
    do { \
        if((param) == NULL) { \
            EMM42_ERROR_PRINTF("Parameter is NULL!\r\n"); \
            return EMM42_ERROR_PARAM; \
        } \
    } while(0)

// 检查返回值
#define EMM42_CHECK_RESULT(result, error_msg) \
    do { \
        if((result) != EMM42_ERROR_NONE) { \
            EMM42_ERROR_PRINTF("%s (Error: %d)\r\n", error_msg, result); \
            return result; \
        } \
    } while(0)

//====================================================结构体定义====================================================
// EMM42设备结构体
typedef struct
{
    uint8 address;                                      // 设备地址
    uint8 checksum_mode;                                // 校验模式
    uart_index_enum uart_index;                         // 使用的UART通道
    uint32 baudrate;                                    // 波特率
    uint32 timeout_ms;                                  // 超时时间
    uint8 receive_buffer[EMM42_RECEIVE_BUFFER_SIZE];    // 接收缓冲区
    uint16 receive_length;                              // 接收数据长度
} emm42_device_struct;

// 电机状态结构体
typedef struct
{
    uint8 enabled;                                      // 使能状态
    uint8 in_position;                                  // 到位状态
    uint8 stalled;                                      // 堵转状态
    uint8 stall_protection;                             // 堵转保护状态
} emm42_motor_status_struct;

// 电机实时信息结构体
typedef struct
{
    int16 speed_rpm;                                    // 实时转速(RPM)
    uint16 current_ma;                                  // 相电流(mA)
    uint16 voltage_mv;                                  // 总线电压(mV)
    int32 position;                                     // 实时位置
    int32 target_position;                              // 目标位置
    int32 position_error;                               // 位置误差
} emm42_motor_info_struct;

//====================================================枚举定义====================================================
// 电机旋转方向
typedef enum
{
    EMM42_DIRECTION_CW  = 0x00,                         // 顺时针
    EMM42_DIRECTION_CCW = 0x01,                         // 逆时针
} emm42_direction_enum;

// 位置模式类型
typedef enum
{
    EMM42_POSITION_RELATIVE = 0x00,                     // 相对位置模式
    EMM42_POSITION_ABSOLUTE = 0x01,                     // 绝对位置模式
} emm42_position_mode_enum;

// 回零模式
typedef enum
{
    EMM42_ORIGIN_NEAREST    = 0x00,                     // 单圈就近回零
    EMM42_ORIGIN_DIRECTION  = 0x01,                     // 单圈方向回零
    EMM42_ORIGIN_COLLISION  = 0x02,                     // 多圈无限位碰撞回零
    EMM42_ORIGIN_ENDSTOP    = 0x03,                     // 多圈有限位开关回零
} emm42_origin_mode_enum;

//====================================================函数声明====================================================
uint8 emm42_init(emm42_device_struct *device, uint8 address, uart_index_enum uart_index, uint32 baudrate);

uint8 emm42_enable_motor(emm42_device_struct *device, uint8 enable);
uint8 emm42_speed_control(emm42_device_struct *device, int16 speed);
uint8 emm42_position_control(emm42_device_struct *device, int32 position, uint8 absolute);
uint8 emm42_rotate_angle(emm42_device_struct *device, float angle);
uint8 emm42_emergency_stop(emm42_device_struct *device);

uint8 emm42_read_param(emm42_device_struct *device, uint8 param_type, uint8 *value);

uint8 emm42_wait_for_completion(emm42_device_struct *device, uint32 timeout_ms);

#endif
