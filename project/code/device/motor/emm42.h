#ifndef _DEVICE_MOTOR_EMM42_H_
#define _DEVICE_MOTOR_EMM42_H_

#include "zf_common_headfile.h"

//====================================================用户配置区域====================================================
// EMM42通讯参数配置
#define EMM42_DEFAULT_BAUDRATE          115200          // 默认波特率
#define EMM42_DEFAULT_ADDRESS           0x01            // 默认设备地址（独立UART时都可以用相同地址）
#define EMM42_BROADCAST_ADDRESS         0               // 广播地址
#define EMM42_TIMEOUT_MS                100             // 通讯超时时间(ms)
#define EMM42_RECEIVE_BUFFER_SIZE       64              // 接收缓冲区大小
#define EMM42_MAX_RETRY_COUNT           3               // 最大重试次数

// EMM42电机参数配置
#define EMM42_MOTOR_STEP_ANGLE          1.8             // 步进角度(度)
#define EMM42_DEFAULT_SUBDIVISION       16              // 默认细分数
#define EMM42_PULSES_PER_REVOLUTION     (360.0 / EMM42_MOTOR_STEP_ANGLE * EMM42_DEFAULT_SUBDIVISION) // 每圈脉冲数

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

//====================================================错误码定义====================================================
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

//====================================================结构体定义====================================================
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

// 回零状态标志位定义
typedef enum
{
    EMM42_ORIGIN_STATUS_ENCODER_READY   = 0x01,         // 编码器就绪状态标志位
    EMM42_ORIGIN_STATUS_CAL_READY       = 0x02,         // 校准表就绪状态标志位
    EMM42_ORIGIN_STATUS_HOMING          = 0x04,         // 正在回零标志位
    EMM42_ORIGIN_STATUS_FAIL            = 0x08,         // 回零失败标志位
} emm42_origin_status_enum;

// 回零参数结构体
typedef struct
{
    emm42_origin_mode_enum mode;                        // 回零模式
    emm42_direction_enum direction;                     // 回零方向
    uint16 speed_rpm;                                   // 回零转速(RPM)
    uint32 timeout_ms;                                  // 回零超时时间(ms)
    uint16 collision_speed_rpm;                         // 无限位碰撞回零检测转速(RPM)
    uint16 collision_current_ma;                        // 无限位碰撞回零检测电流(mA)
    uint16 collision_time_ms;                           // 无限位碰撞回零检测时间(ms)
    uint8 auto_trigger;                                 // 是否使能上电自动触发回零功能
} emm42_origin_params_struct;

//====================================================全局配置与缓冲区====================================================
// 全局接收缓冲区 - 由于通讯是串行的，可以共用一个缓冲区
extern uint8 emm42_receive_buffer[EMM42_RECEIVE_BUFFER_SIZE];
extern uint16 emm42_receive_length;

// 全局配置函数声明
void emm42_set_global_config(uint8 default_address, uint8 checksum_mode, uint32 baudrate, uint32 timeout_ms);
uint8 emm42_get_default_address(void);
uint8 emm42_get_checksum_mode(void);
uint32 emm42_get_baudrate(void);
uint32 emm42_get_timeout_ms(void);

//====================================================函数声明====================================================
uint8 emm42_init(uart_index_enum uart_index, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);

uint8 emm42_enable_motor(uart_index_enum uart_index, uint8 enable);
uint8 emm42_speed_control(uart_index_enum uart_index, int16 speed);
uint8 emm42_position_control(uart_index_enum uart_index, int32 position, uint8 absolute);
uint8 emm42_rotate_angle(uart_index_enum uart_index, float angle);
uint8 emm42_emergency_stop(uart_index_enum uart_index);

uint8 emm42_read_param(uart_index_enum uart_index, uint8 param_type, uint8 *value);

uint8 emm42_wait_for_completion(uart_index_enum uart_index, uint32 timeout_ms);

// 回零相关函数
uint8 emm42_set_origin_zero(uart_index_enum uart_index, uint8 store_flag);
uint8 emm42_trigger_origin(uart_index_enum uart_index, emm42_origin_mode_enum mode, uint8 sync_flag);
uint8 emm42_interrupt_origin(uart_index_enum uart_index);
uint8 emm42_read_origin_params(uart_index_enum uart_index, emm42_origin_params_struct *params);
uint8 emm42_write_origin_params(uart_index_enum uart_index, const emm42_origin_params_struct *params, uint8 store_flag);
uint8 emm42_read_origin_status(uart_index_enum uart_index, uint8 *status);
uint8 emm42_wait_for_origin_completion(uart_index_enum uart_index, uint32 timeout_ms);

#endif
