/*
 * @file       : emm42.c
 * @brief      : EMM42步进闭环驱动器驱动库
 * @author     : ZeroHzzzz
 * @version    : 2.0
 * @date       : 2024-12-19
 */

#include "emm42.h"

//====================================================全局变量====================================================
// 全局接收缓冲区
uint8 emm42_receive_buffer[EMM42_RECEIVE_BUFFER_SIZE];
uint16 emm42_receive_length = 0;

// 全局配置变量
static uint8 g_default_address = EMM42_DEFAULT_ADDRESS;
static uint8 g_checksum_mode = EMM42_CHECKSUM_0X6B;
static uint32 g_baudrate = EMM42_DEFAULT_BAUDRATE;
static uint32 g_timeout_ms = EMM42_TIMEOUT_MS;

//====================================================全局配置函数实现====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置全局配置
// 参数说明     default_address     默认设备地址
// 参数说明     checksum_mode       校验模式
// 参数说明     baudrate            波特率
// 参数说明     timeout_ms          超时时间(ms)
// 返回参数     void
// 使用示例     emm42_set_global_config(0x01, EMM42_CHECKSUM_XOR, 115200, 100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void emm42_set_global_config(uint8 default_address, uint8 checksum_mode, uint32 baudrate, uint32 timeout_ms)
{
    g_default_address = default_address;
    g_checksum_mode = checksum_mode;
    g_baudrate = baudrate;
    g_timeout_ms = timeout_ms;
}

uint8 emm42_get_default_address(void) { return g_default_address; }
uint8 emm42_get_checksum_mode(void) { return g_checksum_mode; }
uint32 emm42_get_baudrate(void) { return g_baudrate; }
uint32 emm42_get_timeout_ms(void) { return g_timeout_ms; }

//====================================================底层通讯函数实现====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     构建命令帧
// 参数说明     *frame              帧数据指针
// 参数说明     address             设备地址
// 参数说明     command             命令
// 参数说明     *data               数据指针
// 参数说明     data_len            数据长度
// 返回参数     void
// 使用示例     emm42_build_frame(frame, 1, 0xF3, &data, 1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static void emm42_build_frame(uint8 *frame, uint8 address, uint8 command, const uint8 *data, uint8 data_len)
{
    frame[0] = address;      // 地址
    frame[1] = command;      // 命令
    
    // 数据位
    if(data && data_len > 0)
    {
        for(uint8 i = 0; i < data_len && i < 4; i++)
        {
            frame[2 + i] = data[i];
        }
        for(uint8 i = data_len; i < 4; i++)
        {
            frame[2 + i] = 0; // 不足部分补0
        }
    }
    else
    {
        for(uint8 i = 0; i < 4; i++)
        {
            frame[2 + i] = 0;
        }
    }
    
    // 计算校验和 - 根据全局配置选择校验方式
    uint8 checksum = 0;
    switch(g_checksum_mode)
    {
        case EMM42_CHECKSUM_0X6B:
            for(uint8 i = 0; i < 6; i++)
            {
                checksum += frame[i];
            }
            frame[6] = checksum;
            frame[7] = 0x6B;
            break;
            
        case EMM42_CHECKSUM_XOR:
            for(uint8 i = 0; i < 6; i++)
            {
                checksum ^= frame[i];
            }
            frame[6] = checksum;
            frame[7] = 0x6B;
            break;
            
        default:
            // 默认使用固定0x6B校验
            for(uint8 i = 0; i < 6; i++)
            {
                checksum += frame[i];
            }
            frame[6] = checksum;
            frame[7] = 0x6B;
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送命令
// 参数说明     uart_index          UART通道
// 参数说明     *data               数据指针
// 参数说明     length              数据长度
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_send_command(UART_1, command, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 emm42_send_command(uart_index_enum uart_index, const uint8 *data, uint8 length)
{
    if(!data) return EMM42_ERROR_PARAM;
    
    // 发送数据
    for(uint8 i = 0; i < length; i++)
    {
        uart_write_byte(uart_index, data[i]);
    }
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     校验响应帧
// 参数说明     *data               数据指针
// 参数说明     length              数据长度
// 返回参数     uint8               1-校验成功 0-校验失败
// 使用示例     result = emm42_verify_checksum(response, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 emm42_verify_checksum(const uint8 *data, uint8 length)
{
    if(!data || length < 8) return 0;
    
    uint8 checksum = 0;
    switch(g_checksum_mode)
    {
        case EMM42_CHECKSUM_0X6B:
            for(uint8 i = 0; i < 6; i++)
            {
                checksum += data[i];
            }
            return (checksum == data[6] && data[7] == 0x6B);
            
        case EMM42_CHECKSUM_XOR:
            for(uint8 i = 0; i < 6; i++)
            {
                checksum ^= data[i];
            }
            return (checksum == data[6] && data[7] == 0x6B);
            
        default:
            // 默认使用固定0x6B校验
            for(uint8 i = 0; i < 6; i++)
            {
                checksum += data[i];
            }
            return (checksum == data[6] && data[7] == 0x6B);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收响应数据
// 参数说明     uart_index          UART通道
// 返回参数     uint8               实际接收长度
// 使用示例     length = emm42_receive_response(UART_1);
// 备注信息     接收到的数据存储在全局缓冲区emm42_receive_buffer中
//-------------------------------------------------------------------------------------------------------------------
static uint8 emm42_receive_response(uart_index_enum uart_index)
{
    emm42_receive_length = 0;
    uint32 timeout_count = 0;
    const uint32 timeout_limit = g_timeout_ms * 1000; // 转换为us
    
    while(emm42_receive_length < EMM42_RECEIVE_BUFFER_SIZE && timeout_count < timeout_limit)
    {
        if(uart_query_8bit(uart_index, &emm42_receive_buffer[emm42_receive_length]))
        {
            emm42_receive_length++;
            timeout_count = 0; // 重置超时计数
            
            // 如果接收到完整的8字节响应，退出
            if(emm42_receive_length >= 8)
            {
                break;
            }
        }
        else
        {
            timeout_count++;
            system_delay_us(1); // 1us延时
        }
    }
    
    return emm42_receive_length;
}

//====================================================单设备控制函数实现====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化EMM42电机UART通道
// 参数说明     uart_index          UART通道
// 参数说明     tx_pin              UART发送引脚
// 参数说明     rx_pin              UART接收引脚
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_init(UART_1, UART1_TX_A8, UART1_RX_A9);
// 备注信息     使用全局配置的波特率进行初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_init(uart_index_enum uart_index, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // 初始化UART
    uart_init(uart_index, g_baudrate, tx_pin, rx_pin);
    
    // 清空接收缓冲区
    uart_clear_index(uart_index);
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使能/失能电机
// 参数说明     uart_index          UART通道
// 参数说明     enable              1-使能 0-失能
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_enable_motor(UART_1, 1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_enable_motor(uart_index_enum uart_index, uint8 enable)
{
    uint8 command[8];
    uint8 data = enable ? 1 : 0;
    
    emm42_build_frame(command, g_default_address, 0xF3, &data, 1);
    
    return emm42_send_command(uart_index, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度控制
// 参数说明     uart_index          UART通道
// 参数说明     speed               速度 (rpm, 带符号)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_speed_control(UART_1, 300);
// 备注信息     正值顺时针，负值逆时针
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_speed_control(uart_index_enum uart_index, int16 speed)
{
    uint8 command[8];
    uint8 data[2];
    
    data[0] = (uint8)(speed & 0xFF);        // 低字节
    data[1] = (uint8)((speed >> 8) & 0xFF); // 高字节
    
    emm42_build_frame(command, g_default_address, 0xF6, data, 2);
    
    return emm42_send_command(uart_index, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置控制
// 参数说明     uart_index          UART通道
// 参数说明     position            目标位置 (脉冲数)
// 参数说明     absolute            1-绝对位置 0-相对位置
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_position_control(UART_1, 1600, 1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_position_control(uart_index_enum uart_index, int32 position, uint8 absolute)
{
    uint8 command[8];
    uint8 data[4];
    uint8 cmd = absolute ? 0xFD : 0xF4;
    
    data[0] = (uint8)(position & 0xFF);         // 位置低字节
    data[1] = (uint8)((position >> 8) & 0xFF);  // 位置次低字节
    data[2] = (uint8)((position >> 16) & 0xFF); // 位置次高字节
    data[3] = (uint8)((position >> 24) & 0xFF); // 位置高字节
    
    emm42_build_frame(command, g_default_address, cmd, data, 4);
    
    return emm42_send_command(uart_index, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     旋转指定角度
// 参数说明     uart_index          UART通道
// 参数说明     angle               角度 (度, 带符号)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_rotate_angle(UART_1, 90);
// 备注信息     正值顺时针，负值逆时针
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_rotate_angle(uart_index_enum uart_index, float angle)
{
    // 将角度转换为脉冲数
    int32 pulses = (int32)(angle * EMM42_PULSES_PER_REVOLUTION / 360.0);
    
    return emm42_position_control(uart_index, pulses, 0); // 相对位置模式
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停止
// 参数说明     uart_index          UART通道
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_emergency_stop(UART_1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_emergency_stop(uart_index_enum uart_index)
{
    uint8 command[8];
    
    emm42_build_frame(command, g_default_address, 0xFE, NULL, 0);
    
    return emm42_send_command(uart_index, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取设备参数
// 参数说明     uart_index          UART通道
// 参数说明     param_type          参数类型
// 参数说明     *value              参数值指针
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_read_param(UART_1, 0x30, &status);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_read_param(uart_index_enum uart_index, uint8 param_type, uint8 *value)
{
    if(!value) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    
    emm42_build_frame(command, g_default_address, param_type, NULL, 0);
    
    if(emm42_send_command(uart_index, command, 8) != EMM42_ERROR_NONE)
    {
        return EMM42_ERROR_COMMUNICATION;
    }
    
    if(emm42_receive_response(uart_index) < 8)
    {
        return EMM42_ERROR_COMMUNICATION;
    }
    
    if(!emm42_verify_checksum(emm42_receive_buffer, 8))
    {
        return EMM42_ERROR_CHECKSUM;
    }
    
    *value = emm42_receive_buffer[3]; // 数据位
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待运动完成
// 参数说明     uart_index          UART通道
// 参数说明     timeout_ms          超时时间(毫秒)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_wait_for_completion(UART_1, 5000);
// 备注信息     通过读取电机状态寄存器判断运动是否完成
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_wait_for_completion(uart_index_enum uart_index, uint32 timeout_ms)
{
    uint32 start_time = system_get_time_ms();
    uint8 status = 0;
    uint8 result;
    
    while((system_get_time_ms() - start_time) < timeout_ms)
    {
        // 读取电机状态寄存器 (0x30是状态寄存器地址)
        result = emm42_read_param(uart_index, 0x30, &status);
        if(result == EMM42_ERROR_NONE)
        {
            // 检查运动状态位 (bit0表示运动状态: 0-静止, 1-运动中)
            if((status & 0x01) == 0)
            {
                return EMM42_ERROR_NONE;
            }
        }
        
        system_delay_ms(10); // 10ms检查间隔
    }
    
    return EMM42_ERROR_TIMEOUT;
}
