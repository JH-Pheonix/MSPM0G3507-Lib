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
        if(uart_query_byte(uart_index, &emm42_receive_buffer[emm42_receive_length]))
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
    uint32 timeout_count = 0;
    uint8 status = 0;
    uint8 result;
    
    while(timeout_count < timeout_ms)
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
        timeout_count += 10;
    }
    
    return EMM42_ERROR_TIMEOUT;
}

//====================================================回零相关函数实现====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置单圈回零的零点位置
// 参数说明     uart_index          UART通道
// 参数说明     store_flag          是否存储标志 (1-存储 0-不存储)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_set_origin_zero(UART_1, 1);
// 备注信息     需要先将电机转到想要的零点位置，然后调用此函数设置
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_set_origin_zero(uart_index_enum uart_index, uint8 store_flag)
{
    uint8 command[8];
    uint8 data = store_flag ? 1 : 0;
    
    emm42_build_frame(command, g_default_address, 0x93, NULL, 0);
    command[2] = 0x88;
    command[3] = data;
    
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
    
    // 检查命令执行状态
    if(emm42_receive_buffer[2] != EMM42_CMD_SUCCESS)
    {
        return EMM42_ERROR_CONDITION;
    }
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     触发回零
// 参数说明     uart_index          UART通道
// 参数说明     mode                回零模式
// 参数说明     sync_flag           多机同步标志 (1-启用 0-不启用)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_trigger_origin(UART_1, EMM42_ORIGIN_NEAREST, 0);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_trigger_origin(uart_index_enum uart_index, emm42_origin_mode_enum mode, uint8 sync_flag)
{
    uint8 command[8];
    uint8 data[2];
    
    data[0] = (uint8)mode;
    data[1] = sync_flag ? 1 : 0;
    
    emm42_build_frame(command, g_default_address, 0x9A, data, 2);
    
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
    
    // 检查命令执行状态
    if(emm42_receive_buffer[2] != EMM42_CMD_SUCCESS)
    {
        return EMM42_ERROR_CONDITION;
    }
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     强制中断并退出回零操作
// 参数说明     uart_index          UART通道
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_interrupt_origin(UART_1);
// 备注信息     在回零过程中调用可强制停止回零
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_interrupt_origin(uart_index_enum uart_index)
{
    uint8 command[8];
    
    emm42_build_frame(command, g_default_address, 0x9C, NULL, 0);
    command[2] = 0x48;
    
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
    
    // 检查命令执行状态
    if(emm42_receive_buffer[2] != EMM42_CMD_SUCCESS)
    {
        return EMM42_ERROR_CONDITION;
    }
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取原点回零参数
// 参数说明     uart_index          UART通道
// 参数说明     *params             参数结构体指针
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_read_origin_params(UART_1, &origin_params);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_read_origin_params(uart_index_enum uart_index, emm42_origin_params_struct *params)
{
    if(!params) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    
    emm42_build_frame(command, g_default_address, 0x22, NULL, 0);
    
    if(emm42_send_command(uart_index, command, 8) != EMM42_ERROR_NONE)
    {
        return EMM42_ERROR_COMMUNICATION;
    }
    
    if(emm42_receive_response(uart_index) < 20) // 至少需要20字节响应
    {
        return EMM42_ERROR_COMMUNICATION;
    }
    
    if(!emm42_verify_checksum(emm42_receive_buffer, emm42_receive_length))
    {
        return EMM42_ERROR_CHECKSUM;
    }
    
    // 解析返回的参数数据
    params->mode = (emm42_origin_mode_enum)emm42_receive_buffer[2];
    params->direction = (emm42_direction_enum)emm42_receive_buffer[3];
    params->speed_rpm = (emm42_receive_buffer[4] << 8) | emm42_receive_buffer[5];
    params->timeout_ms = (emm42_receive_buffer[6] << 24) | (emm42_receive_buffer[7] << 16) | 
                        (emm42_receive_buffer[8] << 8) | emm42_receive_buffer[9];
    params->collision_speed_rpm = (emm42_receive_buffer[10] << 8) | emm42_receive_buffer[11];
    params->collision_current_ma = (emm42_receive_buffer[12] << 8) | emm42_receive_buffer[13];
    params->collision_time_ms = (emm42_receive_buffer[14] << 8) | emm42_receive_buffer[15];
    params->auto_trigger = emm42_receive_buffer[16];
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     修改原点回零参数
// 参数说明     uart_index          UART通道
// 参数说明     *params             参数结构体指针
// 参数说明     store_flag          是否存储标志 (1-存储 0-不存储)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_write_origin_params(UART_1, &origin_params, 1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_write_origin_params(uart_index_enum uart_index, const emm42_origin_params_struct *params, uint8 store_flag)
{
    if(!params) return EMM42_ERROR_PARAM;
    
    uint8 command[24]; // 需要更长的命令缓冲区
    uint8 data[15];
    
    // 构建参数数据
    data[0] = store_flag ? 1 : 0;
    data[1] = (uint8)params->mode;
    data[2] = (uint8)params->direction;
    data[3] = (params->speed_rpm >> 8) & 0xFF;
    data[4] = params->speed_rpm & 0xFF;
    data[5] = (params->timeout_ms >> 24) & 0xFF;
    data[6] = (params->timeout_ms >> 16) & 0xFF;
    data[7] = (params->timeout_ms >> 8) & 0xFF;
    data[8] = params->timeout_ms & 0xFF;
    data[9] = (params->collision_speed_rpm >> 8) & 0xFF;
    data[10] = params->collision_speed_rpm & 0xFF;
    data[11] = (params->collision_current_ma >> 8) & 0xFF;
    data[12] = params->collision_current_ma & 0xFF;
    data[13] = (params->collision_time_ms >> 8) & 0xFF;
    data[14] = params->collision_time_ms & 0xFF;
    
    // 构建基础帧头
    command[0] = g_default_address;
    command[1] = 0x4C;
    command[2] = 0xAE;
    
    // 复制参数数据
    for(uint8 i = 0; i < 15; i++)
    {
        command[3 + i] = data[i];
    }
    
    // 添加auto_trigger参数
    command[18] = params->auto_trigger ? 1 : 0;
    
    // 计算校验和
    uint8 checksum = 0;
    switch(g_checksum_mode)
    {
        case EMM42_CHECKSUM_0X6B:
            for(uint8 i = 0; i < 19; i++)
            {
                checksum += command[i];
            }
            command[19] = checksum;
            command[20] = 0x6B;
            break;
            
        case EMM42_CHECKSUM_XOR:
            for(uint8 i = 0; i < 19; i++)
            {
                checksum ^= command[i];
            }
            command[19] = checksum;
            command[20] = 0x6B;
            break;
            
        default:
            // 默认使用固定0x6B校验
            for(uint8 i = 0; i < 19; i++)
            {
                checksum += command[i];
            }
            command[19] = checksum;
            command[20] = 0x6B;
            break;
    }
    
    if(emm42_send_command(uart_index, command, 21) != EMM42_ERROR_NONE)
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
    
    // 检查命令执行状态
    if(emm42_receive_buffer[2] != EMM42_CMD_SUCCESS)
    {
        return EMM42_ERROR_CONDITION;
    }
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取回零状态标志位
// 参数说明     uart_index          UART通道
// 参数说明     *status             状态标志指针
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_read_origin_status(UART_1, &status);
// 备注信息     状态标志按位表示: bit0-编码器就绪 bit1-校准表就绪 bit2-正在回零 bit3-回零失败
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_read_origin_status(uart_index_enum uart_index, uint8 *status)
{
    if(!status) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    
    emm42_build_frame(command, g_default_address, 0x3B, NULL, 0);
    
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
    
    *status = emm42_receive_buffer[2];
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待回零完成
// 参数说明     uart_index          UART通道
// 参数说明     timeout_ms          超时时间(毫秒)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_wait_for_origin_completion(UART_1, 30000);
// 备注信息     通过读取回零状态标志位判断回零是否完成或失败
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_wait_for_origin_completion(uart_index_enum uart_index, uint32 timeout_ms)
{
    uint32 timeout_count = 0;
    uint8 status = 0;
    uint8 result;
    
    while(timeout_count < timeout_ms)
    {
        // 读取回零状态标志位
        result = emm42_read_origin_status(uart_index, &status);
        if(result == EMM42_ERROR_NONE)
        {
            // 检查回零失败标志位 (bit3)
            if(status & EMM42_ORIGIN_STATUS_FAIL)
            {
                return EMM42_ERROR_CONDITION; // 回零失败
            }
            
            // 检查正在回零标志位 (bit2)
            if(!(status & EMM42_ORIGIN_STATUS_HOMING))
            {
                // 没有在回零，检查编码器和校准表是否就绪
                if((status & EMM42_ORIGIN_STATUS_ENCODER_READY) && 
                   (status & EMM42_ORIGIN_STATUS_CAL_READY))
                {
                    return EMM42_ERROR_NONE; // 回零成功
                }
            }
        }
        
        system_delay_ms(100); // 100ms检查间隔
        timeout_count += 100;
    }
    
    return EMM42_ERROR_TIMEOUT;
}
