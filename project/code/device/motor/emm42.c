/*
 * @file       : emm42.c
 * @brief      : EMM42步进闭环驱动器驱动库
 * @author     : ZeroHzzzz
 * @version    : 1.0
 * @date       : 2024-12-19
 */

#include "emm42.h"

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
    
    // 计算校验和
    uint8 checksum = 0;
    for(uint8 i = 0; i < 6; i++)
    {
        checksum += frame[i];
    }
    frame[6] = checksum;
    frame[7] = 0x6B; // 帧尾
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送命令
// 参数说明     *device             设备结构体指针
// 参数说明     *data               数据指针
// 参数说明     length              数据长度
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_send_command(&emm42_device, command, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 emm42_send_command(emm42_device_struct *device, const uint8 *data, uint8 length)
{
    if(!device || !data) return EMM42_ERROR_PARAM;
    
    // 发送数据
    for(uint8 i = 0; i < length; i++)
    {
        uart_write_byte(device->uart_index, data[i]);
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
    for(uint8 i = 0; i < 6; i++)
    {
        checksum += data[i];
    }
    
    return (checksum == data[6] && data[7] == 0x6B);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收响应数据
// 参数说明     *device             设备结构体指针
// 参数说明     *data               数据缓冲区指针
// 参数说明     max_length          最大长度
// 返回参数     uint8               实际接收长度
// 使用示例     length = emm42_receive_response(&emm42_device, buffer, 8);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 emm42_receive_response(emm42_device_struct *device, uint8 *data, uint8 max_length)
{
    if(!device || !data) return 0;
    
    uint8 length = 0;
    uint32 timeout_count = 0;
    const uint32 timeout_limit = device->timeout_ms * 1000; // 转换为us
    
    while(length < max_length && timeout_count < timeout_limit)
    {
        if(uart_query_8bit(device->uart_index, &data[length]))
        {
            length++;
            timeout_count = 0; // 重置超时计数
        }
        else
        {
            timeout_count++;
            system_delay_us(1); // 1us延时
        }
    }
    
    return length;
}

//====================================================单设备控制函数实现====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化EMM42设备
// 参数说明     *device             设备结构体指针
// 参数说明     address             设备地址 (1-247)
// 参数说明     uart_index          UART通道
// 参数说明     baudrate            波特率
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_init(&emm42_device, 1, UART_1, 115200);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_init(emm42_device_struct *device, uint8 address, uart_index_enum uart_index, uint32 baudrate)
{
    if(!device || address == 0 || address > 247)
    {
        return EMM42_ERROR_PARAM;
    }
    
    // 初始化设备参数
    device->address = address;
    device->uart_index = uart_index;
    device->baudrate = baudrate;
    device->checksum_mode = EMM42_CHECKSUM_0X6B;
    device->timeout_ms = EMM42_TIMEOUT_MS;
    device->receive_length = 0;
    
    // 初始化UART
    uart_init(uart_index, baudrate, UART_1_TX, UART_1_RX);
    
    // 清空接收缓冲区
    uart_clear_index(uart_index);
    
    EMM42_DEBUG_PRINTF("EMM42设备初始化完成: 地址=%d, UART=%d, 波特率=%d\r\n", 
                       address, uart_index, baudrate);
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使能/失能电机
// 参数说明     *device             设备结构体指针
// 参数说明     enable              1-使能 0-失能
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_enable_motor(&emm42_device, 1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_enable_motor(emm42_device_struct *device, uint8 enable)
{
    if(!device) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    uint8 data = enable ? 1 : 0;
    
    emm42_build_frame(command, device->address, 0xF3, &data, 1);
    
    EMM42_DEBUG_PRINTF("电机%s: 地址=%d\r\n", enable ? "使能" : "失能", device->address);
    
    return emm42_send_command(device, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度控制
// 参数说明     *device             设备结构体指针
// 参数说明     speed               速度 (rpm, 带符号)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_speed_control(&emm42_device, 300);
// 备注信息     正值顺时针，负值逆时针
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_speed_control(emm42_device_struct *device, int16 speed)
{
    if(!device) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    uint8 data[2];
    
    data[0] = (uint8)(speed & 0xFF);        // 低字节
    data[1] = (uint8)((speed >> 8) & 0xFF); // 高字节
    
    emm42_build_frame(command, device->address, 0xF6, data, 2);
    
    EMM42_DEBUG_PRINTF("速度控制: 地址=%d, 速度=%d rpm\r\n", device->address, speed);
    
    return emm42_send_command(device, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置控制
// 参数说明     *device             设备结构体指针
// 参数说明     position            目标位置 (脉冲数)
// 参数说明     absolute            1-绝对位置 0-相对位置
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_position_control(&emm42_device, 1600, 1);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_position_control(emm42_device_struct *device, int32 position, uint8 absolute)
{
    if(!device) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    uint8 data[4];
    uint8 cmd = absolute ? 0xFD : 0xF4;
    
    data[0] = (uint8)(position & 0xFF);         // 位置低字节
    data[1] = (uint8)((position >> 8) & 0xFF);  // 位置次低字节
    data[2] = (uint8)((position >> 16) & 0xFF); // 位置次高字节
    data[3] = (uint8)((position >> 24) & 0xFF); // 位置高字节
    
    emm42_build_frame(command, device->address, cmd, data, 4);
    
    EMM42_DEBUG_PRINTF("位置控制: 地址=%d, %s位置=%d\r\n", 
                       device->address, absolute ? "绝对" : "相对", position);
    
    return emm42_send_command(device, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     旋转指定角度
// 参数说明     *device             设备结构体指针
// 参数说明     angle               角度 (度, 带符号)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_rotate_angle(&emm42_device, 90);
// 备注信息     正值顺时针，负值逆时针
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_rotate_angle(emm42_device_struct *device, float angle)
{
    if(!device) return EMM42_ERROR_PARAM;
    
    // 将角度转换为脉冲数
    int32 pulses = (int32)(angle * EMM42_PULSES_PER_REVOLUTION / 360.0);
    
    EMM42_DEBUG_PRINTF("角度控制: 地址=%d, 角度=%.1f度, 脉冲数=%d\r\n", 
                       device->address, angle, pulses);
    
    return emm42_position_control(device, pulses, 0); // 相对位置模式
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停止
// 参数说明     *device             设备结构体指针
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_emergency_stop(&emm42_device);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_emergency_stop(emm42_device_struct *device)
{
    if(!device) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    
    emm42_build_frame(command, device->address, 0xFE, NULL, 0);
    
    EMM42_DEBUG_PRINTF("紧急停止: 地址=%d\r\n", device->address);
    
    return emm42_send_command(device, command, 8);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取设备参数
// 参数说明     *device             设备结构体指针
// 参数说明     param_type          参数类型
// 参数说明     *value              参数值指针
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_read_param(&emm42_device, 0x30, &status);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_read_param(emm42_device_struct *device, uint8 param_type, uint8 *value)
{
    if(!device || !value) return EMM42_ERROR_PARAM;
    
    uint8 command[8];
    uint8 response[8];
    
    emm42_build_frame(command, device->address, param_type, NULL, 0);
    
    if(emm42_send_command(device, command, 8) != EMM42_ERROR_NONE)
    {
        EMM42_ERROR_PRINTF("发送读取参数命令失败: 地址=%d, 参数=0x%02X\r\n", 
                          device->address, param_type);
        return EMM42_ERROR_COMMUNICATION;
    }
    
    if(emm42_receive_response(device, response, 8) != 8)
    {
        EMM42_ERROR_PRINTF("接收参数响应失败: 地址=%d, 参数=0x%02X\r\n", 
                          device->address, param_type);
        return EMM42_ERROR_COMMUNICATION;
    }
    
    if(!emm42_verify_checksum(response, 8))
    {
        EMM42_ERROR_PRINTF("参数响应校验失败: 地址=%d, 参数=0x%02X\r\n", 
                          device->address, param_type);
        return EMM42_ERROR_CHECKSUM;
    }
    
    *value = response[3]; // 数据位
    
    EMM42_DEBUG_PRINTF("读取参数成功: 地址=%d, 参数=0x%02X, 值=0x%02X\r\n", 
                       device->address, param_type, *value);
    
    return EMM42_ERROR_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待运动完成
// 参数说明     *device             设备结构体指针
// 参数说明     timeout_ms          超时时间(毫秒)
// 返回参数     uint8               0-成功 其他-失败
// 使用示例     result = emm42_wait_for_completion(&emm42_device, 5000);
// 备注信息     通过读取电机状态寄存器判断运动是否完成
//-------------------------------------------------------------------------------------------------------------------
uint8 emm42_wait_for_completion(emm42_device_struct *device, uint32 timeout_ms)
{
    if(!device) return EMM42_ERROR_PARAM;
    
    uint32 start_time = system_get_time_ms();
    uint8 status = 0;
    uint8 result;
    
    EMM42_DEBUG_PRINTF("开始等待运动完成: 地址=%d, 超时=%dms\r\n", 
                       device->address, timeout_ms);
    
    while((system_get_time_ms() - start_time) < timeout_ms)
    {
        // 读取电机状态寄存器 (0x30是状态寄存器地址)
        result = emm42_read_param(device, 0x30, &status);
        if(result == EMM42_ERROR_NONE)
        {
            // 检查运动状态位 (bit0表示运动状态: 0-静止, 1-运动中)
            if((status & 0x01) == 0)
            {
                EMM42_DEBUG_PRINTF("运动完成: 地址=%d, 状态=0x%02X, 用时=%dms\r\n", 
                                   device->address, status, 
                                   (uint32)(system_get_time_ms() - start_time));
                return EMM42_ERROR_NONE;
            }
            
            EMM42_DEBUG_PRINTF("运动中: 地址=%d, 状态=0x%02X\r\n", 
                               device->address, status);
        }
        else
        {
            EMM42_ERROR_PRINTF("读取运动状态失败: 地址=%d, 错误=%d\r\n", 
                              device->address, result);
        }
        
        system_delay_ms(10); // 10ms检查间隔
    }
    
    EMM42_ERROR_PRINTF("等待运动完成超时: 地址=%d, 超时=%dms\r\n", 
                       device->address, timeout_ms);
    return EMM42_ERROR_TIMEOUT;
}
