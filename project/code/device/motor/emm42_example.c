//====================================================EMM42电机驱动使用示例====================================================
#include "emm42.h"

//====================================================全局变量====================================================
emm42_device_struct motor1;    // 电机1
emm42_device_struct motor2;    // 电机2
emm42_device_struct motor3;    // 电机3

//====================================================示例函数====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单个电机基础控制示例
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_single_motor_example();
// 备注信息     演示单个电机的基本控制功能
//-------------------------------------------------------------------------------------------------------------------
void emm42_single_motor_example(void)
{
    uint8 result;
    
    printf("=== 单个电机控制示例 ===\r\n");
    
    // 1. 初始化电机 (地址1, UART1, 115200波特率)
    result = emm42_init(&motor1, 1, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机初始化失败! 错误码: %d\r\n", result);
        return;
    }
    printf("电机初始化成功!\r\n");
    
    // 2. 使能电机
    result = emm42_enable_motor(&motor1, 1);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机使能失败! 错误码: %d\r\n", result);
        return;
    }
    printf("电机已使能!\r\n");
    
    // 3. 速度控制 - 顺时针300RPM
    printf("速度控制: 顺时针300RPM\r\n");
    result = emm42_speed_control(&motor1, 300);
    if(result != EMM42_ERROR_NONE)
    {
        printf("速度控制失败! 错误码: %d\r\n", result);
    }
    
    system_delay_ms(3000); // 运行3秒
    
    // 4. 停止电机
    printf("停止电机\r\n");
    result = emm42_speed_control(&motor1, 0);
    if(result != EMM42_ERROR_NONE)
    {
        printf("停止电机失败! 错误码: %d\r\n", result);
    }
    
    system_delay_ms(1000);
    
    // 5. 位置控制 - 相对位置移动90度
    printf("位置控制: 相对移动90度\r\n");
    result = emm42_rotate_angle(&motor1, 90.0);
    if(result != EMM42_ERROR_NONE)
    {
        printf("位置控制失败! 错误码: %d\r\n", result);
    }
    
    // 6. 等待运动完成
    result = emm42_wait_for_completion(&motor1, 5000);
    if(result == EMM42_ERROR_NONE)
    {
        printf("运动完成!\r\n");
    }
    else
    {
        printf("等待运动完成超时!\r\n");
    }
    
    // 7. 逆时针旋转180度
    printf("位置控制: 逆时针旋转180度\r\n");
    result = emm42_rotate_angle(&motor1, -180.0);
    if(result != EMM42_ERROR_NONE)
    {
        printf("位置控制失败! 错误码: %d\r\n", result);
    }
    
    // 等待运动完成
    result = emm42_wait_for_completion(&motor1, 5000);
    if(result == EMM42_ERROR_NONE)
    {
        printf("运动完成!\r\n");
    }
    
    // 8. 绝对位置控制
    printf("绝对位置控制: 移动到0位置\r\n");
    result = emm42_position_control(&motor1, 0, 1);  // 绝对位置模式
    if(result != EMM42_ERROR_NONE)
    {
        printf("绝对位置控制失败! 错误码: %d\r\n", result);
    }
    
    // 等待运动完成
    result = emm42_wait_for_completion(&motor1, 5000);
    if(result == EMM42_ERROR_NONE)
    {
        printf("绝对位置控制完成!\r\n");
    }
    
    // 9. 失能电机
    result = emm42_enable_motor(&motor1, 0);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机失能失败! 错误码: %d\r\n", result);
    }
    else
    {
        printf("电机已失能!\r\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     多个电机独立控制示例
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_multi_motor_example();
// 备注信息     演示多个电机的独立控制
//-------------------------------------------------------------------------------------------------------------------
void emm42_multi_motor_example(void)
{
    uint8 result;
    
    printf("=== 多电机独立控制示例 ===\r\n");
    
    // 1. 初始化三个电机 (使用同一个UART，不同地址)
    result = emm42_init(&motor1, 1, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机1初始化失败! 错误码: %d\r\n", result);
        return;
    }
    
    result = emm42_init(&motor2, 2, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机2初始化失败! 错误码: %d\r\n", result);
        return;
    }
    
    result = emm42_init(&motor3, 3, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机3初始化失败! 错误码: %d\r\n", result);
        return;
    }
    
    printf("所有电机初始化成功!\r\n");
    
    // 2. 使能所有电机
    emm42_enable_motor(&motor1, 1);
    emm42_enable_motor(&motor2, 1);
    emm42_enable_motor(&motor3, 1);
    printf("所有电机已使能!\r\n");
    
    // 3. 让每个电机执行不同的动作
    printf("电机1: 顺时针旋转180度\r\n");
    emm42_rotate_angle(&motor1, 180.0);
    
    printf("电机2: 逆时针旋转90度\r\n");
    emm42_rotate_angle(&motor2, -90.0);
    
    printf("电机3: 连续旋转300RPM\r\n");
    emm42_speed_control(&motor3, 300);
    
    // 4. 等待位置控制电机完成
    system_delay_ms(100); // 等待命令发送完成
    
    printf("等待电机1完成...\r\n");
    emm42_wait_for_completion(&motor1, 5000);
    
    printf("等待电机2完成...\r\n");
    emm42_wait_for_completion(&motor2, 5000);
    
    // 5. 停止电机3的连续旋转
    system_delay_ms(3000); // 让电机3旋转3秒
    printf("停止电机3\r\n");
    emm42_speed_control(&motor3, 0);
    
    // 6. 失能所有电机
    emm42_enable_motor(&motor1, 0);
    emm42_enable_motor(&motor2, 0);
    emm42_enable_motor(&motor3, 0);
    
    printf("所有电机已失能!\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机参数读取示例
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_read_param_example();
// 备注信息     演示如何读取电机参数
//-------------------------------------------------------------------------------------------------------------------
void emm42_read_param_example(void)
{
    uint8 result;
    uint8 status;
    
    printf("=== 电机参数读取示例 ===\r\n");
    
    // 初始化电机
    result = emm42_init(&motor1, 1, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机初始化失败!\r\n");
        return;
    }
    
    // 读取电机状态
    result = emm42_read_param(&motor1, 0x30, &status);
    if(result == EMM42_ERROR_NONE)
    {
        printf("电机状态: 0x%02X\r\n", status);
        printf("  - 运动状态: %s\r\n", (status & 0x01) ? "运动中" : "静止");
        printf("  - 使能状态: %s\r\n", (status & 0x02) ? "使能" : "失能");
        printf("  - 到位状态: %s\r\n", (status & 0x04) ? "到位" : "未到位");
    }
    else
    {
        printf("读取电机状态失败! 错误码: %d\r\n", result);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机紧急停止示例
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_emergency_stop_example();
// 备注信息     演示电机紧急停止功能
//-------------------------------------------------------------------------------------------------------------------
void emm42_emergency_stop_example(void)
{
    uint8 result;
    
    printf("=== 电机紧急停止示例 ===\r\n");
    
    // 初始化电机
    result = emm42_init(&motor1, 1, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机初始化失败!\r\n");
        return;
    }
    
    // 使能电机
    result = emm42_enable_motor(&motor1, 1);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机使能失败!\r\n");
        return;
    }
    
    // 启动高速旋转
    printf("启动高速旋转: 1500RPM\r\n");
    result = emm42_speed_control(&motor1, 1500);
    if(result != EMM42_ERROR_NONE)
    {
        printf("速度控制失败!\r\n");
        return;
    }
    
    // 运行2秒后紧急停止
    system_delay_ms(2000);
    printf("执行紧急停止...\r\n");
    result = emm42_emergency_stop(&motor1);
    if(result != EMM42_ERROR_NONE)
    {
        printf("紧急停止失败! 错误码: %d\r\n", result);
    }
    else
    {
        printf("紧急停止成功!\r\n");
    }
    
    // 失能电机
    system_delay_ms(1000);
    emm42_enable_motor(&motor1, 0);
    printf("电机已失能!\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置控制详细示例
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_position_control_example();
// 备注信息     演示不同的位置控制方式
//-------------------------------------------------------------------------------------------------------------------
void emm42_position_control_example(void)
{
    uint8 result;
    
    printf("=== 位置控制详细示例 ===\r\n");
    
    // 初始化电机
    result = emm42_init(&motor1, 1, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机初始化失败!\r\n");
        return;
    }
    
    // 使能电机
    result = emm42_enable_motor(&motor1, 1);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机使能失败!\r\n");
        return;
    }
    
    // 1. 相对位置控制 - 顺时针90度
    printf("相对位置控制: 顺时针90度\r\n");
    result = emm42_rotate_angle(&motor1, 90.0);
    if(result == EMM42_ERROR_NONE)
    {
        emm42_wait_for_completion(&motor1, 3000);
        printf("第一段运动完成\r\n");
    }
    
    system_delay_ms(1000);
    
    // 2. 相对位置控制 - 逆时针180度
    printf("相对位置控制: 逆时针180度\r\n");
    result = emm42_rotate_angle(&motor1, -180.0);
    if(result == EMM42_ERROR_NONE)
    {
        emm42_wait_for_completion(&motor1, 3000);
        printf("第二段运动完成\r\n");
    }
    
    system_delay_ms(1000);
    
    // 3. 绝对位置控制 - 回到0度位置
    printf("绝对位置控制: 回到0度位置\r\n");
    result = emm42_position_control(&motor1, 0, 1);  // 绝对位置模式
    if(result == EMM42_ERROR_NONE)
    {
        emm42_wait_for_completion(&motor1, 3000);
        printf("回零完成\r\n");
    }
    
    // 4. 使用脉冲数进行精确控制
    printf("脉冲控制: 移动800脉冲\r\n");
    result = emm42_position_control(&motor1, 800, 0);  // 相对位置模式，800脉冲
    if(result == EMM42_ERROR_NONE)
    {
        emm42_wait_for_completion(&motor1, 3000);
        printf("脉冲控制完成\r\n");
    }
    
    // 失能电机
    emm42_enable_motor(&motor1, 0);
    printf("位置控制示例完成!\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度控制详细示例
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_speed_control_example();
// 备注信息     演示不同的速度控制方式
//-------------------------------------------------------------------------------------------------------------------
void emm42_speed_control_example(void)
{
    uint8 result;
    
    printf("=== 速度控制详细示例 ===\r\n");
    
    // 初始化电机
    result = emm42_init(&motor1, 1, UART_1, 115200);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机初始化失败!\r\n");
        return;
    }
    
    // 使能电机
    result = emm42_enable_motor(&motor1, 1);
    if(result != EMM42_ERROR_NONE)
    {
        printf("电机使能失败!\r\n");
        return;
    }
    
    // 1. 低速旋转
    printf("低速旋转: 200RPM\r\n");
    emm42_speed_control(&motor1, EMM42_SPEED_SLOW);
    system_delay_ms(3000);
    
    // 2. 中速旋转
    printf("中速旋转: 500RPM\r\n");
    emm42_speed_control(&motor1, EMM42_SPEED_MEDIUM);
    system_delay_ms(3000);
    
    // 3. 高速旋转
    printf("高速旋转: 1000RPM\r\n");
    emm42_speed_control(&motor1, EMM42_SPEED_FAST);
    system_delay_ms(3000);
    
    // 4. 逆时针旋转
    printf("逆时针旋转: -800RPM\r\n");
    emm42_speed_control(&motor1, -800);
    system_delay_ms(3000);
    
    // 5. 停止
    printf("停止电机\r\n");
    emm42_speed_control(&motor1, 0);
    system_delay_ms(1000);
    
    // 失能电机
    emm42_enable_motor(&motor1, 0);
    printf("速度控制示例完成!\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主测试函数
// 参数说明     void
// 返回参数     void
// 使用示例     emm42_test();
// 备注信息     运行所有测试示例
//-------------------------------------------------------------------------------------------------------------------
void emm42_test(void)
{
    printf("开始EMM42电机控制测试...\r\n\r\n");
    
    // 单电机控制示例
    emm42_single_motor_example();
    system_delay_ms(2000);
    
    // 参数读取示例
    emm42_read_param_example();
    system_delay_ms(1000);
    
    // 紧急停止示例
    emm42_emergency_stop_example();
    system_delay_ms(2000);
    
    // 位置控制详细示例
    emm42_position_control_example();
    system_delay_ms(2000);
    
    // 速度控制详细示例
    emm42_speed_control_example();
    system_delay_ms(2000);
    
    // 多电机控制示例
    emm42_multi_motor_example();
    
    printf("\r\nEMM42电机控制测试完成!\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主函数示例
// 参数说明     void
// 返回参数     void
// 使用示例     在main函数中调用
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void emm42_main_example(void)
{
    printf("EMM42驱动库使用示例\r\n");
    printf("====================\r\n");
    
    // 运行完整测试
    emm42_test();
    
    printf("所有示例执行完成!\r\n");
}
