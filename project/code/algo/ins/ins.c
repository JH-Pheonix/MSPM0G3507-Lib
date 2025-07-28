/*
 * ins.c - 惯导系统实现
 * 移植自Guandao_Plus.c的惯导算法
 * 适配MSPM0G3507库结构
 */

#include "ins.h"

// 全局变量定义
ins_system_t ins_system = {0};
path_memory_t path_memory = {0};

// 静态变量
static int target_index = 0;                     // 小车目标点
static const int32_t MAX_PHYSICAL_DELTA = 1000;  // 最大允许物理增量

/**
 * @brief 计算编码器增量，自动处理int16溢出
 * @param current 当前编码器值
 * @param last 上一次编码器值
 * @return 修正后的增量
 */
static inline int32_t calculate_delta(int16_t current, int16_t last)
{
    int32_t delta = (int32_t)current - (int32_t)last;

    // 处理正向溢出（例如从32767到-32768，实际增量为+1）
    if (delta > 32767) {
        delta -= 65536; // 65536 = 2^16
    }
    // 处理反向溢出（例如从-32768到32767，实际增量为-1）
    else if (delta < -32768) {
        delta += 65536;
    }

    return delta;
}

/**
 * @brief 惯导系统初始化
 */
void ins_init(void)
{
    // 初始化IMU
    imu_init(IMU_DEVICE_ADIS16505);  // 根据实际使用的IMU修改
    
    // 初始化编码器
    encoder_init(ENCODER_INCR);
    
    // 初始化惯导系统状态
    ins_system.x_position = 0.0f;
    ins_system.y_position = 0.0f;
    ins_system.yaw_angle = 0.0f;
    ins_system.speed_left = 0;
    ins_system.speed_right = 0;
    ins_system.left_wheel_speed_cmps = 0.0f;
    ins_system.right_wheel_speed_cmps = 0.0f;
    ins_system.current_avg_speed_cmps = 0.0f;
    ins_system.acc_y_speed = 0.0f;
    ins_system.wheel_slip_flags = SLIP_FLAG_NONE;
    ins_system.err_guandao_plus = 0;
    ins_system.e_lat = 0.0f;
    ins_system.locate_index = 0;
    
    // 初始化左轮数据结构体
    ins_system.wheel_left.total_pulses = 0;
    ins_system.wheel_left.last_total_pulses = 0;
    ins_system.wheel_left.encoder_last = 0;
    ins_system.wheel_left.encoder_prev_speed = 0;
    ins_system.wheel_left.target_pulses = ENCODER_PULSES_PER_REVOLUTION;
    
    // 初始化右轮数据结构体
    ins_system.wheel_right.total_pulses = 0;
    ins_system.wheel_right.last_total_pulses = 0;
    ins_system.wheel_right.encoder_last = 0;
    ins_system.wheel_right.encoder_prev_speed = 0;
    ins_system.wheel_right.target_pulses = ENCODER_PULSES_PER_REVOLUTION;
    
    // 初始化路径记忆系统
    path_memory.road_memory_finish_flag = 0;
    path_memory.road_memory_start_flag = 0;
    path_memory.road_recurrent_flag = 0;
    path_memory.num_l = 0;
    path_memory.num_r = 0;
    path_memory.road_destination = 0;
    
    // 清零路径数组
    for (int i = 0; i < MAX_PATH_POINTS; i++) {
        path_memory.x_memory[i] = 0.0f;
        path_memory.y_memory[i] = 0.0f;
        path_memory.x_memory_store[i] = 0.0f;
        path_memory.y_memory_store[i] = 0.0f;
    }
}

/**
 * @brief 更新惯导系统状态
 * @param dt 时间间隔（秒）
 */
void ins_update(float dt)
{
    // 读取IMU数据
    ins_system.imu_data = imu_get_data();
    
    // 读取编码器数据
    encoder_data_t left_encoder = encoder_read(0);
    encoder_data_t right_encoder = encoder_read(1);
    
    // 更新编码器速度（假设encoder_read返回的velocity就是脉冲增量）
    ins_system.speed_left = (int32_t)(left_encoder.velocity);
    ins_system.speed_right = (int32_t)(right_encoder.velocity);
    
    // 更新姿态角（积分角速度）
    // 注意：这里需要将角速度从度/秒转换为弧度/秒
    ins_system.yaw_angle += ins_system.imu_data.gyro_z * dt * (M_PI / 180.0f);
    
    // 归一化偏航角到[-π, π]
    while (ins_system.yaw_angle > M_PI) {
        ins_system.yaw_angle -= 2.0f * M_PI;
    }
    while (ins_system.yaw_angle < -M_PI) {
        ins_system.yaw_angle += 2.0f * M_PI;
    }
    
    // 更新加速度积分速度（假设y轴为前进方向）
    ins_system.acc_y_speed += ins_system.imu_data.accel_y * dt;
    
    // 执行打滑检测（这会更新轮速）
    slip_check();
    
    // === 实时位置更新 ===
    // 计算平均轮速（厘米/秒）
    float avg_wheel_speed = 0.5f * (ins_system.left_wheel_speed_cmps + ins_system.right_wheel_speed_cmps);
    
    // 方法1：基于编码器的位置积分（推荐）
    // 计算本次时间间隔内的行驶距离（厘米）
    float distance_traveled = avg_wheel_speed * dt;
    
    // 根据当前偏航角更新位置（东天北坐标系）
    ins_system.x_position += distance_traveled * (-sinf(ins_system.yaw_angle));
    ins_system.y_position += distance_traveled * cosf(ins_system.yaw_angle);
}

/**
 * @brief 更新轮子脉冲累计值并处理虚拟清零
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return 前进厘米数
 */
int32_t update_wheel_pulses(WheelData *wheel, int16_t current_encoder)
{
    // 1. 计算增量 (包含溢出处理)
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_last);
    int32_t forward_crossings = 0;

    // 2. 智能跳变检测逻辑
    if (abs(delta) > 1000) {
        // 重置累计脉冲，避免跳变干扰后续计数
        wheel->total_pulses = 0;
        forward_crossings = 0; // 确保跳变时返回0
    }
    else {
        // 保存上一次的累计脉冲值
        wheel->last_total_pulses = wheel->total_pulses;

        // 更新累计脉冲数（支持正反转）
        wheel->total_pulses += delta;

        // 计算前向阈值穿越次数
        if (wheel->total_pulses >= wheel->target_pulses) {
            forward_crossings = wheel->total_pulses / wheel->target_pulses;
            wheel->total_pulses %= wheel->target_pulses;
        }
        // 处理反向穿越
        else if (wheel->total_pulses <= -wheel->target_pulses) {
            wheel->total_pulses %= wheel->target_pulses;
            if (wheel->total_pulses == -wheel->target_pulses) {
                wheel->total_pulses = 0;
            }
        }
    }

    // 3. 保存当前编码器值供下次使用
    wheel->encoder_last = current_encoder;

    // 4. 返回本次计算出的前进厘米数
    return forward_crossings;
}

/**
 * @brief 获取轮子在固定时间窗口内的速度（脉冲增量）
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return 时间窗口内的脉冲增量
 */
int32_t get_wheel_speed(WheelData *wheel, int16_t current_encoder)
{
    // 计算当前编码器值与上一次中断时的基准值的增量
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_prev_speed);

    // 更新基准值为当前值，用于下一次速度计算
    wheel->encoder_prev_speed = current_encoder;

    if (abs(delta) > 1000)
        delta = 0;
    return delta;
}

/**
 * @brief 路径记忆函数
 */
void distance_get_plus(void)
{
    // 确保不会越界访问数组
    if (path_memory.num_l >= MAX_PATH_POINTS - 2) {
        path_memory.road_memory_finish_flag = 1; // 路径记忆完成标志位
        return; // 直接返回，不再记录新的点
    }

    path_memory.num_l++;
    ins_system.x_position += DISTANCE_INTERVAL * (-sinf(ins_system.yaw_angle));
    ins_system.y_position += DISTANCE_INTERVAL * cosf(ins_system.yaw_angle);
    path_memory.x_memory[path_memory.num_l] = ins_system.x_position;
    path_memory.y_memory[path_memory.num_l] = ins_system.y_position;
}

/**
 * @brief 查找距离当前位姿L厘米的预瞄点索引
 * @param L 预瞄距离（单位：厘米）
 */
void find_lookahead_index(float L)
{
    // 步骤1：找到距离当前位置最近的前方路径点
    float min_distance = FLT_MAX;
    int closest_index = 0;

    // 搜索范围控制
    int search_start = target_index - LOOKAHEAD_SEARCH_RANGE;
    int search_end = target_index + LOOKAHEAD_SEARCH_RANGE;

    // 边界检查
    if (search_start < 0)
        search_start = 0;
    if (search_end >= MAX_PATH_POINTS)
        search_end = MAX_PATH_POINTS - 1;

    // 在设定范围内寻找距离当前位置最近的前方路径点
    for (int i = search_start; i <= search_end; i++) {
        float dx = path_memory.x_memory_store[i] - ins_system.x_position;
        float dy = path_memory.y_memory_store[i] - ins_system.y_position;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist < min_distance) {
            min_distance = dist;
            closest_index = i;
        }
    }

    // 更新定位点
    ins_system.locate_index = closest_index;

    // 步骤2：跳跃式搜索策略
    int search_limit = closest_index + LOOKAHEAD_MAX_STEPS;
    if (search_limit >= MAX_PATH_POINTS)
        search_limit = MAX_PATH_POINTS - 1;

    // 估算跳跃步数
    int jump_steps = (int)(L / LOOKAHEAD_AVG_DIST);
    if (jump_steps < 1)
        jump_steps = 1;

    // 计算跳跃目标点
    int jump_target = closest_index + jump_steps;
    if (jump_target > search_limit)
        jump_target = search_limit;

    // 计算跳跃点到当前位置的实际距离
    float jump_dx = path_memory.x_memory_store[jump_target] - ins_system.x_position;
    float jump_dy = path_memory.y_memory_store[jump_target] - ins_system.y_position;
    float jump_dist = sqrtf(jump_dx * jump_dx + jump_dy * jump_dy);

    // 步骤3：确定精确搜索范围
    int fine_search_start, fine_search_end;

    if (jump_dist < L) {
        fine_search_start = jump_target;
        fine_search_end = search_limit;
    }
    else {
        fine_search_start = closest_index;
        fine_search_end = jump_target;
    }

    // 步骤4：精确搜索预瞄点
    for (int i = fine_search_start; i <= fine_search_end; i++) {
        float dx = path_memory.x_memory_store[i] - ins_system.x_position;
        float dy = path_memory.y_memory_store[i] - ins_system.y_position;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist >= L) {
            target_index = i;
            return;
        }
    }

    // 边界处理
    target_index = fine_search_end;
}

/**
 * @brief 计算预瞄点的横向误差（右前上坐标系）
 * @param lookahead_index 预瞄点索引
 * @return 横向误差（厘米，正值表示预瞄点在右侧）
 */
float calculate_e_lat(int lookahead_index)
{
    float dx = path_memory.x_memory_store[lookahead_index] - ins_system.x_position;
    float dy = path_memory.y_memory_store[lookahead_index] - ins_system.y_position;
    
    // 使用航向角进行坐标变换
    float rotated_x = dx * cosf(ins_system.yaw_angle) + dy * sinf(ins_system.yaw_angle);
    return rotated_x; // 正值表示预瞄点在小车右侧
}

/**
 * @brief 纯跟踪控制算法主函数
 * @return 差速控制量（负值表示左转，正值表示右转）
 */
int16_t pure_pursuit_control(void)
{
    // 设置预瞄距离（单位：厘米）
    static float L_distance = 20.0f;

    // 使用"距离法"查找预瞄点
    find_lookahead_index(L_distance);

    // 计算横向误差
    ins_system.e_lat = calculate_e_lat(target_index);

    // 计算预瞄点到当前位置的实际距离
    float dx = path_memory.x_memory_store[target_index] - ins_system.x_position;
    float dy = path_memory.y_memory_store[target_index] - ins_system.y_position;
    float L_actual = sqrtf(dx * dx + dy * dy);

    // 防止除零
    if (L_actual < 1e-3f)
        L_actual = 1e-3f;

    // 计算曲率 - 纯跟踪控制核心公式
    float curvature = 2.0f * ins_system.e_lat / (L_actual * L_actual);

    // 将曲率转换为差速控制量
    int16_t delta_v = (int16_t)(curvature * WHEELBASE * SCALE_FACTOR);

    // 差速限制
    if (delta_v > 200)
        delta_v = 200;
    if (delta_v < -200)
        delta_v = -200;

    ins_system.err_guandao_plus = delta_v;
    return delta_v;
}

/**
 * @brief 更新轮速（将脉冲转换为实际轮速物理量）
 * @param left_pulses 左轮脉冲增量
 * @param right_pulses 右轮脉冲增量
 */
static void update_wheel_speeds(int32_t left_pulses, int32_t right_pulses)
{
    // 脉冲转线速度：脉冲数 → 厘米/秒
    static float scale = (1000.0f / SAMPLING_INTERVAL_MS);

    ins_system.left_wheel_speed_cmps = (float)left_pulses / ENCODER_PULSES_PER_REVOLUTION * scale;
    ins_system.right_wheel_speed_cmps = (float)right_pulses / ENCODER_PULSES_PER_REVOLUTION * scale;
}

/**
 * @brief 检测横向滑移（差速转向不匹配）
 */
static void detect_lateral_slip(void)
{
    // 计算理论偏航率（单位：弧度/秒）
    float theoretical_yaw_rate = (ins_system.right_wheel_speed_cmps - ins_system.left_wheel_speed_cmps) 
                                / WHEEL_BASE_CM;

    // 计算偏航率偏差（绝对值）
    float yaw_rate_deviation = fabsf(theoretical_yaw_rate - ins_system.imu_data.gyro_z * (M_PI / 180.0f));

    // 计算平均速度
    float avg_speed_cmps = 0.5f * (fabsf(ins_system.left_wheel_speed_cmps) + 
                                  fabsf(ins_system.right_wheel_speed_cmps));

    // 检测逻辑
    if (avg_speed_cmps > MIN_LATERAL_DETECT_SPEED_CMPS &&
        yaw_rate_deviation > LATERAL_SLIP_YAW_RATE_THRESHOLD) {
        
        if (theoretical_yaw_rate > ins_system.imu_data.gyro_z * (M_PI / 180.0f)) {
            ins_system.wheel_slip_flags |= SLIP_FLAG_LATERAL_RIGHT; // 右轮打滑
        }
        else {
            ins_system.wheel_slip_flags |= SLIP_FLAG_LATERAL_LEFT; // 左轮打滑
        }
    }
}

/**
 * @brief 检测纵向滑移（驱动轮空转或抱死）
 */
static void detect_longitudinal_slip(void)
{
    // 计算当前平均轮速（单位：米/秒）
    ins_system.current_avg_speed_cmps = 0.5f * (ins_system.left_wheel_speed_cmps + 
                                               ins_system.right_wheel_speed_cmps) / 100.0f;

    float accel_diff = fabsf(ins_system.current_avg_speed_cmps - ins_system.acc_y_speed);

    // 检测逻辑
    if (ins_system.current_avg_speed_cmps > MIN_LONGITUDINAL_DETECT_SPEED_CMPS) {
        if (accel_diff > LONGITUDINAL_ACCEL_DIFF_THRESHOLD) {
            ins_system.wheel_slip_flags |= SLIP_FLAG_LONGITUDINAL_ACCEL; // 滑移
        }
    }
}

/**
 * @brief 打滑检测主函数
 */
void slip_check(void)
{
    // 清除之前的打滑标志
    ins_system.wheel_slip_flags = SLIP_FLAG_NONE;
    
    // 更新轮速
    update_wheel_speeds(ins_system.speed_left, ins_system.speed_right);
    
    // 执行滑移检测
    detect_longitudinal_slip();
    detect_lateral_slip();
}

/**
 * @brief 获取当前实时位置
 * @param x_pos 输出X坐标指针（单位：厘米）
 * @param y_pos 输出Y坐标指针（单位：厘米）
 */
void ins_get_position(float *x_pos, float *y_pos)
{
    if (x_pos != NULL) {
        *x_pos = ins_system.x_position;
    }
    if (y_pos != NULL) {
        *y_pos = ins_system.y_position;
    }
}

/**
 * @brief 获取当前偏航角
 * @return 偏航角（弧度）
 */
float ins_get_yaw(void)
{
    return ins_system.yaw_angle;
}

/**
 * @brief 获取当前速度信息
 * @param left_speed 左轮速度指针（厘米/秒）
 * @param right_speed 右轮速度指针（厘米/秒）
 * @param avg_speed 平均速度指针（厘米/秒）
 */
void ins_get_velocity(float *left_speed, float *right_speed, float *avg_speed)
{
    if (left_speed != NULL) {
        *left_speed = ins_system.left_wheel_speed_cmps;
    }
    if (right_speed != NULL) {
        *right_speed = ins_system.right_wheel_speed_cmps;
    }
    if (avg_speed != NULL) {
        *avg_speed = 0.5f * (ins_system.left_wheel_speed_cmps + ins_system.right_wheel_speed_cmps);
    }
}

/**
 * @brief 重置位置到原点
 */
void ins_reset_position(void)
{
    ins_system.x_position = 0.0f;
    ins_system.y_position = 0.0f;
}

/**
 * @brief 设置当前位置
 * @param x_pos X坐标（厘米）
 * @param y_pos Y坐标（厘米）
 */
void ins_set_position(float x_pos, float y_pos)
{
    ins_system.x_position = x_pos;
    ins_system.y_position = y_pos;
}