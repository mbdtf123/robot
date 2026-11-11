/**
 * @file main.cpp
 * @brief 轮足机器人平衡控制系统 - 完整功能版本
 * 
 * 工作流程：
 * 1. 上电设置腿的位置（运动学逆解）
 * 2. 6050获取偏转角
 * 3. 电机根据偏转角控制力矩实现平衡
 * 4. 蓝牙控制命令接收和处理
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// 导入项目模块
#include "duoji_control.h"
#include "esp_hw_link.h"
#include "com_uart.h"
#include "Kms.h"  // 包含运动学逆解函数
#include "6050_middle.h"  // 包含6050驱动函数

// 蓝牙模块 - 严格按照您提供的可行方案
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "nvs_flash.h"
#include "common.h"  // 包含蓝牙基础头文件和DEVICE_NAME定义
#include "gap.h"
#include "gatt_svc.h"

#define TAG "ROBOT_BALANCE"

// ==================== 机器人参数配置 ====================
#define CONTROL_INTERVAL_MS     10      // 控制周期10ms
#define LEG_LENGTH_1           70.0f  // 大腿长度(mm)
#define LEG_LENGTH_2           120.0f  // 小腿长度(mm)
#define STAND_HEIGHT           90.0f  // 站立高度(mm)
#define STAND_FORWARD          40.0f   // 站立前倾距离(mm)
#define PI                     3.14159265358979323846f

// ==================== PID参数配置 ====================
// 第六课PID参数结构体（用于单高度测试）
struct PIDParams {
    float vel_kp;           // 速度环比例系数
    float balance_kp;       // 直立环比例系数
    float balance_kd;       // 直立环微分系数
    float robot_kp;         // 机器人位置控制比例系数
};

// 当前使用的PID参数（根据第六课经验值）
static PIDParams current_pid = {
    .vel_kp = -0.05f,       // 速度环Kp
    .balance_kp = 0.15f,   // 直立环Kp
    .balance_kd = -0.02f,    // 直立环Kd
    .robot_kp = 0.0f        // 位置环Kp（暂时不用）
};

static float max_torque = 1.0f;     // 最大力矩限制
static float torque_normalization_factor = 1.0f / max_torque;  // 归一化因子
static float max_velocity = 2.0f;   // 最大速度限制

// ==================== 全局变量 ====================
static bool robot_running = false;

// 机器人姿态数据（需要从6050读取）
static float current_pitch = 0.0f;      // 当前俯仰角
static float current_gyro_y = 0.0f;     // 当前角速度Y

// 速度控制相关
static float target_velocity = 0.0f;    // 目标速度
static float current_velocity = 0.0f;   // 当前速度（通过轮子编码器获取）
static float wheel_velocity_left = 0.0f;  // 左轮速度
static float wheel_velocity_right = 0.0f; // 右轮速度

// 位置控制相关
static float target_position = 0.0f;    // 目标位置（原点）
static float current_position = 0.0f;   // 当前位置（通过积分速度得到）

// 腿部位置防倒机制相关
static float leg_position_offset = 0.0f;  // 腿部位置偏移量（用于防倒）
static float leg_original_x = STAND_FORWARD;  // 腿部原始X位置

// 腿部目标位置
static float leg_target_x = STAND_FORWARD;  // X轴位置
static float leg_target_y = STAND_HEIGHT;   // Y轴位置

// ==================== 函数声明 ====================
static void robot_control_task(void *pvParameters);
static void initialize_robot(void);
static float calculate_wheel_torque(void);
static void read_sensor_data(void);
static void read_wheel_velocity(void);
static void adjust_leg_position_for_balance(void);
static void parse_balance_command(const char *cmd, int len);
static void ble_host_sync_cb(void);
static void ble_host_task(void *param);
static void print_startup_info(void);
static void set_leg_position_smooth(float x, float y, int duration_ms);

// ==================== 主函数 ====================
extern "C" void app_main(void)
{
    esp_err_t ret;
    
    /* 打印启动信息 */
    print_startup_info();
    
    // 1. 初始化机器人系统
    initialize_robot();
    
    // 2. 创建控制任务
    xTaskCreate(robot_control_task, "balance_control", 4096, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "✓ 机器人平衡控制系统启动完成");
    
    /* 蓝牙系统初始化 */
    
    /* 1. 初始化 NVS 闪存 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupted, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS flash initialized");
    
    /* 2. 初始化 NimBLE 主机 */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ NimBLE port init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "✓ NimBLE port initialized");
    
    /* 3. 初始化 GAP 服务 */
    if (gap_init() != 0) {
        ESP_LOGE(TAG, "✗ GAP initialization failed");
        return;
    }
    ESP_LOGI(TAG, "✓ GAP service initialized");
    
    /* 4. 初始化 GATT 消息服务 */
    ble_cmd_init();
    ESP_LOGI(TAG, "✓ GATT message service initialized");
    
    /* 5. 设置消息接收回调 */
    ble_cmd_set_callback(parse_balance_command);
    ESP_LOGI(TAG, "✓ Message callback registered");
    
    /* 6. 启动 NimBLE 主机任务 */
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "✓ 轮足机器人蓝牙控制系统就绪!");
    ESP_LOGI(TAG, "设备名称: 轮足机器人");
    ESP_LOGI(TAG, "等待蓝牙连接...");
    ESP_LOGI(TAG, "可用命令: start, stop, height:XXX, forward:XXX, speed:X.XX");
    ESP_LOGI(TAG, "PID调参: kp:X.XX, kd:X.XX, velkp:X.XX, robotkp:X.XX, setpid:vel_kp,balance_kp,balance_kd,robot_kp");
    ESP_LOGI(TAG, "状态查询: status");
    ESP_LOGI(TAG, "==============================================");
    
    /* 主循环 - 保持运行 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== 系统初始化 ====================
static void initialize_robot(void)
{
    ESP_LOGI(TAG, "正在初始化机器人系统...");
    
    // 1. 初始化串口通信
    com_uart_init();
    ESP_LOGI(TAG, "✓ 串口通信初始化完成");
    
    // 2. 初始化舵机系统
    duoji_init(115200);
    ESP_LOGI(TAG, "✓ 舵机系统初始化完成");
    
    // 3. 初始化FOC电机控制系统
    enc_init(0, 7, 15);
    enc_init(1, 8, 3);
    motor_foc_init();
    ESP_LOGI(TAG, "✓ FOC电机控制系统初始化完成");

    // 4. 初始化MPU6050姿态传感器（软件I2C: SDA=GPIO9, SCL=GPIO10）
    mpu6050_init();
    vTaskDelay(pdMS_TO_TICKS(500)); // 等待传感器稳定和校准完成
    ESP_LOGI(TAG, "✓ MPU6050姿态传感器初始化完成");

    
    // 5. 设置机器人站立姿态（运动学逆解）- 缓慢移动
    set_leg_position_smooth(leg_target_x, leg_target_y, 2000); // 2秒缓慢移动
    
    ESP_LOGI(TAG, "✓ 机器人系统初始化完成");
    ESP_LOGI(TAG, "初始站立位置: X=%.1fmm, Y=%.1fmm", leg_target_x, leg_target_y);
}

// ==================== 机器人控制任务 ====================
static void robot_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "机器人平衡控制任务启动");
    
    int64_t last_control_time = esp_timer_get_time();
    int64_t last_print_time = last_control_time;
    
    while (1) {
        int64_t current_time = esp_timer_get_time();
        
        // 控制周期检查（10ms）
        if ((current_time - last_control_time) >= (CONTROL_INTERVAL_MS * 1000)) {
            last_control_time = current_time;
            
            if (robot_running) {
                // 1. 读取传感器数据（6050）
                read_sensor_data();
                
                // 2. 读取轮子速度（编码器）
                read_wheel_velocity();
                
                // 3. 计算轮子力矩
                float torque = calculate_wheel_torque();
                
                // 4. 应用力矩到电机（已经是归一化后的值）
                motor_torque(0, torque);  // 左轮
                motor_torque(1, torque);  // 右轮
                
                // 5. 更新电机控制
                motor_update(0);
                motor_update(1);
                
                // 6. 腿部位置防倒机制（根据俯仰角调整腿部位置）
                adjust_leg_position_for_balance();
                
                // 打印周期检查（1秒）
                if ((current_time - last_print_time) >= (1000 * 1000)) {
                    last_print_time = current_time;
                    
                    if (robot_running) {
                        ESP_LOGI(TAG, "平衡状态 - 俯仰角: %.2f°, 角速度Y: %.2f°/s, 速度: %.2fm/s, 腿部偏移: %.1fmm, 力矩: %.3f", 
                                current_pitch, current_gyro_y, current_velocity, leg_position_offset, torque);
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== 缓慢设置腿部位置 ====================
static void set_leg_position_smooth(float x, float y, int duration_ms)
{
    ESP_LOGI(TAG, "缓慢设置腿部位置: X=%.1fmm, Y=%.1fmm, 耗时: %dms", x, y, duration_ms);
    
    // 使用您现有的运动学逆解函数
    Node left_target = {x, y};
    Node right_target = {x, y};  // 左右腿对称
    JointAngles joint_angles;
    
    inverse_kinematics(&left_target, &right_target, &joint_angles);
    
    // 角度换算和限制
    float alpha_left_deg = joint_angles.alpha_left * 180.0f / PI;
    float beta_left_deg = joint_angles.beta_left * 180.0f / PI;
    float alpha_right_deg = joint_angles.alpha_right * 180.0f / PI;
    float beta_right_deg = joint_angles.beta_right * 180.0f / PI;
    
    // 角度限制
    if (alpha_left_deg < 80.0f) alpha_left_deg = 80.0f;
    if (alpha_left_deg > 200.0f) alpha_left_deg = 200.0f;
    if (beta_left_deg < -20.0f) beta_left_deg = -20.0f;
    if (beta_left_deg > 80.0f) beta_left_deg = 80.0f;
    
    // 应用角度偏移
    float s0_rot_left = alpha_left_deg - 90.0f;
    float s2_rot_left = 90.0f - beta_left_deg;
    
    // 限制舵机旋转范围
    if (s0_rot_left < 0.0f) s0_rot_left = 0.0f;
    if (s0_rot_left > 130.0f) s0_rot_left = 130.0f;
    if (s2_rot_left < 0.0f) s2_rot_left = 0.0f;
    if (s2_rot_left > 120.0f) s2_rot_left = 120.0f;
    
    // 转换为脉冲
    #define PULSE_PER_DEG 7.4f
    #define OFFSET_LF 130      // ID2 - 左前腿（大腿）
    #define OFFSET_LR -130      // ID4 - 左后腿（小腿）
    #define OFFSET_RR 0     // ID1 - 右前腿（大腿）
    #define OFFSET_RF  -75     // ID3 - 右后腿（小腿）
    
    uint16_t pulse_alpha_left = 1500 + OFFSET_LF + (int)(s0_rot_left * PULSE_PER_DEG);
    uint16_t pulse_beta_left = 1500 + OFFSET_LR + (int)(s2_rot_left * PULSE_PER_DEG);
    
    // 右腿同理
    float s0_rot_right = alpha_right_deg - 90.0f;
    float s2_rot_right = 90.0f - beta_right_deg;
    
    if (s0_rot_right < 0.0f) s0_rot_right = 0.0f;
    if (s0_rot_right > 90.0f) s0_rot_right = 90.0f;
    if (s2_rot_right < 0.0f) s2_rot_right = 0.0f;
    if (s2_rot_right > 180.0f) s2_rot_right = 180.0f;
    
    uint16_t pulse_alpha_right = 1500 + OFFSET_RR + (int)(s0_rot_right * PULSE_PER_DEG);
    uint16_t pulse_beta_right = 1500 + OFFSET_RF + (int)(s2_rot_right * PULSE_PER_DEG);
    
    // 脉冲限幅
    if (pulse_alpha_left < 500) pulse_alpha_left = 500;
    if (pulse_alpha_left > 2500) pulse_alpha_left = 2500;
    if (pulse_beta_left < 500) pulse_beta_left = 500;
    if (pulse_beta_left > 2500) pulse_beta_left = 2500;
    if (pulse_alpha_right < 500) pulse_alpha_right = 500;
    if (pulse_alpha_right > 2500) pulse_alpha_right = 2500;
    if (pulse_beta_right < 500) pulse_beta_right = 500;
    if (pulse_beta_right > 2500) pulse_beta_right = 2500;
    
    // 控制舵机缓慢移动到目标位置
    duoji_move_to(2, pulse_alpha_left, duration_ms);   // 舵机ID2 - 左前腿（大腿）
    duoji_move_to(4, pulse_beta_left, duration_ms);     // 舵机ID4 - 左后腿（小腿）
    duoji_move_to(1, pulse_alpha_right, duration_ms);  // 舵机ID1 - 右前腿（大腿）
    duoji_move_to(3, pulse_beta_right, duration_ms);    // 舵机ID3 - 右后腿（小腿）
    
    // 等待舵机移动完成
    vTaskDelay(pdMS_TO_TICKS(duration_ms + 100));
    
        ESP_LOGI(TAG, "舵机移动完成: 角度 L(α=%.1f°,β=%.1f°) R(α=%.1f°,β=%.1f°) | 脉冲 L(%d,%d) R(%d,%d)", 
            alpha_left_deg, beta_left_deg, alpha_right_deg, beta_right_deg,
            pulse_alpha_left, pulse_beta_left, pulse_alpha_right, pulse_beta_right);

}


// ==================== 读取传感器数据 ====================
static void read_sensor_data(void)
{
    // 使用新的6050接口获取当前姿态数据
    float roll, pitch, yaw;
    mpu6050_get_data(&roll, &pitch, &yaw);
    
    // 更新当前俯仰角
    current_pitch = pitch;
    
    // 获取角速度数据
    float gx, gy, gz;
    mpu6050_get_gyro(&gx, &gy, &gz);
    current_gyro_y = gy;
}

// ==================== 读取轮子速度 ====================
static void read_wheel_velocity(void)
{
    // 使用编码器接口获取真实的轮子速度
    wheel_velocity_left = enc_get_speed(0);   // 左轮速度（编码器ID 0）
    wheel_velocity_right = enc_get_speed(1);  // 右轮速度（编码器ID 1）
    
    // 计算平均速度作为当前速度
    current_velocity = (wheel_velocity_left + wheel_velocity_right) / 2.0f;
    
    // 通过积分速度得到当前位置（简单积分）
    current_position += current_velocity * 0.01f; // 10ms控制周期
    
    // 调试信息（可选）
    static int64_t last_debug_time = 0;
    int64_t current_time = esp_timer_get_time();
    if (current_time - last_debug_time > 1000000) { // 每秒打印一次
        ESP_LOGD(TAG, "轮子速度 - 左: %.2f m/s, 右: %.2f m/s, 平均: %.2f m/s", 
                wheel_velocity_left, wheel_velocity_right, current_velocity);
        last_debug_time = current_time;
    }
}

// ==================== 腿部位置防倒机制 ====================
static void adjust_leg_position_for_balance(void)
{
    // 根据俯仰角计算腿部位置偏移（防倒机制）
    // 当机器人后倒时（pitch为正），腿部向后移动，重心相对前移
    // 当机器人前倾时（pitch为负），腿部向前移动，重心相对后移
    
    float pitch_control_gain = 2.0f;   // 俯仰角控制增益
    float max_leg_offset = 20.0f;      // 最大腿部偏移量
    float dead_zone = 1.0f;            // 死区范围（避免微小抖动）
    
    // 明确的pitch判断逻辑
    if (current_pitch > dead_zone) {
        // 机器人后倒（pitch > +1°）：腿部向后移动
        leg_position_offset = -fabs(current_pitch) * pitch_control_gain;
        ESP_LOGD(TAG, "后倒状态: pitch=%.1f°, 腿部向后移动: %.1fmm", current_pitch, leg_position_offset);
    } else if (current_pitch < -dead_zone) {
        // 机器人前倾（pitch < -1°）：腿部向前移动
        leg_position_offset = fabs(current_pitch) * pitch_control_gain;
        ESP_LOGD(TAG, "前倾状态: pitch=%.1f°, 腿部向前移动: %.1fmm", current_pitch, leg_position_offset);
    } else {
        // 直立状态（-1° < pitch < +1°）：保持当前位置
        leg_position_offset = 0.0f;
        ESP_LOGD(TAG, "直立状态: pitch=%.1f°, 腿部位置保持", current_pitch);
    }
    
    // 限幅
    if (leg_position_offset > max_leg_offset) leg_position_offset = max_leg_offset;
    if (leg_position_offset < -max_leg_offset) leg_position_offset = -max_leg_offset;
    
    // 应用位置环控制（如果启用了位置环）- 用于腿部位置调整
    if (current_pid.robot_kp > 0.0f) {
        
        float position_error = current_position -target_position;
        leg_position_offset += current_pid.robot_kp * leg_position_offset;
        
        // 调试信息
        static int64_t last_pos_debug_time = 0;
        int64_t current_time = esp_timer_get_time();
        if (current_time - last_pos_debug_time > 2000000) { // 每2秒打印一次
            ESP_LOGD(TAG, "位置环: 目标=%.2fm, 当前=%.2fm, 误差=%.2fm, 腿部偏移增量=%.2fmm", 
                    target_position, current_position, position_error, current_pid.robot_kp * position_error);
            last_pos_debug_time = current_time;
        }
    }
    
    // 计算新的腿部X位置
    float new_leg_x = leg_original_x + leg_position_offset;
    
    // 限制腿部位置范围
    if (new_leg_x < 0.0f) new_leg_x = 0.0f;
    if (new_leg_x > 100.0f) new_leg_x = 100.0f;
    
    // 如果腿部位置有显著变化，则更新腿部位置
    static float last_leg_x = STAND_FORWARD;
    if (fabs(new_leg_x - last_leg_x) > 0.5f) {
        set_leg_position_smooth(new_leg_x, leg_target_y,250);
        last_leg_x = new_leg_x;
        ESP_LOGI(TAG, "更新腿部位置: %.1fmm (偏移: %.1fmm)", new_leg_x, leg_position_offset);
    }
}

// ==================== 计算轮子力矩 ====================
static float calculate_wheel_torque(void)
{
    // 双环PID控制算法（第六课的核心思路）
    
    // 1. 速度环：计算目标俯仰角
    float velocity_error =  current_velocity -target_velocity ;
    float target_pitch_angle = current_pid.vel_kp * velocity_error;
    
    // 目标角度限幅（防止过度倾斜）
    if (target_pitch_angle > 10.0f) target_pitch_angle = 10.0f;
    if (target_pitch_angle < -10.0f) target_pitch_angle = -10.0f;
    
    // 3. 直立环：计算轮子力矩
    float angle_error = target_pitch_angle - current_pitch ;  // 角度误差
    float torque = current_pid.balance_kp * angle_error + current_pid.balance_kd * current_gyro_y;
    
    // 力矩限幅
    if (torque > max_torque) torque = max_torque;
    if (torque < -max_torque) torque = -max_torque;
    
    // 归一化到-1.0到1.0之间（motor_torque函数要求）
    float normalized_torque = torque * torque_normalization_factor;
    
    return normalized_torque;
}

// ==================== 蓝牙命令解析 ====================
static void parse_balance_command(const char *cmd, int len)
{
    ESP_LOGI(TAG, "收到命令: %.*s", len, cmd);
    
    // 启动命令
    if (strncmp(cmd, "start", 5) == 0) {
        robot_running = true;
        ESP_LOGI(TAG, "启动平衡控制");
        ble_send_response("平衡控制已启动", strlen("平衡控制已启动"));
        return;
    }
    
    // 停止命令
    if (strncmp(cmd, "stop", 4) == 0) {
        robot_running = false;
        motor_torque(0, 0);
        motor_torque(1, 0);
        ESP_LOGI(TAG, "停止平衡控制");
        ble_send_response("平衡控制已停止", strlen("平衡控制已停止"));
        return;
    }
    
    // 高度设置命令
    if (strncmp(cmd, "height:", 7) == 0) {
        float new_height = atof(cmd + 7);
        if (new_height >= 130.0f && new_height <= 300.0f) {
            leg_target_y = new_height;
            set_leg_position_smooth(leg_target_x, leg_target_y, 1500); // 1.5秒缓慢移动
            ESP_LOGI(TAG, "设置站立高度: %.1fmm", new_height);
            char response[64];
            snprintf(response, sizeof(response), "高度设置为: %.1fmm", new_height);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效高度: %.1fmm", new_height);
            ble_send_response("无效高度(130-300mm)", strlen("无效高度(130-300mm)"));
        }
        return;
    }
    
    // 前倾距离设置命令
    if (strncmp(cmd, "forward:", 8) == 0) {
        float new_forward = atof(cmd + 8);
        if (new_forward >= 0.0f && new_forward <= 100.0f) {
            leg_target_x = new_forward;
            set_leg_position_smooth(leg_target_x, leg_target_y, 1500); // 1.5秒缓慢移动
            ESP_LOGI(TAG, "设置前倾距离: %.1fmm", new_forward);
            char response[64];
            snprintf(response, sizeof(response), "前倾距离设置为: %.1fmm", new_forward);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效前倾距离: %.1fmm", new_forward);
            ble_send_response("无效前倾距离(0-100mm)", strlen("无效前倾距离(0-100mm)"));
        }
        return;
    }
    
    // 直立环Kp设置命令
    if (strncmp(cmd, "kp:", 3) == 0) {
        float new_kp = atof(cmd + 3);
        if (new_kp >= 0 && new_kp <= 1.0f) {
            current_pid.balance_kp = new_kp;
            ESP_LOGI(TAG, "设置直立环Kp: %.3f", current_pid.balance_kp);
            char response[64];
            snprintf(response, sizeof(response), "直立环Kp设置为: %.3f", current_pid.balance_kp);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效Kp: %.3f", new_kp);
            ble_send_response("无效Kp(-1.0~0.0)", strlen("无效Kp(-1.0~0.0)"));
        }
        return;
    }
    
    // 直立环Kd设置命令
    if (strncmp(cmd, "kd:", 3) == 0) {
        float new_kd = atof(cmd + 3);
        if (new_kd >= -0.10 && new_kd <= 0) {
            current_pid.balance_kd = new_kd;
            ESP_LOGI(TAG, "设置直立环Kd: %.3f", current_pid.balance_kd);
            char response[64];
            snprintf(response, sizeof(response), "直立环Kd设置为: %.3f", current_pid.balance_kd);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效Kd: %.3f", new_kd);
            ble_send_response("无效Kd(0.0~0.1)", strlen("无效Kd(0.0~0.1)"));
        }
        return;
    }
    
    // 速度环Kp设置命令
    if (strncmp(cmd, "velkp:", 6) == 0) {
        float new_velkp = atof(cmd + 6);
        if (new_velkp <= 0 && new_velkp >= -0.5f) {
            current_pid.vel_kp = new_velkp;
            ESP_LOGI(TAG, "设置速度环Kp: %.3f", current_pid.vel_kp);
            char response[64];
            snprintf(response, sizeof(response), "速度环Kp设置为: %.3f", current_pid.vel_kp);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效速度环Kp: %.3f", new_velkp);
            ble_send_response("无效速度环Kp(-0.5~0.0)", strlen("无效速度环Kp(-0.5~0.0)"));
        }
        return;
    }
    
    // 机器人位置Kp设置命令
    if (strncmp(cmd, "robotkp:", 8) == 0) {
        float new_robotkp = atof(cmd + 8);
        if (new_robotkp >= 0 && new_robotkp <= 0.5f) {
            current_pid.robot_kp = new_robotkp;
            ESP_LOGI(TAG, "设置机器人位置Kp: %.3f", current_pid.robot_kp);
            char response[64];
            snprintf(response, sizeof(response), "机器人位置Kp设置为: %.3f", current_pid.robot_kp);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效机器人位置Kp: %.3f", new_robotkp);
            ble_send_response("无效机器人位置Kp(0.0~0.5)", strlen("无效机器人位置Kp(0.0~0.5)"));
        }
        return;
    }
    
    // 重置位置命令（将当前位置设为原点）
    if (strncmp(cmd, "resetpos", 8) == 0) {
        current_position = 0.0f;
        leg_position_offset = 0.0f;
        set_leg_position_smooth(leg_original_x, leg_target_y,250);
        ESP_LOGI(TAG, "重置当前位置和腿部位置");
        ble_send_response("位置和腿部位置已重置", strlen("位置和腿部位置已重置"));
        return;
    }
    
    // 设置所有PID参数命令（方便调试）
    if (strncmp(cmd, "setpid:", 7) == 0) {
        float vel_kp, balance_kp, balance_kd, robot_kp;
        if (sscanf(cmd + 7, "%f,%f,%f,%f", &vel_kp, &balance_kp, &balance_kd, &robot_kp) == 4) {
            current_pid.vel_kp = vel_kp;
            current_pid.balance_kp = balance_kp;
            current_pid.balance_kd = balance_kd;
            current_pid.robot_kp = robot_kp;
            ESP_LOGI(TAG, "设置PID参数: vel_kp=%.3f, balance_kp=%.3f, balance_kd=%.3f, robot_kp=%.3f", 
                    current_pid.vel_kp, current_pid.balance_kp, current_pid.balance_kd, current_pid.robot_kp);
            char response[128];
            snprintf(response, sizeof(response), "PID参数设置成功: vel_kp=%.3f, balance_kp=%.3f, balance_kd=%.3f, robot_kp=%.3f", 
                    current_pid.vel_kp, current_pid.balance_kp, current_pid.balance_kd, current_pid.robot_kp);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效PID参数格式");
            ble_send_response("无效PID参数格式，使用: setpid:vel_kp,balance_kp,balance_kd,robot_kp", 
                            strlen("无效PID参数格式，使用: setpid:vel_kp,balance_kp,balance_kd,robot_kp"));
        }
        return;
    }
    
    // 速度设置命令
    if (strncmp(cmd, "speed:", 6) == 0) {
        float new_speed = atof(cmd + 6);
        if (new_speed >= -max_velocity && new_speed <= max_velocity) {
            target_velocity = new_speed;
            ESP_LOGI(TAG, "设置目标速度: %.2fm/s", target_velocity);
            char response[64];
            snprintf(response, sizeof(response), "速度设置为: %.2fm/s", target_velocity);
            ble_send_response(response, strlen(response));
        } else {
            ESP_LOGE(TAG, "无效速度: %.2fm/s", new_speed);
            char response[64];
            snprintf(response, sizeof(response), "无效速度(%.1f~%.1fm/s)", -max_velocity, max_velocity);
            ble_send_response(response, strlen(response));
        }
        return;
    }
    
    // 状态查询
    if (strncmp(cmd, "status", 6) == 0) {
        char status[256];
        snprintf(status, sizeof(status), 
                "平衡控制: %s\n高度: %.1fmm\n前倾: %.1fmm\n腿部偏移: %.1fmm\n目标速度: %.2fm/s\n当前速度: %.2fm/s\n当前位置: %.2fm\nPID参数:\n  速度环Kp: %.3f\n  直立环Kp: %.3f\n  直立环Kd: %.3f\n  位置环Kp: %.3f\n俯仰角: %.2f°", 
                robot_running ? "运行中" : "停止", 
                leg_target_y, leg_target_x, leg_position_offset, target_velocity, current_velocity, current_position,
                current_pid.vel_kp, current_pid.balance_kp, current_pid.balance_kd, current_pid.robot_kp,
                current_pitch);
        ble_send_response(status, strlen(status));
        return;
    }
    
    // 默认回显
    char echo[128];
    snprintf(echo, sizeof(echo), "收到: %.*s", len, cmd);
    ble_send_response(echo, strlen(echo));
}

// ==================== 蓝牙回调函数 ====================
/* NimBLE 主机同步回调 */
static void ble_host_sync_cb(void)
{
    ESP_LOGI(TAG, "NimBLE host synchronized");
    adv_init();
    ESP_LOGI(TAG, "BLE advertising started");
}

/* NimBLE 主机任务函数 */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    
    ble_hs_cfg.sync_cb = ble_host_sync_cb;
    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;
    
    ESP_LOGI(TAG, "NimBLE host configured");
    
    nimble_port_run();
    vTaskDelete(NULL);
}

/* 打印启动信息 */
static void print_startup_info(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "轮足机器人平衡控制系统");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "设备名称: 轮足机器人");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");
}