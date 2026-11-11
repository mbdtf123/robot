#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "duoji_control.h"
#include "kms.h"

static const char *TAG = "BIPEDAL";

// ==================== 舵机ID ====================
#define ID_LR  2  // 左右 Beta
#define ID_LF  0  // 左左 Alpha

// ==================== 舵机参数 ====================
#define MOVE_TIME         50      // 舵机运动时间（毫秒）
#define STEP_DELAY        30      // 每步延时（毫秒）

// ==================== 可调偏差值（修改这里！）====================
#define OFFSET_LF   0     // 舵机0 偏差值（脉冲）
#define OFFSET_LR   -20     // 舵机2 偏差值（脉冲）
#define PULSE_PER_DEG    7.5f  // 每度脉冲数

// ==================== 正弦波参数 ====================
static float g_offset = 120.0f;   // Y轴偏移
static float g_amp = 15.0f;       // 振幅
static float g_period = 128.0f;   // 周期

// 站立位置
static float g_stand_x = 50.0f;
static float g_stand_y = 120.0f;

// ==================== 全局变量 ====================
static JointAngles g_joint_angles;
static Node g_left_target;
static Node g_right_target;

// ==================== 角度归一化 ====================
static float normalize_angle(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg > 180.0f) angle_deg -= 360.0f;
    else if (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

// ==================== 舵机0 脉冲转换 ====================
static uint16_t rotation_to_pulse_lf(float rot_deg)
{
    if (rot_deg < -180.0f) rot_deg = -180.0f;
    if (rot_deg > 180.0f) rot_deg = 180.0f;
    int pulse = 1500 + OFFSET_LF + (int)(rot_deg * PULSE_PER_DEG);
    if (pulse < 500) pulse = 500;
    if (pulse > 2500) pulse = 2500;
    return (uint16_t)pulse;
}

// ==================== 舵机2 脉冲转换 ====================
static uint16_t rotation_to_pulse_lr(float rot_deg)
{
    if (rot_deg < -180.0f) rot_deg = -180.0f;
    if (rot_deg > 180.0f) rot_deg = 180.0f;
    int pulse = 1500 + OFFSET_LR + (int)(rot_deg * PULSE_PER_DEG);
    if (pulse < 500) pulse = 500;
    if (pulse > 2500) pulse = 2500;
    return (uint16_t)pulse;
}

// ==================== 左腿舵机控制 ====================
static void left_leg_control(void)
{
    float alpha_deg = g_joint_angles.alpha_left * 180.0f / PI;
    float beta_deg = g_joint_angles.beta_left * 180.0f / PI;
    
    alpha_deg = normalize_angle(alpha_deg);
    beta_deg = normalize_angle(beta_deg);
    
    if (alpha_deg < 80.0f) alpha_deg = 80.0f;
    if (alpha_deg > 160.0f) alpha_deg = 160.0f;
    if (beta_deg < -10.0f) beta_deg = -10.0f;
    if (beta_deg > 80.0f) beta_deg = 80.0f;
    
    float s0_rot = alpha_deg - 90.0f;
    if (s0_rot < 0.0f) s0_rot = 0.0f;
    if (s0_rot > 90.0f) s0_rot = 90.0f;
    uint16_t p_lf = rotation_to_pulse_lf(s0_rot);
    
    float s2_rot = 90.0f - beta_deg;
    if (s2_rot < 0.0f) s2_rot = 0.0f;
    if (s2_rot > 180.0f) s2_rot = 180.0f;
    uint16_t p_lr = rotation_to_pulse_lr(s2_rot);
    
    uint8_t ids[2] = {ID_LR, ID_LF};
    uint16_t pulses[2] = {p_lr, p_lf};
    uint16_t times[2] = {MOVE_TIME, MOVE_TIME};
    
    duoji_move_batch_formatted(2, ids, pulses, times);
    
    ESP_LOGI(TAG, "Alpha=%.1f° Beta=%.1f° -> LF=%d LR=%d", alpha_deg, beta_deg, p_lf, p_lr);
}

// ==================== 调试串口初始化 ====================
static void debug_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

// ==================== 主函数 ====================
void app_main(void)
{
    ESP_LOGI(TAG, "Bipedal Robot Starting...");
    ESP_LOGI(TAG, "Offsets: LF=%d, LR=%d", OFFSET_LF, OFFSET_LR);
    
    debug_uart_init();
    duoji_init(115200);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    g_right_target.x = g_stand_x;
    g_right_target.y = g_stand_y;
    
    ESP_LOGI(TAG, "Ready!");
    
    while (1) {
        for (int i = 0; i < 100; i++) {
            g_left_target.x = i;
            g_left_target.y = g_offset + g_amp * sinf((2 * PI / g_period) * i);
            inverse_kinematics(&g_left_target, &g_right_target, &g_joint_angles);
            left_leg_control();
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
        }
        
        for (int i = 99; i >= 0; i--) {
            g_left_target.x = i;
            g_left_target.y = g_offset + g_amp * sinf((2 * PI / g_period) * i);
            inverse_kinematics(&g_left_target, &g_right_target, &g_joint_angles);
            left_leg_control();
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
        }
    }
}