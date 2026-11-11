#ifndef ESPFOC__HW_LINK_H_
#define ESPFOC__HW_LINK_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// ================================================================
// 编码器 (AS5600)
// ================================================================
bool  enc_init(int encoder_id, int sda, int scl);   // 初始化, 入参: SDA/SCL
float enc_read_raw(int encoder_id);                  // 读角度 (rad, 直接发起I2C), 失败返回-1
void  enc_update(int encoder_id);                    // 更新+计算速度, 内部缓存角度
float enc_get_cached_angle(int encoder_id);          // 获取enc_update缓存的角度 (无I2C)
float enc_get_speed(int encoder_id);                 // 获取上次更新算出的速度

// ================================================================
// FOC 电机控制
// ================================================================

/**
 * @brief 初始化FOC电机系统 (PWM + 状态机 + PID)
 *
 * 内部自动初始化两个电机的PWM引脚和PID控制器。
 * 引脚定义来自 foc_config.h 中的 MOTOR_PHASE_A/B/C_GPIO_0/1
 */
void motor_foc_init(void);

/**
 * @brief 力矩环控制
 * @param id     电机编号 (0 或 1)
 * @param torque 目标力矩, 归一化 [-1, 1], 正=顺时针
 *
 * 最底层控制, 无PID, 直接转换为Uq电压输入SVPWM/FOC变换
 */
void motor_torque(int id, float torque);

/**
 * @brief 速度环控制
 * @param id           电机编号 (0 或 1)
 * @param target_speed 目标速度 (rad/s), 正=顺时针, 负=逆时针
 *
 * 内部PID: 误差 = target - current → 输出Uq(V) → SVPWM变换驱动
 * 电角度使用编码器闭环(非开环推演), 防止发热和卡死
 * PID带条件积分(Anti-windup Clamping)和积分分离带
 */
void motor_speed(int id, float target_speed);

/**
 * @brief 位置维持控制
 * @param id 电机编号 (0 或 1)
 *
 * 调用时读取当前编码器角度作为目标, 之后持续锁定在该位置。
 * 使用编码器闭环电角度 + SVPWM + 速度前馈阻尼。
 * PID带条件积分(Anti-windup Clamping)和积分分离带
 */
void motor_position_hold(int id);

/**
 * @brief 单电机FOC控制循环 (每1ms调用一次)
 * @param id 电机编号
 *
 * 内部完成:
 *   1. 编码器单次I2C读取 (无printf, 无冗余读)
 *   2. 连续角度累积 + 闭环电角度计算
 *   3. 根据模式计算Uq电压 (PID/直接力矩)
 *   4. SVPWM空间矢量调制 → 三相PWM输出
 */
void motor_update(int id);

#endif // ESPFOC__HW_LINK_H_
