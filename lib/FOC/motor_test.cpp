/**
 * @file main.cpp
 * @brief FOC电机控制测试程序
 *
 * 三种控制模式:
 *   motor_torque(id, torque)     - 力矩环: torque∈[-1,1], 正=顺时针
 *   motor_speed(id, target_speed) - 速度环:  rad/s, 正=CW
 *   motor_position_hold(id)       - 位置环:  锁定当前位置
 */

#include "esp_hw_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

extern "C" void app_main(void) {
    // 初始化编码器 (SDA, SCL) - 与 foc_config.h 定义一致
    enc_init(0, 7, 15);   // M0: SDA=GPIO7, SCL=GPIO15

    // 初始化FOC电机控制系统
    motor_foc_init();

    // ===== 测试区域: 选择模式和参数 =====

    // ★ 先用最简单的力矩模式测试电机能否转动
    //motor_torque(0, 0.5f);    // 50%正转矩

    // 示例2: 速度模式 (20 rad/s)
    motor_speed(0, 20.0f);

    // 示例1: 位置维持
    //motor_position_hold(0);

    // ===== 控制循环 =====
    int dbg_cnt = 0;
    while (1) {
        motor_update(0);   // ★ 只更新M0 (当前只有一台电机接线)

        // 调试打印 (每5秒)
        if (++dbg_cnt >= 5000) {
            dbg_cnt = 0;
            printf("[DBG] M0 raw=%.3f spd=%.2f\n",
                   enc_read_raw(0), enc_get_speed(0));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}