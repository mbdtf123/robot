#ifndef DUOJI_CONTROL_H
#define DUOJI_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// ==================== 状态码定义 ====================
typedef enum {
    DUOJI_OK = 0,       // 操作成功 (发送成功)
    DUOJI_TIMEOUT,      // 通信超时
    DUOJI_ERROR,        // 通信错误或指令失败
    DUOJI_INVALID_ID,   // 无效的舵机ID
    DUOJI_PARSE_ERROR,  // 解析返回值失败
} duoji_status_t;

// ==================== 库的初始化函数 ====================
/**
 * @brief 初始化舵机控制库 (内部会初始化UART2)
 * @param baudrate 与舵机通信的波特率，默认为 115200
 * @return duoji_status_t 操作结果
 */
duoji_status_t duoji_init(uint32_t baudrate);

// ==================== 发送指令（无返回值或仅确认发送） ====================

/**
 * @brief 控制舵机移动到指定位置
 * @param id 舵机ID (0-254)
 * @param pulse_width 目标PWM值 (500-2500)
 * @param time_ms 运动时间 (0-9999 ms)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_move_to(uint8_t id, uint16_t pulse_width, uint16_t time_ms);

/**
 * @brief 释放舵机扭力
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_unlock(uint8_t id);

/**
 * @brief 恢复舵机扭力
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_lock(uint8_t id);

/**
 * @brief 设置舵机开机启动位置
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_set_boot_position(uint8_t id);

/**
 * @brief 设置舵机角度范围最小值
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_set_min_angle(uint8_t id);

/**
 * @brief 设置舵机角度范围最大值
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_set_max_angle(uint8_t id);

/**
 * @brief 半恢复出厂设置
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_reset_partial(uint8_t id);

/**
 * @brief 停止舵机运动
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_stop(uint8_t id);

/**
 * @brief 继续舵机运动
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_continue(uint8_t id);

// ==================== 发送并接收指令（有返回值） ====================

/**
 * @brief 查询舵机当前位置 (注意：此指令在当前硬件环境下可能无返回值，需验证)
 * @param id 舵机ID (0-254)
 * @param[out] position 指向用于存储返回位置值的变量的指针
 * @return duoji_status_t 操作结果
 */
duoji_status_t duoji_get_position(uint8_t id, uint16_t *position);

/**
 * @brief 查询指定ID的舵机是否存在
 * @param id 待查询的舵机ID (0-254)
 * @return bool 如果存在返回true，否则返回false
 */
bool duoji_id_exists(uint8_t id);

/**
 * @brief 修改舵机ID
 * @param old_id 原来的舵机ID (0-254)
 * @param new_id 新的舵机ID (0-254)
 * @return duoji_status_t 操作结果
 */
duoji_status_t duoji_change_id(uint8_t old_id, uint8_t new_id);

// ==================== 批量控制 ====================

/**
 * @brief 批量控制多个舵机移动 (使用括号格式)
 * @param commands_str 预先格式化好的批量指令字符串, 例如 "{G0000#001P1000T1000!#002P2000T1500!}"
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_move_batch(const char *commands_str);

/**
 * @brief 便捷的批量控制函数 (使用内部格式化)
 * @param num_servos 要控制的舵机数量 (建议不超过10个，避免缓冲区溢出)
 * @param ids 舵机ID数组 (例如 {1, 2})
 * @param pulse_widths 目标PWM值数组 (例如 {1000, 2000})
 * @param time_ms 运动时间数组 (例如 {1000, 1500})
 * @return duoji_status_t 操作结果 (仅确认指令发送成功)
 */
duoji_status_t duoji_move_batch_formatted(uint8_t num_servos, const uint8_t *ids, const uint16_t *pulse_widths, const uint16_t *time_ms);


/**
 * @brief 设置舵机工作模式为270度 顺时针方向
 * @param id 舵机ID
 * @return 操作状态
 */
duoji_status_t duoji_set_mode_270_cw(uint8_t id);

/**
 * @brief 设置舵机工作模式为270度 逆时针方向
 * @param id 舵机ID
 * @return 操作状态
 */
duoji_status_t duoji_set_mode_270_ccw(uint8_t id);


// 发送 PCSR 恢复初始值（上电自动回位）
duoji_status_t duoji_restore_initial(uint8_t id);

/**
 * @brief 将舵机当前位置设置为1500脉冲的中间值（零点校准）
 * @param id 舵机ID (0-254)
 * @return duoji_status_t 操作结果
 */
duoji_status_t duoji_set_midpoint(uint8_t id);




#ifdef __cplusplus
}
#endif

#endif // DUOJI_CONTROL_H