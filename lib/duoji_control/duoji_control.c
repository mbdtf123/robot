#include "duoji_control.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// 使用我们之前创建的uart_to_com库
#include "uart_to_com.h"

static const char *TAG = "DUOJI_CONTROL";

// 为舵机通信专用的缓冲区大小
#define CMD_BUF_SIZE 256 // 增大以容纳批量指令
#define REPLY_BUF_SIZE 64

// 串口通信的超时时间 (单位: 毫秒)
#define UART_TIMEOUT_MS 1000

// ==================== 内部辅助函数 ====================

/**
 * @brief 内部封装的指令格式化函数
 */
static bool format_duoji_cmd(char *buffer, size_t buf_size, const char *format, ...) {
    va_list args;
    va_start(args, format);
    int result = vsnprintf(buffer, buf_size, format, args);
    va_end(args);
    return (result >= 0 && result < (int)buf_size);
}

/**
 * @brief 格式化批量指令
 */
static bool format_duoji_batch_cmd(char *buffer, size_t buf_size, uint8_t num_servos, const uint8_t *ids, const uint16_t *pulse_widths, const uint16_t *time_ms) {
    if (num_servos == 0 || !ids || !pulse_widths || !time_ms) return false;
    if (num_servos > 20) return false; // 限制数量，防止缓冲区溢出

    int offset = snprintf(buffer, buf_size, "{G0000"); // 开始部分
    if (offset < 0 || offset >= (int)buf_size) return false;

    for (int i = 0; i < num_servos; i++) {
        int written = snprintf(buffer + offset, buf_size - offset, "#%03dP%04dT%04d!", ids[i], pulse_widths[i], time_ms[i]);
        if (written < 0 || written >= (int)(buf_size - offset)) {
            return false; // 缓冲区不够
        }
        offset += written;
    }

    // 检查是否有空间添加结束符
    if (offset >= (int)buf_size - 1) return false;
    strcat(buffer, "}"); // 结束部分
    return true;
}


/**
 * @brief 格式化发送指令到UART2
 */
static duoji_status_t send_command(const char *cmd) {
    if (!cmd) return DUOJI_ERROR;
    int len = strlen(cmd);
    int sent = uart_write_bytes(UART_NUM_2, cmd, len);
    uart_flush(UART_NUM_2); // 确保发送完成
    return (sent == len) ? DUOJI_OK : DUOJI_ERROR;
}

/**
 * @brief 从UART2读取响应
 */
static duoji_status_t read_reply(char *buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) return DUOJI_ERROR;
    int len = uart_read_bytes(UART_NUM_2, (uint8_t*)buffer, buffer_size - 1, pdMS_TO_TICKS(UART_TIMEOUT_MS));
    if (len > 0) {
        buffer[len] = '\0'; // 添加字符串结束符
        return DUOJI_OK;
    } else {
        return DUOJI_TIMEOUT;
    }
}

/**
 * @brief 检查舵机ID是否有效
 */
static inline bool is_valid_id(uint8_t id) {
    return id <= 254;
}

// ==================== 初始化 ====================

duoji_status_t duoji_init(uint32_t baudrate) {
    ESP_LOGI(TAG, "Initializing DuoJi Control Library with baudrate: %d", baudrate);

    // 配置UART2用于舵机通信
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 重新安装UART2，因为uart_to_com可能已经安装过
    uart_driver_delete(UART_NUM_2);
    esp_err_t err = uart_driver_install(UART_NUM_2, UART_TO_COM_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver for DuoJi: %s", esp_err_to_name(err));
        return DUOJI_ERROR;
    }
    err = uart_param_config(UART_NUM_2, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters for DuoJi: %s", esp_err_to_name(err));
        return DUOJI_ERROR;
    }
    err = uart_set_pin(UART_NUM_2, UART_TO_COM_UART2_TXD, UART_TO_COM_UART2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins for DuoJi: %s", esp_err_to_name(err));
        return DUOJI_ERROR;
    }

    ESP_LOGI(TAG, "DuoJi Control Library initialized successfully.");
    return DUOJI_OK;
}


// ==================== 发送指令函数实现 (更新：只确认发送) ====================

duoji_status_t duoji_move_to(uint8_t id, uint16_t pulse_width, uint16_t time_ms) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    if (pulse_width < 500 || pulse_width > 2500) return DUOJI_ERROR;
    if (time_ms > 9999) return DUOJI_ERROR;

    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dP%04dT%04d!", id, pulse_width, time_ms)) {
        return DUOJI_ERROR; // 格式化失败
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_unlock(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPULK!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_lock(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPULR!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_set_boot_position(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPCSD!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_set_min_angle(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPSMI!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_set_max_angle(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPSMX!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_reset_partial(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPCLEO!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_stop(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPDST!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

duoji_status_t duoji_continue(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPDCT!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

// ==================== 发送并接收指令函数实现 (保留) ====================

// 注意：根据您的描述，#000PRAD! 指令也可能无法返回值，但此处仍保留其逻辑，
// 以便在硬件支持时可以使用。调用者需自行判断返回值的意义。
duoji_status_t duoji_get_position(uint8_t id, uint16_t *position) {
    if (!is_valid_id(id) || position == NULL) return DUOJI_INVALID_ID;

    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPRAD!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);

    duoji_status_t ret = send_command(cmd);
    if (ret != DUOJI_OK) return ret;

    char reply[REPLY_BUF_SIZE];
    ret = read_reply(reply, sizeof(reply));
    if (ret != DUOJI_OK) {
        ESP_LOGW(TAG, "No reply received for get_position from ID %03d. Hardware may not support this.", id);
        // 根据实际情况，您可以选择返回超时，或者将position置为0并返回OK
        // 这里我们返回超时，让用户知道没有收到回复
        return ret;
    }

    // 解析返回值: "#001P1500!"
    if (reply[0] != '#' || reply[4] != 'P' || reply[strlen(reply)-1] != '!') {
        ESP_LOGE(TAG, "Malformed reply received: %s", reply);
        return DUOJI_PARSE_ERROR;
    }

    char pwm_str[5];
    strncpy(pwm_str, reply + 5, 4);
    pwm_str[4] = '\0';
    int parsed_pwm = atoi(pwm_str);

    if (parsed_pwm >= 500 && parsed_pwm <= 2500) {
        *position = (uint16_t)parsed_pwm;
        ESP_LOGD(TAG, "Received position for ID %03d: %d", id, *position);
        return DUOJI_OK;
    } else {
        ESP_LOGE(TAG, "Parsed invalid PWM value: %d", parsed_pwm);
        return DUOJI_PARSE_ERROR;
    }
}

bool duoji_id_exists(uint8_t id) {
    if (!is_valid_id(id)) return false;

    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPID!", id)) {
         // 如果格式化失败，肯定不存在
         return false;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);

    // 清空可能存在的旧数据 (不需要 dummy 缓冲区)
    uart_flush_input(UART_NUM_2);
    vTaskDelay(pdMS_TO_TICKS(10)); // 延迟一小会儿确保清空完成

    send_command(cmd);

    char reply[REPLY_BUF_SIZE];
    duoji_status_t ret = read_reply(reply, sizeof(reply));

    // 如果收到回复 "#001P!"，则认为存在
    if (ret == DUOJI_OK) {
        char expected_reply[10];
        snprintf(expected_reply, sizeof(expected_reply), "#%03dP!", id);
        if (strcmp(reply, expected_reply) == 0) {
            ESP_LOGD(TAG, "ID %03d exists.", id);
            return true;
        }
    }
    // ESP_LOGD(TAG, "ID %03d does not exist or no response.", id);
    return false;
}

duoji_status_t duoji_change_id(uint8_t old_id, uint8_t new_id) {
    if (!is_valid_id(old_id) || !is_valid_id(new_id)) return DUOJI_INVALID_ID;
    if (old_id == new_id) return DUOJI_OK; // ID未改变

    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPID%03d!", old_id, new_id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);

    duoji_status_t ret = send_command(cmd);
    if (ret != DUOJI_OK) return ret;

    // 读取返回值以确认修改成功
    char reply[REPLY_BUF_SIZE];
    ret = read_reply(reply, sizeof(reply));
    if (ret != DUOJI_OK) return ret;

    char expected_reply[10];
    snprintf(expected_reply, sizeof(expected_reply), "#%03dP!", new_id);
    if (strcmp(reply, expected_reply) == 0) {
        ESP_LOGI(TAG, "ID changed from %03d to %03d successfully.", old_id, new_id);
        return DUOJI_OK;
    } else {
        return DUOJI_ERROR;
    }
}

// ==================== 批量控制函数实现 (更新：只确认发送) ====================

duoji_status_t duoji_move_batch(const char *commands_str) {
    if (!commands_str) return DUOJI_ERROR;
    if (strlen(commands_str) > CMD_BUF_SIZE - 1) return DUOJI_ERROR; // 检查长度
    ESP_LOGD(TAG, "Sending batch command: %s", commands_str);
    return send_command(commands_str); // 只返回发送结果
}

duoji_status_t duoji_move_batch_formatted(uint8_t num_servos, const uint8_t *ids, const uint16_t *pulse_widths, const uint16_t *time_ms) {
    if (num_servos == 0 || !ids || !pulse_widths || !time_ms) return DUOJI_ERROR;

    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_batch_cmd(cmd, sizeof(cmd), num_servos, ids, pulse_widths, time_ms)) {
        ESP_LOGE(TAG, "Failed to format batch command, possibly due to buffer overflow or invalid input.");
        return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending formatted batch command: %s", cmd);
    return send_command(cmd); // 只返回发送结果
}

// ==================== 新增：设置舵机模式 ====================
duoji_status_t duoji_set_mode_270_cw(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    // 模式1：270度 顺时针
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPMOD1!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending mode command: %s", cmd);
    return send_command(cmd);
}

duoji_status_t duoji_set_mode_270_ccw(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    // 模式2：270度 逆时针
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPMOD2!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending mode command: %s", cmd);
    return send_command(cmd);
}

duoji_status_t duoji_restore_initial(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPCSR!", id)) {
         return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending command: %s", cmd);
    return send_command(cmd);
}

duoji_status_t duoji_set_midpoint(uint8_t id) {
    if (!is_valid_id(id)) return DUOJI_INVALID_ID;
    
    char cmd[CMD_BUF_SIZE];
    if (!format_duoji_cmd(cmd, sizeof(cmd), "#%03dPSCK!", id)) {
        return DUOJI_ERROR;
    }
    ESP_LOGD(TAG, "Sending PSCK command: %s", cmd);
    return send_command(cmd);
}