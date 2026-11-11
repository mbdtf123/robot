#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include <stdarg.h>
// 包含我们自己的库
#include "duoji_control.h"
#include "uart_to_com.h" // 用于配置GPIO引脚

static const char *TAG_MAIN = "MAIN";
static const char *TAG_SELF_TEST = "SELF_TEST";

// UART1 的配置常量
#define EX_UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
// extern "C" void app_main(void);
// 用于重定向日志的全局变量
static uart_port_t s_uart_num = UART_NUM_1;

/**
 * @brief 自定义的日志输出函数，将日志发送到指定的UART
 *        签名必须匹配 esp_log_set_vprintf 要求: int (*)(const char *, va_list)
 */
static int log_output_function(const char *fmt, va_list ap) {
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (len <= 0) {
        return -1;
    }
    if (s_uart_num == UART_NUM_1 || s_uart_num == UART_NUM_2) {
        int written = uart_write_bytes(s_uart_num, buf, (len < sizeof(buf)) ? len : sizeof(buf));
        return (written > 0) ? written : -1;
    }
    return -1;
}

/**
 * @brief 任务：执行舵机自检程序
 */
static void self_test_task(void *pvParameters) {
    ESP_LOGI(TAG_SELF_TEST, "Starting Self-Test...");

    // 1. 初始化舵机库 (使用115200波特率)
    duoji_status_t init_ret = duoji_init(115200);
    if (init_ret != DUOJI_OK) {
        ESP_LOGE(TAG_SELF_TEST, "Failed to initialize DuoJi library. Error code: %d", init_ret);
        vTaskDelete(NULL); // 初始化失败，删除任务
        return;
    }
    ESP_LOGI(TAG_SELF_TEST, "DuoJi library initialized successfully.");

    // 2. 测试：查找ID为2的舵机是否存在
    ESP_LOGI(TAG_SELF_TEST, "Testing ID existence for ID 2...");
    bool id2_exists = duoji_id_exists(2);
    if (id2_exists) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: ID 2 exists.");
    } else {
        ESP_LOGW(TAG_SELF_TEST, "WARNING: ID 2 does not exist or did not respond.");
        // 如果ID2不存在，可以尝试其他ID，比如0
        bool id0_exists = duoji_id_exists(0);
        if (id0_exists) {
             ESP_LOGI(TAG_SELF_TEST, "INFO: Found ID 0 instead.");
        }
    }

    // 3. 测试：控制舵机移动
    ESP_LOGI(TAG_SELF_TEST, "Testing move command for ID 2...");
    duoji_status_t move_ret = duoji_move_to(2, 1500, 1000); // 移动到1500us，耗时1秒
    if (move_ret == DUOJI_OK) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: Move command for ID 2 sent successfully.");
    } else {
        ESP_LOGW(TAG_SELF_TEST, "FAILED: Move command for ID 2 failed. Error code: %d", move_ret);
    }
    vTaskDelay(pdMS_TO_TICKS(1500)); // 等待舵机动作完成

    // 再次移动到另一个位置
    ESP_LOGI(TAG_SELF_TEST, "Testing move command for ID 2 (second move)...");
    move_ret = duoji_move_to(2, 2000, 1000); // 移动到2000us，耗时1秒
    if (move_ret == DUOJI_OK) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: Second move command for ID 2 sent successfully.");
    } else {
        ESP_LOGW(TAG_SELF_TEST, "FAILED: Second move command for ID 2 failed. Error code: %d", move_ret);
    }
    vTaskDelay(pdMS_TO_TICKS(1500)); // 等待舵机动作完成

    // 4. 测试：解锁和上锁
    ESP_LOGI(TAG_SELF_TEST, "Testing unlock command for ID 2...");
    duoji_status_t unlock_ret = duoji_unlock(2);
    if (unlock_ret == DUOJI_OK) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: Unlock command for ID 2 sent successfully.");
    } else {
        ESP_LOGW(TAG_SELF_TEST, "FAILED: Unlock command for ID 2 failed. Error code: %d", unlock_ret);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG_SELF_TEST, "Testing lock command for ID 2...");
    duoji_status_t lock_ret = duoji_lock(2);
    if (lock_ret == DUOJI_OK) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: Lock command for ID 2 sent successfully.");
    } else {
        ESP_LOGW(TAG_SELF_TEST, "FAILED: Lock command for ID 2 failed. Error code: %d", lock_ret);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 5. 测试：尝试获取位置 (注意：此指令可能无返回)
    ESP_LOGI(TAG_SELF_TEST, "Testing get_position command for ID 2...");
    uint16_t position = 0;
    duoji_status_t get_pos_ret = duoji_get_position(2, &position);
    if (get_pos_ret == DUOJI_OK) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: Position of ID 2 is %d us.", position);
    } else {
        ESP_LOGW(TAG_SELF_TEST, "INFO: Get position command for ID 2 returned: %d. This is expected if the hardware doesn't reply.", get_pos_ret);
    }

    // 6. 测试：批量控制
    ESP_LOGI(TAG_SELF_TEST, "Testing batch move command for ID 2...");
    uint8_t ids[] = {2}; // 改为ID 2
    uint16_t positions[] = {1000};
    uint16_t times[] = {1000};
    duoji_status_t batch_ret = duoji_move_batch_formatted(1, ids, positions, times); // 第一个参数是数组长度，仍为1
    if (batch_ret == DUOJI_OK) {
        ESP_LOGI(TAG_SELF_TEST, "SUCCESS: Batch move command for ID 2 sent successfully.");
    } else {
        ESP_LOGW(TAG_SELF_TEST, "FAILED: Batch move command for ID 2 failed. Error code: %d", batch_ret);
    }
    vTaskDelay(pdMS_TO_TICKS(1500)); // 等待舵机动作完成

    ESP_LOGI(TAG_SELF_TEST, "Self-Test completed.");
    vTaskDelete(NULL); // 自检完成，删除任务
}

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "Starting DuoJi Self-Test Application with ID 2...");

    // 1. 配置UART1用于打印和接收数据
    uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // 安装UART1驱动，但不创建队列，因为我们只用来输出
    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart1_config));
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_TO_COM_UART1_TXD, UART_TO_COM_UART1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 2. 重定向日志到UART1
    esp_log_set_vprintf(log_output_function);

    // 3. 启动舵机自检任务
    xTaskCreate(self_test_task, "self_test_task", 4096, NULL, configMAX_PRIORITIES-5, NULL);

    // 主任务进入低优先级循环
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}