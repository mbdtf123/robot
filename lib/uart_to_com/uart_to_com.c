#include "uart_to_com.h" // 包含自己的头文件
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "UART_TO_COM";

// ==================== 库函数实现 ====================

void uart_to_com_start_bridge(void) {
    ESP_LOGI(TAG, "Initializing UART Bridge...");

    // --- 1. 配置并安装 UART1 ---
    uart_config_t uart1_config = {
        .baud_rate = UART_TO_COM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_TO_COM_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart1_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TO_COM_UART1_TXD, UART_TO_COM_UART1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // --- 2. 配置并安装 UART2 ---
    uart_config_t uart2_config = {
        .baud_rate = UART_TO_COM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, UART_TO_COM_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart2_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_TO_COM_UART2_TXD, UART_TO_COM_UART2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // --- 3. 创建FreeRTOS任务 ---
    xTaskCreate(task_process_uart1_to_uart2, "u1_to_u2", 4096, NULL, 10, NULL);
    xTaskCreate(task_process_uart2_to_uart1, "u2_to_u1", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "UART Bridge started successfully.");
}

// ==================== 公共任务函数实现 ====================

void task_process_uart1_to_uart2(void *pvParameters) {
    uint8_t data[UART_TO_COM_BUF_SIZE];
    int len;

    while(1) {
        len = uart_read_bytes(UART_NUM_1, data, UART_TO_COM_BUF_SIZE, pdMS_TO_TICKS(10));

        if (len > 0) {
            uart_write_bytes(UART_NUM_2, (const char*)data, len);
            uart_flush(UART_NUM_2);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void task_process_uart2_to_uart1(void *pvParameters) {
    uint8_t data[UART_TO_COM_BUF_SIZE];
    int len;

    while(1) {
        len = uart_read_bytes(UART_NUM_2, data, UART_TO_COM_BUF_SIZE, pdMS_TO_TICKS(10));

        if (len > 0) {
            uart_write_bytes(UART_NUM_1, (const char*)data, len);
            uart_flush(UART_NUM_1);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}