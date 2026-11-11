/**
 * @file uart_bridge_simple.c
 * @brief ESP32-S3 双串口透传桥接代码 (ESP-IDF 版本 - 修正版)
 *
 * 该程序在ESP32-S3的两个硬件串口之间创建一个透明的数据通道。
 * 数据从UART1输入将被转发到UART2，同时从UART2输入的数据也将被转发到UART1。
 * 主要用于连接电脑与总线舵机，实现指令透传。
 */

#include <stdio.h>      // 标准输入输出库
#include <string.h>     // 字符串操作库
#include "freertos/FreeRTOS.h"          // FreeRTOS核心头文件
#include "freertos/task.h"              // FreeRTOS任务管理头文件
#include "driver/uart.h"                // ESP-IDF UART驱动头文件
#include "driver/gpio.h"                // ESP-IDF GPIO驱动头文件
#include "sdkconfig.h"                  // ESP-IDF配置文件
#include "esp_log.h"                    // ESP-IDF日志库头文件

// ==================== 硬件引脚配置区域 ====================
// 定义UART1的TXD (发送) 和 RXD (接收) 引脚
// 此串口通常连接至电脑的USB转串口芯片
#define ECHO_UART1_TXD  (17)  // UART1发送引脚，连接到电脑
#define ECHO_UART1_RXD  (16)  // UART1接收引脚，连接到电脑

// 定义UART2的TXD (发送) 和 RXD (接收) 引脚
// 此串口通常连接至总线舵机
#define ECHO_UART2_TXD  (11)  // UART2发送引脚，连接到舵机 (Tx -> Rx)
#define ECHO_UART2_RXD  (12)  // UART2接收引脚，连接到舵机 (Rx <- Tx)

// ==================== UART参数配置 ====================
// 不再定义冗余宏，直接使用ESP-IDF提供的UART_NUM_1, UART_NUM_2
#define BAUD_RATE       (115200)          // 串口通信波特率，需与舵机波特率一致
#define BUF_SIZE        (256)             // 读写缓冲区大小，单位为字节

// 日志标签，用于在串口监视器中标识此程序的日志
static const char *TAG = "UART_BRIDGE";

// ==================== FreeRTOS任务声明 ====================

/**
 * @brief FreeRTOS任务：处理UART1到UART2的数据转发
 * 
 * 该任务会持续检查UART1的接收缓冲区，一旦发现数据，
 * 立即将其读取并写入UART2的发送缓冲区，实现U1->U2的数据透传。
 * 
 * @param pvParameters 任务参数，此任务未使用，传入NULL
 */
void task_process_uart1_to_uart2(void *pvParameters);

/**
 * @brief FreeRTOS任务：处理UART2到UART1的数据转发
 * 
 * 该任务会持续检查UART2的接收缓冲区，一旦发现数据，
 * 立即将其读取并写入UART1的发送缓冲区，实现U2->U1的数据透传。
 * 
 * @param pvParameters 任务参数，此任务未使用，传入NULL
 */
void task_process_uart2_to_uart1(void *pvParameters);

// ==================== 主函数 (app_main) ====================

/**
 * @brief 程序入口点
 * 
 * 1. 配置并初始化UART1和UART2。
 * 2. 创建两个FreeRTOS任务，分别负责双向数据流。
 * 3. 本任务自身完成后删除自己。
 */
void app_main(void) {
    // --- 1. 配置并安装 UART1 ---
    uart_config_t uart1_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart1_config));
    // 统一使用宏进行引脚配置
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, ECHO_UART1_TXD, ECHO_UART1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // --- 2. 配置并安装 UART2 ---
    uart_config_t uart2_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart2_config));
    // 统一使用宏进行引脚配置
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, ECHO_UART2_TXD, ECHO_UART2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // --- 3. 创建FreeRTOS任务 ---
    xTaskCreate(task_process_uart1_to_uart2, "u1_to_u2", 4096, NULL, 10, NULL);
    xTaskCreate(task_process_uart2_to_uart1, "u2_to_u1", 4096, NULL, 10, NULL);

    // --- 4. 删除本任务 ---
    vTaskDelete(NULL);
}

// ==================== 任务函数实现 ====================

void task_process_uart1_to_uart2(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    int len;

    while(1) {
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, pdMS_TO_TICKS(10));

        if (len > 0) {
            uart_write_bytes(UART_NUM_2, (const char*)data, len);
            uart_flush(UART_NUM_2);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void task_process_uart2_to_uart1(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    int len;

    while(1) {
        len = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, pdMS_TO_TICKS(10));

        if (len > 0) {
            uart_write_bytes(UART_NUM_1, (const char*)data, len);
            uart_flush(UART_NUM_1);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}