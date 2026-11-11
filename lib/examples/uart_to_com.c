#include <stdio.h>
#include "uart_to_com.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CUSTOM_EXAMPLE";

void app_main(void) {
    printf("Custom Task Example Starting...\n");
    ESP_LOGI(TAG, "Manually creating bridge tasks...");

    // 手动初始化UART（复用库中的配置）
    uart_config_t uart1_config = {
        .baud_rate = UART_TO_COM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_config_t uart2_config = uart1_config; // 两个串口配置相同

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_TO_COM_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart1_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TO_COM_UART1_TXD, UART_TO_COM_UART1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, UART_TO_COM_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart2_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_TO_COM_UART2_TXD, UART_TO_COM_UART2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 方式二：手动创建任务，实现更灵活的控制
    // 例如，您可以给任务不同的优先级，或者只创建单向任务
    xTaskCreate(task_process_uart1_to_uart2, "custom_u1_to_u2", 4096, NULL, 8, NULL); // 优先级略低
    xTaskCreate(task_process_uart2_to_uart1, "custom_u2_to_u1", 4096, NULL, 8, NULL);

    ESP_LOGI(TAG, "Custom tasks created.");
    
    // 主任务可以在这里做其他事情
    while(1) {
        ESP_LOGI(TAG, "Main task is running...");
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒打印一次
    }
}