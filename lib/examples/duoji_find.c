#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "duoji_control.h"

static const char *TAG = "QUERY_ID";

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

void app_main(void)
{
    debug_uart_init();
    duoji_init(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "=== Scanning IDs 0-10 at 115200 ===");
    
    int found_count = 0;
    for (int id = 0; id <= 10; id++) {
        if (duoji_id_exists(id)) {
            ESP_LOGI(TAG, "✓ ID %d EXISTS", id);
            found_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    
    if (found_count == 0) {
        ESP_LOGE(TAG, "No servo found at 115200!");
        ESP_LOGI(TAG, "Check: 1) Only ONE servo connected  2) Power  3) Wiring");
    } else {
        ESP_LOGI(TAG, "Found %d servo(s).", found_count);
    }

    ESP_LOGI(TAG, "=== Scan complete ===");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}