#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "duoji_control.h"

static const char *TAG = "CHANGE_ID";

// 调试串口初始化
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

    // ====== 修改这里的ID ======
    uint8_t old_id = 0;   // 当前舵机的ID
    uint8_t new_id = 1;   // 要改成的新ID

    ESP_LOGI(TAG, "Changing ID from %d to %d...", old_id, new_id);
    
    duoji_status_t ret = duoji_change_id(old_id, new_id);
    
    if (ret == DUOJI_OK) {
        ESP_LOGI(TAG, "✓ Success! ID changed to %d", new_id);
    } else {
        ESP_LOGE(TAG, "✗ Failed! Error code: %d", ret);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}