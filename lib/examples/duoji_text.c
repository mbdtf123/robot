#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "duoji_control.h"

static const char *TAG = "CENTER_TEST";

#define ID_LR  2  // 舵机2
#define ID_LF  0  // 舵机0

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
    ESP_LOGI(TAG, "Center Position Test (1500 pulse)");
    
    debug_uart_init();
    duoji_init(115200);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 设置模式
    duoji_set_mode_270_cw(ID_LF);   // 舵机0 顺时针
    vTaskDelay(pdMS_TO_TICKS(100));
    duoji_set_mode_270_ccw(ID_LR);  // 舵机2 逆时针
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Moving both servos to 1500 pulse (center position)...");
    
    // 两个舵机都移动到 1500 脉冲
    uint8_t ids[2] = {ID_LR, ID_LF};
    uint16_t pulses[2] = {1500, 1500};
    uint16_t times[2] = {1000, 1000};
    
    duoji_move_batch_formatted(2, ids, pulses, times);
    
    ESP_LOGI(TAG, "Servos should now be at center position");
    ESP_LOGI(TAG, "Observe the mechanical position of both servos");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}