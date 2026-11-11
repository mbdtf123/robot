#include <duoji_control.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

// 舵机0 和 舵机2
uint8_t  my_ids[]       = {2, 4};      // LR, LF
uint16_t my_times[]     = {1000, 1000};

void control_servos_task(void *arg)
{
    // 中心脉冲（假设已经校准为 1500）
    int center_lf = 1500;
    int center_lr = 1500;
    
    // 40° 对应的脉冲增量：40 × 7.5 = 300
    int delta = 500;
    
    // 目标位置：中心 + 300（正方向转 40°）
    int target_lf = center_lf + delta;  // 1800
    int target_lr = center_lr + delta;  // 1800
    
    ESP_LOGI(TAG, "Moving both servos +40° (center=1500, target=%d)", target_lf);
    
    uint16_t pos_target[] = {target_lr, target_lf};
    duoji_move_batch_formatted(2, my_ids, pos_target, my_times);
    
    ESP_LOGI(TAG, "Done. Servos should have moved +40°.");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    duoji_init(115200);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(control_servos_task, "servo_loop", 4096, NULL, 10, NULL);
}