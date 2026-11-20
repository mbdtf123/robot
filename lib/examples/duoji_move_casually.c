#include <duoji_control.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

#define SERVO_NUM 4
static const uint8_t ids[SERVO_NUM] = {1, 2, 3, 4};   // 舵机 ID

/**
 * @brief 可调位置：四个舵机同时运动到各自的脉冲位置
 * @param p1~p4   目标脉冲，500-2500（1500 = 中位）
 * @param time_ms 运动时长，四路一致则同步启停
 */
void servo_move(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4, uint16_t time_ms)
{
    uint16_t pos[SERVO_NUM] = {p1, p2, p3, p4};
    uint16_t tms[SERVO_NUM] = {time_ms, time_ms, time_ms, time_ms};

    if (duoji_move_batch_formatted(SERVO_NUM, ids, pos, tms) != DUOJI_OK) {
        ESP_LOGE(TAG, "move failed");
    }
}

void app_main(void)
{
    duoji_init(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    servo_move(2000, 2000, 2000, 2000, 1000);   // ← 位置/时长在这里改
}
