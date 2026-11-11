#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "6050_middle.h"

static const char *TAG = "MPU6050_TEST";

extern "C" void app_main(void)
{
    printf("=== MPU6050 Test Application ===\n");
    
    // 初始化MPU6050
    mpu6050_init();
    
    // 延迟一段时间等待初始化完成
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    float roll, pitch, yaw;
    
    while(1) {
        // 获取姿态数据
        mpu6050_get_data(&roll, &pitch, &yaw);
        
        // 打印数据
        printf("Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n", roll, pitch, yaw);
        
        // 延迟0.5秒
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}