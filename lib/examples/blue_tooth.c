/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"

#include "gap.h"
#include "gatt_svc.h"

/* 定义 TAG 和设备名称 */
#define TAG "BLE_Receiver"
#define DEVICE_NAME "ESP32_BLE_Receiver"

/* 消息接收回调函数 */
static void on_message_received(const char *msg, int len)
{
    if (msg == NULL || len <= 0) {
        ESP_LOGW(TAG, "Invalid message received");
        return;
    }
    
    ESP_LOGI(TAG, "📨 Received message (%d bytes): %.*s", len, len, msg);
    
    /* 处理接收到的命令 */
    if (strncmp(msg, "LED_ON", len) == 0) {
        ESP_LOGI(TAG, "💡 Turning LED ON");
        ble_send_response("LED turned ON", strlen("LED turned ON"));
        
    } else if (strncmp(msg, "LED_OFF", len) == 0) {
        ESP_LOGI(TAG, "💡 Turning LED OFF");
        ble_send_response("LED turned OFF", strlen("LED turned OFF"));
        
    } else if (strncmp(msg, "STATUS", len) == 0) {
        const char *status = "ESP32 BLE Receiver is running";
        ble_send_response(status, strlen(status));
        
    } else if (strncmp(msg, "PING", len) == 0) {
        ble_send_response("PONG", strlen("PONG"));
        
    } else if (strncmp(msg, "ECHO:", 5) == 0 && len > 5) {
        /* 回显功能 */
        ble_send_response(msg + 5, len - 5);
        
    } else {
        /* 默认回显收到的消息 */
        char echo[128];
        snprintf(echo, sizeof(echo), "Echo: %.*s", len, msg);
        ble_send_response(echo, strlen(echo));
    }
}

/* NimBLE 主机同步回调 */
static void ble_host_sync_cb(void)
{
    ESP_LOGI(TAG, "NimBLE host synchronized");
    
    /* 开始广播 */
    adv_init();
    ESP_LOGI(TAG, "BLE advertising started");
}

/* NimBLE 主机重置回调 */
static void ble_host_reset_cb(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason: %d", reason);
}

/* NimBLE 主机任务函数 - 注意参数类型必须是 void * */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    
    /* 配置 NimBLE 主机 */
    ble_hs_cfg.sync_cb = ble_host_sync_cb;
    ble_hs_cfg.reset_cb = ble_host_reset_cb;
    ble_hs_cfg.store_status_cb = NULL;
    
    ESP_LOGI(TAG, "NimBLE host configured");
    
    /* 运行 NimBLE 主机 */
    nimble_port_run();
    
    /* 不应该到达这里 */
    vTaskDelete(NULL);
}

/* 打印启动信息 */
static void print_startup_info(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32 BLE Message Receiver");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Device Name: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");
}

/* 应用入口函数 */
void app_main(void)
{
    esp_err_t ret;
    
    /* 打印启动信息 */
    print_startup_info();
    
    /* 1. 初始化 NVS 闪存 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupted, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS flash initialized");
    
    /* 2. 初始化 NimBLE 主机 */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ NimBLE port init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "✓ NimBLE port initialized");
    
    /* 3. 初始化 GAP 服务 */
    if (gap_init() != 0) {
        ESP_LOGE(TAG, "✗ GAP initialization failed");
        return;
    }
    ESP_LOGI(TAG, "✓ GAP service initialized");
    
    /* 4. 初始化 GATT 消息服务 */
    ble_cmd_init();
    ESP_LOGI(TAG, "✓ GATT message service initialized");
    
    /* 5. 设置消息接收回调 */
    ble_cmd_set_callback(on_message_received);
    ESP_LOGI(TAG, "✓ Message callback registered");
    
    /* 6. 启动 NimBLE 主机任务 */
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✓ BLE Receiver is ready!");
    ESP_LOGI(TAG, "Waiting for connections...");
    ESP_LOGI(TAG, "========================================");
}