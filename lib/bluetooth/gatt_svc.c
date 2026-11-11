/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "gatt_svc.h"
#include "common.h"

/* 添加必要的头文件 */
#include <stdlib.h>
#include <string.h>

/* 私有变量 */
static ble_cmd_callback_t cmd_callback = NULL;

/* 接收缓冲区 */
#define RX_BUFFER_SIZE 512
static char rx_buffer[RX_BUFFER_SIZE];
static int rx_index = 0;

/* GATT 句柄和连接句柄 */
static uint16_t rx_char_handle = 0;
static uint16_t tx_char_handle = 0;
static uint16_t current_conn_handle = 0;

/* 消息服务 UUID (16-bit) */
static const ble_uuid16_t message_service_uuid = BLE_UUID16_INIT(0xFFE0);
static const ble_uuid16_t rx_char_uuid = BLE_UUID16_INIT(0xFFE1);
static const ble_uuid16_t tx_char_uuid = BLE_UUID16_INIT(0xFFE2);

/* 函数前向声明 */
static int message_service_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int send_notification(uint16_t conn_handle, uint16_t char_handle, 
                              uint8_t *data, uint16_t len);

/* GATT 服务定义 */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &message_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX 特征值 - 接收消息 */
                .uuid = &rx_char_uuid.u,
                .access_cb = message_service_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &rx_char_handle,
            },
            {
                /* TX 特征值 - 发送响应 */
                .uuid = &tx_char_uuid.u,
                .access_cb = message_service_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_char_handle,
            },
            { 0 },
        },
    },
    { 0 },
};

/* 访问回调函数 */
static int message_service_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc;
    
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (attr_handle == rx_char_handle) {
                struct os_mbuf *om = ctxt->om;
                if (om == NULL || om->om_len == 0) {
                    ESP_LOGW(TAG, "Empty message received");
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                }
                
                uint16_t data_len = om->om_len;
                char *data = malloc(data_len + 1);
                if (data == NULL) {
                    ESP_LOGE(TAG, "Memory allocation failed");
                    return BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                
                rc = os_mbuf_copydata(om, 0, data_len, (uint8_t *)data);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Failed to copy data");
                    free(data);
                    return BLE_ATT_ERR_UNLIKELY;
                }
                data[data_len] = '\0';
                
                ESP_LOGI(TAG, "📨 Received: %.*s (%d bytes)", data_len, data, data_len);
                
                /* 保存到缓冲区 */
                if (rx_index + data_len < RX_BUFFER_SIZE) {
                    memcpy(rx_buffer + rx_index, data, data_len);
                    rx_index += data_len;
                } else {
                    ESP_LOGW(TAG, "Buffer full");
                }
                
                /* 调用回调 */
                if (cmd_callback) {
                    cmd_callback(data, data_len);
                }
                
                free(data);
                
                /* 发送确认 */
                send_notification(conn_handle, tx_char_handle, (uint8_t *)"OK", 2);
            }
            break;
            
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if (attr_handle == tx_char_handle) {
                char status[32];
                snprintf(status, sizeof(status), "Ready, buf:%d", rx_index);
                rc = os_mbuf_append(ctxt->om, status, strlen(status));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown operation: %d", ctxt->op);
            return BLE_ATT_ERR_UNLIKELY;
    }
    
    return 0;
}

/* 发送通知 */
static int send_notification(uint16_t conn_handle, uint16_t char_handle, 
                              uint8_t *data, uint16_t len) {
    struct os_mbuf *om;
    
    if (conn_handle == 0 || char_handle == 0 || data == NULL || len == 0) {
        return -1;
    }
    
    om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to create mbuf");
        return -1;
    }
    
    int rc = ble_gattc_notify_custom(conn_handle, char_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Notify failed: %d", rc);
    }
    return rc;
}

/* ========== 公共函数 ========== */

void ble_cmd_init(void) {
    int rc;
    
    /* 初始化 GATT 服务 */
    ble_svc_gatt_init();
    
    /* 计数并添加服务 */
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Count GATT services failed: %d", rc);
        return;
    }
    
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Add GATT services failed: %d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "✅ BLE Message Service initialized");
    ESP_LOGI(TAG, "   Service UUID: FFE0");
    ESP_LOGI(TAG, "   RX Char: FFE1 (Write)");
    ESP_LOGI(TAG, "   TX Char: FFE2 (Read/Notify)");
}

bool ble_cmd_available(void) {
    return rx_index > 0;
}

char ble_cmd_get(void) {
    if (rx_index <= 0) return '\0';
    char c = rx_buffer[0];
    for (int i = 0; i < rx_index - 1; i++) {
        rx_buffer[i] = rx_buffer[i + 1];
    }
    rx_index--;
    return c;
}

int ble_cmd_get_message(char *buf, int buf_len) {
    if (buf == NULL || buf_len <= 0 || rx_index == 0) {
        return 0;
    }
    
    int copy_len = (rx_index < buf_len) ? rx_index : buf_len - 1;
    memcpy(buf, rx_buffer, copy_len);
    buf[copy_len] = '\0';
    
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    
    return copy_len;
}

void ble_cmd_set_callback(ble_cmd_callback_t callback) {
    cmd_callback = callback;
}

int ble_send_response(const char *message, int len) {
    if (current_conn_handle == 0) {
        ESP_LOGW(TAG, "No device connected");
        return -1;
    }
    if (tx_char_handle == 0) {
        ESP_LOGE(TAG, "TX char not initialized");
        return -1;
    }
    return send_notification(current_conn_handle, tx_char_handle, 
                             (uint8_t *)message, len);
}

void gatt_svc_gap_event_handler(struct ble_gap_event *event) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            current_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "🔗 Device connected, handle: %d", current_conn_handle);
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "🔌 Device disconnected");
            current_conn_handle = 0;
            rx_index = 0;
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
            break;
            
        default:
            break;
    }
}