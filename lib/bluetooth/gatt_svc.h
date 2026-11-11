#ifndef BLE_CMD_H
#define BLE_CMD_H

#include <stdbool.h>
#include <stdint.h>
#include "host/ble_gap.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化函数 */
void ble_cmd_init(void);

/* 检查是否有新消息 */
bool ble_cmd_available(void);

/* 获取一个字符（逐个字符读取） */
char ble_cmd_get(void);

/* 获取完整消息 */
int ble_cmd_get_message(char *buf, int buf_len);

/* 设置消息接收回调 */
typedef void (*ble_cmd_callback_t)(const char *cmd, int len);
void ble_cmd_set_callback(ble_cmd_callback_t callback);

/* 发送响应到客户端 */
int ble_send_response(const char *message, int len);

/* GAP 事件处理（需要在 GAP 回调中调用） */
void gatt_svc_gap_event_handler(struct ble_gap_event *event);

#ifdef __cplusplus
}
#endif

#endif