/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef COMMON_H
#define COMMON_H

/* STD APIs */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ESP APIs */
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* FreeRTOS APIs */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/gap/ble_svc_gap.h"  /* 添加这行 */

/* Defines */
#define TAG "BLE_Receiver"
#define DEVICE_NAME "ESP32S3_BLE"

#endif // COMMON_H