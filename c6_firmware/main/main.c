/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted coprocessor firmware for the ESP32-C6 on the JC-ESP32P4-M3.
 *
 * There is deliberately almost nothing here: the esp_hosted component's
 * coprocessor role registers its own startup hooks, brings up the SDIO slave
 * and serves Wi-Fi RPC + data to the P4 host. All this needs to do is give it
 * NVS (for Wi-Fi calibration and config) and a default event loop.
 */

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

static const char *TAG = "c6_cp";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted coprocessor starting (SDIO slave -> ESP32-P4)");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());
}
