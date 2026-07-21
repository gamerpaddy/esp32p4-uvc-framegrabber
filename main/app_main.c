/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 thermal camera → USB UVC bridge.
 * Captures 14-bit parallel DVP video and streams it as Y16 uncompressed UVC.
 * No encoding, no AGC — raw thermal ADC counts go straight to the host PC.
 */

#include "esp_log.h"
#include "camera_pipeline.h"
#include "uvc_streaming.h"
#include "camera_uart.h"
#include "wifi_console.h"
#include "web_stream.h"
#include "settings.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-P4 Thermal UVC Bridge ===");

    /* NVS up first — settings_get_resolution() is consulted during
     * uvc_stream_init() to restore the last-used delivered frame size. */
    settings_init();

    esp_err_t ret = camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera_init: %s", esp_err_to_name(ret));
        return;
    }

    static uvc_stream_ctx_t stream_ctx;
    ret = uvc_stream_init(&stream_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uvc_stream_init: %s", esp_err_to_name(ret));
        return;
    }

    /* UART command/response channel to the camera (UART1 TX=22 RX=23 @38400,
     * see camera_uart.c).
     * Type commands on the console: e.g. KBD,C (FFC) / SSM,1 / GCO. */
    ret = camera_uart_start(&stream_ctx.camera);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "camera_uart_start: %s (continuing without camera UART)",
                 esp_err_to_name(ret));
    }

    /* Wi-Fi via the on-module ESP32-C6 over SDIO, plus the console on UART0.
     * Commands: scan / join / leave / status / sdiospeed / wifispeed. */
    ret = wifi_console_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "wifi_console_start: %s (continuing without Wi-Fi)",
                 esp_err_to_name(ret));
    }

    /* Browser view of the thermal stream: http://<board-ip>/
     * Started unconditionally - the listener just sits idle until an IP shows
     * up, so it survives joining a network later from the console. */
    ret = web_stream_start(&stream_ctx.camera);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "web_stream_start: %s (continuing without the web view)",
                 esp_err_to_name(ret));
    }

    /* app_main returns; TinyUSB and DVP tasks keep the device alive. */
}
