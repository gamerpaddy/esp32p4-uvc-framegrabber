/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "settings";
static const char *NS  = "thermal";

/* nvs_flash_init is safe to call twice, but track our own guard so we can log
 * the recovery cleanly on the first successful bring-up. */
static bool s_ready;

esp_err_t settings_init(void)
{
    if (s_ready) return ESP_OK;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erase+re-init (was %s)", esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
        return err;
    }
    s_ready = true;
    return ESP_OK;
}

esp_err_t settings_get_resolution(uint32_t *w, uint32_t *h)
{
    if (!w || !h) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nh;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &nh);
    if (err != ESP_OK) return err;
    uint16_t sw = 0, sh = 0;
    err = nvs_get_u16(nh, "res_w", &sw);
    if (err == ESP_OK) err = nvs_get_u16(nh, "res_h", &sh);
    nvs_close(nh);
    if (err != ESP_OK) return err;
    if (sw == 0 || sh == 0) return ESP_ERR_NOT_FOUND;
    *w = sw;
    *h = sh;
    return ESP_OK;
}

esp_err_t settings_set_resolution(uint32_t w, uint32_t h)
{
    if (w == 0 || h == 0 || w > 0xFFFFu || h > 0xFFFFu) return ESP_ERR_INVALID_ARG;

    /* Skip the write if it would be a no-op. Avoids burning NVS wear cycles on
     * every UVC re-commit at the same resolution the user picked yesterday. */
    uint32_t cw = 0, ch = 0;
    if (settings_get_resolution(&cw, &ch) == ESP_OK && cw == w && ch == h) {
        return ESP_OK;
    }

    nvs_handle_t nh;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &nh);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(nh, "res_w", (uint16_t)w);
    if (err == ESP_OK) err = nvs_set_u16(nh, "res_h", (uint16_t)h);
    if (err == ESP_OK) err = nvs_commit(nh);
    nvs_close(nh);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "saved resolution %ux%u", (unsigned)w, (unsigned)h);
    }
    return err;
}
