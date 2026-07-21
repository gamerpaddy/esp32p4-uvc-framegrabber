/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Persistent user-visible settings kept in NVS so the device comes back with
 * whatever the user last used. Currently just the delivered frame resolution;
 * add more fields here as they show up.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time NVS bring-up. Safe to call more than once (it deduplicates against
 * wifi_console_start / any other init path). Call before any get/set. */
esp_err_t settings_init(void);

/* Load the saved delivered resolution. Returns ESP_ERR_NOT_FOUND when nothing
 * has ever been saved — caller falls back to its compile-time default. */
esp_err_t settings_get_resolution(uint32_t *w, uint32_t *h);

/* Persist the delivered resolution. Called after every successful RES,W,H
 * apply and after the UVC host commits its own frame size, so the next boot
 * comes up on whichever resolution was last actually in use. */
esp_err_t settings_set_resolution(uint32_t w, uint32_t h);

#ifdef __cplusplus
}
#endif
