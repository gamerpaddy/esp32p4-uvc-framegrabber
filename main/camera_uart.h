/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART command/response channel to the thermal camera (UART1, 38400 8N1,
 * pins per camera_uart.c). Frames are STX(0x02), len_lo, len_hi, payload,
 * checksum (sum of payload mod 256), ETX(0x03); payload = "CMD," [+ "value,"].
 *
 * Examples: KBD,C = FFC shutter; SSM,1 = filtering on; GCO = report gain 0..255.
 * Commands are typed on the CDC-ACM COM port (composite USB device, alongside
 * UVC on the same HS port); camera replies go back to that COM port prefixed
 * "cam_uart:" and are also printed to the log console. "BIT,8"/"BIT,14" is
 * handled locally (switches the delivered interleave phase).
 */

#pragma once

#include "esp_err.h"
#include "camera_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Install UART1 + start the RX-print task and the typed-command console.
 * `cam` is used to handle the local "BIT" command (8/14-bit phase switch);
 * may be NULL to disable that. */
esp_err_t camera_uart_start(camera_ctx_t *cam);

/*
 * Send a framed command. `cmd` must be exactly 3 chars (e.g. "KBD", "SSM",
 * "GCO"); `values` may be NULL/empty for value-less commands.
 */
esp_err_t camera_uart_send(const char *cmd, const char *values);

#ifdef __cplusplus
}
#endif
