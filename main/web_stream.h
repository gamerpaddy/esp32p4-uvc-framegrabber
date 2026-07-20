/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * HTTP MJPEG view of the DVP thermal stream, for looking at the camera over
 * Wi-Fi without a USB host attached.
 *
 * This is deliberately NOT the project's data path. UVC carries raw Y16 with
 * zero processing precisely so the host PC sees exact 14-bit ADC counts; this
 * server does the opposite - AGC to 8-bit and lossy JPEG - because a browser
 * cannot render Y16 and the link cannot carry it (see web_stream.c).
 */

#pragma once

#include "esp_err.h"
#include "camera_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Start the HTTP server on port 80. Safe to call before Wi-Fi has an address:
 * the listener binds to all interfaces and simply accepts nothing until an IP
 * arrives. `cam` must outlive the server (it is the same context UVC uses).
 */
esp_err_t web_stream_start(camera_ctx_t *cam);

#ifdef __cplusplus
}
#endif
