/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "camera_pipeline.h"
#include "usb_device_uvc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    camera_ctx_t  camera;
    bool          streaming;
    uint16_t      width;
    uint16_t      height;
    uvc_fb_t      fb;
    void         *pending_buf;        /* camera buffer held while UVC transmits; NULL = nosignal */
    uint16_t     *nosignal_buf;       /* static checkerboard sent when DVP has no data */
    volatile uint32_t perf_frame_count;
    volatile uint64_t perf_byte_count;
} uvc_stream_ctx_t;

/* Initialize UVC pipeline: open camera, register callbacks, start USB. */
esp_err_t uvc_stream_init(uvc_stream_ctx_t *ctx);

/* True when a host has committed and is actively pulling frames. The camera
 * resolution is host-owned in that state; anything that wants to change it
 * must go through UVC re-commit (i.e. tell the host viewer to reopen), not
 * touch the pipeline directly, or the in-flight frame length desyncs. */
bool uvc_stream_is_active(void);

#ifdef __cplusplus
}
#endif
