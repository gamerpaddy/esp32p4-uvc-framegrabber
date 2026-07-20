/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure DVP → UVC bridge.  Zero processing: the 16-bit DVP buffer is handed
 * directly to TinyUSB as a Y16 uncompressed frame.  Host PC receives raw
 * 14-bit thermal counts in the lower 14 bits of each uint16_t sample.
 *
 * Python decode example:
 *   import cv2, numpy as np
 *   cap = cv2.VideoCapture(0)
 *   cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # keep raw bytes
 *   ret, raw = cap.read()
 *   frame14 = np.frombuffer(raw, dtype='<u2').reshape(H, W) & 0x3FFF
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "usb_device_uvc.h"
#include "uvc_streaming.h"
#include "uvc_frame_config.h"

static const char *TAG = "uvc_stream";

#define FRAME_BYTES  (THERMAL_WIDTH * THERMAL_HEIGHT * 2)

/* ---- UVC callbacks ------------------------------------------------------ */

static void on_stream_stop(void *cb_ctx);

static esp_err_t on_stream_start(uvc_format_t uvc_format, int width, int height,
                                  int rate, void *cb_ctx)
{
    uvc_stream_ctx_t *ctx = (uvc_stream_ctx_t *)cb_ctx;

    if (ctx->streaming) {
        on_stream_stop(cb_ctx);
    }

    /* Host selected one of the advertised frames (640x480 or 384x288). */
    ctx->width  = (uint16_t)(width  > 0 ? width  : THERMAL_WIDTH);
    ctx->height = (uint16_t)(height > 0 ? height : THERMAL_HEIGHT);

    ESP_LOGI(TAG, "Stream start: %dx%d @%dfps (Y16)", ctx->width, ctx->height, rate);

    /* Match the delivered frame size to what the host committed. */
    camera_set_resolution(&ctx->camera, ctx->width, ctx->height);

    esp_err_t ret = camera_start(&ctx->camera);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera_start: %s", esp_err_to_name(ret));
        return ret;
    }

    ctx->streaming = true;
    return ESP_OK;
}

static void on_stream_stop(void *cb_ctx)
{
    uvc_stream_ctx_t *ctx = (uvc_stream_ctx_t *)cb_ctx;
    if (!ctx->streaming) return;
    ctx->streaming = false;
    camera_stop(&ctx->camera);
    ESP_LOGI(TAG, "Stream stopped");
}

/*
 * Called by TinyUSB UVC task when it needs the next frame to transmit.
 * Blocks until DVP delivers a complete frame (≤1 frame period + margin).
 * Returns a checkerboard NO SIGNAL frame on timeout to keep the stream alive.
 */
static uvc_fb_t *on_fb_get(void *cb_ctx)
{
    uvc_stream_ctx_t *ctx = (uvc_stream_ctx_t *)cb_ctx;
    const uint16_t *raw = NULL;

    /* 5 × frame period as timeout so we miss at most a few frames on hiccups. */
    const uint32_t timeout_ms = (5 * 1000) / CONFIG_THERMAL_FPS;

    /*
     * A BULK UVC stream stays COMMITTED after the host app closes / the bus
     * suspends, so the UVC task keeps calling here even though on_stream_stop
     * already stopped the DVP. Don't touch camera_get_frame() then — its
     * timeouts would trip the stall watchdog, which would pointlessly bounce
     * (restart) the stopped capture. Pace the loop and fall through to the
     * NO SIGNAL frame instead.
     */
    if (!ctx->streaming) {
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    } else {
        raw = camera_get_frame(&ctx->camera, &ctx->pending_buf, timeout_ms);
    }
    if (!raw) {
        /* DVP has no data — send a checkerboard NO SIGNAL frame so the UVC
         * stream stays alive and the host sees something unambiguous. */
        ctx->pending_buf = NULL;
        int64_t us = esp_timer_get_time();
        ctx->fb.buf    = (uint8_t *)ctx->nosignal_buf;
        ctx->fb.len    = ctx->camera.frame_size;   /* match the active resolution */
        ctx->fb.width  = ctx->width;
        ctx->fb.height = ctx->height;
        ctx->fb.format = UVC_FORMAT_UNCOMPR;
        ctx->fb.timestamp.tv_sec  = (long)(us / 1000000LL);
        ctx->fb.timestamp.tv_usec = (long)(us % 1000000LL);
        return &ctx->fb;
    }

    int64_t us = esp_timer_get_time();

    ctx->fb.buf    = (uint8_t *)raw;          /* zero-copy: DMA buffer → USB */
    ctx->fb.len    = ctx->camera.frame_size;  /* match the active resolution */
    ctx->fb.width  = ctx->width;
    ctx->fb.height = ctx->height;
    ctx->fb.format = UVC_FORMAT_UNCOMPR;      /* Y16 uncompressed */
    ctx->fb.timestamp.tv_sec  = (long)(us / 1000000LL);
    ctx->fb.timestamp.tv_usec = (long)(us % 1000000LL);

    ctx->perf_frame_count++;
    ctx->perf_byte_count += ctx->camera.frame_size;

    return &ctx->fb;
}

/* Called when TinyUSB has finished transmitting the frame. */
static void on_fb_return(uvc_fb_t *fb, void *cb_ctx)
{
    uvc_stream_ctx_t *ctx = (uvc_stream_ctx_t *)cb_ctx;
    if (ctx->pending_buf) {
        camera_release_frame(&ctx->camera, ctx->pending_buf);
    }
    /* pending_buf == NULL means the nosignal buffer was sent — nothing to release. */
}

/* ---- Init --------------------------------------------------------------- */

esp_err_t uvc_stream_init(uvc_stream_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));

    ESP_RETURN_ON_ERROR(camera_open(&ctx->camera), TAG, "camera_open failed");

    /* NO SIGNAL frame: 16×16-block checkerboard (0x0000 / 0x3FFF).
     * Host AGC will show full contrast so the pattern is unmistakable. */
    ctx->nosignal_buf = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(ctx->nosignal_buf, ESP_ERR_NO_MEM, TAG,
                        "nosignal buffer alloc failed (%d B)", FRAME_BYTES);
    for (int y = 0; y < THERMAL_HEIGHT; y++) {
        for (int x = 0; x < THERMAL_WIDTH; x++) {
            int block = ((x >> 4) + (y >> 4)) & 1;
            ctx->nosignal_buf[y * THERMAL_WIDTH + x] = block ? 0x3FFF : 0x0000;
        }
    }

    /*
     * UVC transfer buffer: TinyUSB copies from the DVP PSRAM buffer into
     * this buffer before handing off to the USB DMA.  Must be large enough
     * for one full Y16 frame.
     */
    void *uvc_buf = heap_caps_aligned_alloc(64, FRAME_BYTES,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(uvc_buf, ESP_ERR_NO_MEM, TAG,
                        "UVC transfer buffer alloc failed (%d B)", FRAME_BYTES);

    uvc_device_config_t uvc_cfg = {
        .uvc_buffer      = uvc_buf,
        .uvc_buffer_size = FRAME_BYTES,
        .start_cb        = on_stream_start,
        .fb_get_cb       = on_fb_get,
        .fb_return_cb    = on_fb_return,
        .stop_cb         = on_stream_stop,
        .cb_ctx          = ctx,
    };

    ESP_RETURN_ON_ERROR(uvc_device_config(0, &uvc_cfg), TAG, "UVC config failed");
    ESP_RETURN_ON_ERROR(uvc_device_init(),              TAG, "UVC init failed");

    ESP_LOGI(TAG, "Thermal bridge ready: %dx%d Y16 @%d fps, %d B/frame",
             THERMAL_WIDTH, THERMAL_HEIGHT, THERMAL_FPS, FRAME_BYTES);
    return ESP_OK;
}
