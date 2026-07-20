/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * HTTP MJPEG server for the DVP thermal stream.
 *
 * Why this encodes, when the rest of the project refuses to:
 *
 *   Raw Y16 at 384x288x25 fps is 44 Mbit/s. Measured host->air throughput on
 *   this board is ~11.9 Mbit/s (`sdiospeed` connected), so raw does not fit
 *   the radio even before TCP overhead - and no browser renders Y16 anyway.
 *   So the frame is windowed to 8-bit grey and JPEG-encoded.
 *
 *   The encode is done by the P4's JPEG peripheral, not the CPU, so this costs
 *   almost no core time. The AGC pass in front of it is CPU work and is the
 *   expensive part (two passes over 110 k pixels).
 *
 * The UVC path is untouched: it still carries unprocessed Y16. This server is
 * a second consumer of the same frame pool, which has a real cost - see
 * "Sharing frames with UVC" below.
 *
 * Sharing frames with UVC
 *   camera_get_frame() hands out the newest READY buffer and there are only
 *   DVP_READY_SLOTS of them. A frame claimed here is a frame UVC does not get,
 *   so streaming to a browser while a USB host is capturing splits the ~25 fps
 *   thermal rate between them. That is intended - "at the speed it can" - but
 *   it means the UVC frame rate visibly drops while a browser is watching.
 *   Only one streaming client is served at a time so the split is at worst 2x.
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "web_stream.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"

static const char *TAG = "web_stream";

/* Buffers are sized once for the largest frame the pipeline can deliver, so a
 * camera_set_resolution() while the server is up cannot outgrow them. PSRAM. */
#define WEB_MAX_W        640
#define WEB_MAX_H        480
#define WEB_MAX_PIXELS   (WEB_MAX_W * WEB_MAX_H)

#define WEB_JPEG_QUALITY 80
#define WEB_FRAME_TIMEOUT_MS 500

/* Multipart/x-mixed-replace: the format every browser renders natively as a
 * live <img>, with no JavaScript and no MSE. */
#define BOUNDARY "thermalframe"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char *STREAM_PART_HDR =
    "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

typedef struct {
    camera_ctx_t        *cam;
    jpeg_encoder_handle_t enc;
    uint16_t            *y16;        /* tap copy of the raw frame */
    uint8_t             *gray;       /* AGC output, 8bpp, JPEG encoder input */
    uint8_t             *jpg;        /* encoded bitstream */
    size_t               jpg_cap;
    volatile bool        client_busy; /* one streaming client at a time */
    volatile float       stream_fps;  /* last measured MJPEG rate, 0 when idle */
} web_ctx_t;

static web_ctx_t s_web;

/*
 * Window the 14-bit thermal frame into 8-bit grey.
 *
 * Percentile-clipped linear stretch, matching robust_range() in cam_viewer.py
 * (CLIP_PCT there, WEB_CLIP_PERMILLE here). A plain min/max stretch does not
 * work on this sensor: a single dead pixel near 0 and one hot pixel near full
 * scale drag the endpoints to the edges of the 14-bit space, so the actual
 * scene - which occupies a few hundred counts around a ~7000 baseline -
 * compresses into a narrow mid-grey band. That renders exactly as "bright and
 * low contrast, as if it were not normalised at all".
 *
 * Trimming each tail is done from a 1024-bin histogram rather than a sort:
 * two linear passes, no allocation, and 16-count resolution is far finer than
 * the window this ever picks.
 *
 * Per-frame adaptive, so a hot object entering the scene rescales everything -
 * fine for "is it working", wrong for radiometry. Anyone who needs real
 * numbers uses the Y16 UVC stream, not this.
 */
#define WEB_AGC_BINS      1024
#define WEB_AGC_SHIFT     4          /* 14-bit value >> 4 == 1024 bins */
#define WEB_CLIP_PERMILLE 5          /* trim 0.5% off each tail, as the host does */

static void agc_y16_to_gray(const uint16_t *src, uint8_t *dst, size_t pixels)
{
    uint32_t histo[WEB_AGC_BINS] = { 0 };
    for (size_t i = 0; i < pixels; i++) {
        histo[(src[i] & 0x3FFF) >> WEB_AGC_SHIFT]++;
    }

    /* Walk in from both ends until the clipped quota of pixels is consumed. */
    uint32_t quota = (uint32_t)((pixels * WEB_CLIP_PERMILLE) / 1000u);
    uint32_t acc = 0;
    int lo_bin = 0, hi_bin = WEB_AGC_BINS - 1;
    for (; lo_bin < hi_bin; lo_bin++) {
        acc += histo[lo_bin];
        if (acc > quota) break;
    }
    acc = 0;
    for (; hi_bin > lo_bin; hi_bin--) {
        acc += histo[hi_bin];
        if (acc > quota) break;
    }

    uint16_t lo = (uint16_t)(lo_bin << WEB_AGC_SHIFT);
    uint16_t hi = (uint16_t)((hi_bin << WEB_AGC_SHIFT) | ((1 << WEB_AGC_SHIFT) - 1));
    if (hi <= lo) {                         /* flat frame - avoid a divide by zero */
        memset(dst, 0, pixels);
        return;
    }
    /* Fixed-point scale: (v - lo) * 255 / (hi - lo), with the divide hoisted
     * out of the loop as a reciprocal so the per-pixel cost is a multiply.
     * Clamping is mandatory now that the tails are clipped: pixels outside
     * [lo,hi] exist by construction, and an unclamped (v - lo) on unsigned
     * would wrap to a huge value and speckle the image. */
    uint32_t range = (uint32_t)(hi - lo);
    uint32_t recip = (255u << 16) / range;
    for (size_t i = 0; i < pixels; i++) {
        uint16_t s = src[i] & 0x3FFF;
        if (s <= lo) {
            dst[i] = 0;
        } else if (s >= hi) {
            dst[i] = 255;
        } else {
            dst[i] = (uint8_t)((((uint32_t)(s - lo)) * recip) >> 16);
        }
    }
}

/* Grab one frame, AGC it, JPEG it. Returns encoded size, or 0 on failure. */
static uint32_t capture_jpeg(void)
{
    camera_ctx_t *cam = s_web.cam;
    uint32_t w = 0, h = 0;

    /*
     * Preferred path: take a copy of whatever the primary consumer is already
     * pulling. Claiming a buffer ourselves does not work while UVC is
     * streaming - it runs at priority 23 against this task's 5 and wins every
     * publish, which showed up as a stream that produced exactly zero frames.
     */
    if (!camera_tap_get(cam, s_web.y16, (size_t)WEB_MAX_PIXELS * 2, &w, &h,
                        WEB_FRAME_TIMEOUT_MS)) {
        /* Nothing is consuming frames (no USB host attached), so the tap will
         * never fill. With no competing consumer we can safely claim a buffer
         * directly - this is the uncontended case by definition. */
        void *hold = NULL;
        const uint16_t *raw = camera_get_frame(cam, &hold, WEB_FRAME_TIMEOUT_MS);
        if (!raw) {
            return 0;                        /* DVP really has no data */
        }
        w = cam->width;
        h = cam->height;
        size_t bytes = (size_t)w * h * 2;
        if (bytes > (size_t)WEB_MAX_PIXELS * 2) {
            camera_release_frame(cam, hold);
            ESP_LOGE(TAG, "frame %"PRIu32"x%"PRIu32" exceeds the buffer", w, h);
            return 0;
        }
        memcpy(s_web.y16, raw, bytes);
        camera_release_frame(cam, hold);     /* hand it back immediately */
    }

    size_t pixels = (size_t)w * h;
    if (!pixels || pixels > WEB_MAX_PIXELS) {
        ESP_LOGE(TAG, "bad frame geometry %"PRIu32"x%"PRIu32, w, h);
        return 0;
    }

    agc_y16_to_gray(s_web.y16, s_web.gray, pixels);

    jpeg_encode_cfg_t cfg = {
        .width         = w,
        .height        = h,
        .src_type      = JPEG_ENCODE_IN_FORMAT_GRAY,
        .sub_sample    = JPEG_DOWN_SAMPLING_GRAY,
        .image_quality = WEB_JPEG_QUALITY,
    };
    uint32_t out_len = 0;
    esp_err_t err = jpeg_encoder_process(s_web.enc, &cfg, s_web.gray, pixels,
                                         s_web.jpg, s_web.jpg_cap, &out_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "jpeg encode: %s", esp_err_to_name(err));
        return 0;
    }
    return out_len;
}

/* ------------------------------------------------------------------ stats */

/*
 * Per-core CPU load from the FreeRTOS run-time counters.
 *
 * ulTaskGetRunTimeCounter() is not exposed by this kernel build, so walk the
 * task list and pick out the two idle tasks by handle. The counters are
 * esp_timer microseconds (CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER), so
 * they are directly comparable to wall-clock elapsed time and no calibration
 * constant is needed. Load is measured BETWEEN calls - the first call after
 * boot has no previous sample and reports 0.
 */
#define STATS_MAX_TASKS 48

static void cpu_load(float out[2])
{
    static uint32_t prev_idle[2];
    static int64_t  prev_us;

    out[0] = out[1] = 0.0f;

    static TaskStatus_t tasks[STATS_MAX_TASKS];
    UBaseType_t n = uxTaskGetSystemState(tasks, STATS_MAX_TASKS, NULL);
    if (n == 0) {
        return;                              /* array too small - report nothing */
    }

    uint32_t idle[2] = { 0, 0 };
    for (int core = 0; core < 2; core++) {
        TaskHandle_t h = xTaskGetIdleTaskHandleForCore(core);
        for (UBaseType_t i = 0; i < n; i++) {
            if (tasks[i].xHandle == h) {
                idle[core] = (uint32_t)tasks[i].ulRunTimeCounter;
                break;
            }
        }
    }

    int64_t now = esp_timer_get_time();
    if (prev_us != 0 && now > prev_us) {
        uint32_t elapsed = (uint32_t)(now - prev_us);
        for (int core = 0; core < 2; core++) {
            uint32_t busy_idle = idle[core] - prev_idle[core];   /* wraps correctly */
            float frac = (float)busy_idle / (float)elapsed;
            if (frac > 1.0f) frac = 1.0f;
            out[core] = (1.0f - frac) * 100.0f;
        }
    }
    prev_idle[0] = idle[0];
    prev_idle[1] = idle[1];
    prev_us = now;
}

static esp_err_t stats_handler(httpd_req_t *req)
{
    float load[2];
    cpu_load(load);

    size_t int_free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t int_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t int_min   = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t ps_free   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t ps_total  = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t ps_min    = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

    char json[384];
    int n = snprintf(json, sizeof(json),
        "{\"cpu0\":%.1f,\"cpu1\":%.1f,"
        "\"int_free\":%u,\"int_total\":%u,\"int_min\":%u,"
        "\"psram_free\":%u,\"psram_total\":%u,\"psram_min\":%u,"
        "\"fps\":%.1f,\"uptime\":%lld}",
        load[0], load[1],
        (unsigned)int_free, (unsigned)int_total, (unsigned)int_min,
        (unsigned)ps_free, (unsigned)ps_total, (unsigned)ps_min,
        s_web.stream_fps, (long long)(esp_timer_get_time() / 1000000));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n);
}

/* ------------------------------------------------------------------ routes */

static esp_err_t index_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>P4 Thermal</title>"
        "<style>body{margin:0;background:#111;color:#ccc;font:14px system-ui;"
        "display:flex;flex-direction:column;align-items:center;gap:8px;padding:12px}"
        "img{width:min(96vw,768px);image-rendering:pixelated;border:1px solid #333}"
        "#s{display:grid;grid-template-columns:auto 1fr auto;gap:4px 10px;"
        "width:min(96vw,768px);font:12px ui-monospace,monospace}"
        ".b{background:#222;border-radius:3px;overflow:hidden;height:14px}"
        ".f{height:100%;background:#ffa34d;width:0;transition:width .3s}"
        ".v{text-align:right;color:#eee;white-space:nowrap}</style>"
        "<h3>ESP32-P4 thermal (MJPEG, 8-bit AGC)</h3>"
        "<img src='/stream'>"
        "<div id=s>"
        "<span>CPU0</span><div class=b><div class=f id=c0></div></div><span class=v id=c0v>-</span>"
        "<span>CPU1</span><div class=b><div class=f id=c1></div></div><span class=v id=c1v>-</span>"
        "<span>Internal</span><div class=b><div class=f id=ib></div></div><span class=v id=iv>-</span>"
        "<span>PSRAM</span><div class=b><div class=f id=pb></div></div><span class=v id=pv>-</span>"
        "<span>Stream</span><div></div><span class=v id=fv>-</span>"
        "</div>"
        "<p>Raw 14-bit data is on the USB UVC stream, not here.</p>"
        "<script>"
        "const kb=b=>b>1048576?(b/1048576).toFixed(1)+' MB':(b/1024).toFixed(0)+' kB';"
        "function set(bar,val,pct,txt){document.getElementById(bar).style.width=pct+'%';"
        "document.getElementById(val).textContent=txt;}"
        "async function u(){try{const r=await fetch('/stats');const d=await r.json();"
        "set('c0','c0v',d.cpu0,d.cpu0.toFixed(0)+'%');"
        "set('c1','c1v',d.cpu1,d.cpu1.toFixed(0)+'%');"
        "const iu=d.int_total-d.int_free,pu=d.psram_total-d.psram_free;"
        "set('ib','iv',100*iu/d.int_total,kb(iu)+' / '+kb(d.int_total)+'  (min free '+kb(d.int_min)+')');"
        "set('pb','pv',100*pu/d.psram_total,kb(pu)+' / '+kb(d.psram_total)+'  (min free '+kb(d.psram_min)+')');"
        "document.getElementById('fv').textContent=d.fps.toFixed(1)+' fps   up '+d.uptime+'s';"
        "}catch(e){}}"
        /* The MJPEG stream holds one socket open for its whole life, so polling
           must be gentle: httpd only has a handful of sockets and a fast poll
           starves the very stream we are reporting on. */
        "setInterval(u,1000);u();"
        "</script>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    uint32_t len = capture_jpeg();
    if (!len) {
        /* esp_http_server's httpd_err_code_t has no 503, so set it by hand. */
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "no frame from the DVP");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)s_web.jpg, len);
}

/*
 * The MJPEG loop runs in its own task, NOT in the httpd handler.
 *
 * esp_http_server dispatches every request from a single task, so a handler
 * that never returns - which is exactly what an endless multipart response is
 * - blocks the whole server for as long as the stream lasts. That is why the
 * stats on the index page stayed empty while the video played: /stats was
 * never reaching a handler. httpd_req_async_handler_begin() detaches the
 * request so the httpd task can go back to serving everything else.
 */
static volatile bool s_stream_stop;      /* ask the running stream to wind up */

static void stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;

    /* Set on the async copy, not the original: httpd_req_async_handler_begin()
     * duplicates the request, and response state set before the split is not
     * guaranteed to come across. */
    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    ESP_LOGI(TAG, "stream client connected");

    uint32_t frames = 0, empties = 0;
    int64_t  t0 = esp_timer_get_time();
    int64_t  fps_mark = t0;
    uint32_t fps_frames = 0;
    char     hdr[80];

    while (!s_stream_stop) {
        uint32_t len = capture_jpeg();
        if (!len) {
            /* DVP starved or encode failed. Don't kill the stream - a browser
             * that loses the connection stops retrying and shows a broken
             * image, which reads as "the board crashed" when it did not. */
            if (++empties > 20) {
                ESP_LOGW(TAG, "no frames for ~%d ms, ending stream", 20 * WEB_FRAME_TIMEOUT_MS);
                break;
            }
            continue;
        }
        empties = 0;

        int n = snprintf(hdr, sizeof(hdr), STREAM_PART_HDR, (unsigned)len);
        if (httpd_resp_send_chunk(req, hdr, n) != ESP_OK ||
            httpd_resp_send_chunk(req, (const char *)s_web.jpg, len) != ESP_OK) {
            break;                            /* client went away */
        }
        frames++;

        /* Publish a rolling rate for /stats, recomputed about once a second. */
        if (++fps_frames >= 4) {
            int64_t now = esp_timer_get_time();
            if (now - fps_mark >= 1000000) {
                s_web.stream_fps = (float)fps_frames * 1e6f / (float)(now - fps_mark);
                fps_mark = now;
                fps_frames = 0;
            }
        }
    }
    s_web.stream_fps = 0.0f;                  /* nobody watching */

    int64_t secs_us = esp_timer_get_time() - t0;
    ESP_LOGI(TAG, "stream client gone: %"PRIu32" frames in %.1f s (%.1f fps)",
             frames, (double)secs_us / 1e6,
             secs_us > 0 ? (double)frames * 1e6 / (double)secs_us : 0.0);

    httpd_resp_send_chunk(req, NULL, 0);      /* terminate the chunked response */
    httpd_req_async_handler_complete(req);    /* hands the socket back to httpd */

    s_web.client_busy = false;
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    /*
     * A browser reload opens the new stream before the old socket has died, so
     * "already connected" must not simply be refused - that is what made a
     * reloaded tab lag: the old task kept running and kept taking frames,
     * while the new request either waited or retried. Ask the incumbent to
     * stop and wait for it, so exactly one stream is ever pulling frames.
     */
    if (s_web.client_busy) {
        s_stream_stop = true;
        for (int i = 0; i < 40 && s_web.client_busy; i++) {   /* up to ~2 s */
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (s_web.client_busy) {
            httpd_resp_set_status(req, "503 Service Unavailable");
            httpd_resp_set_type(req, "text/plain");
            httpd_resp_sendstr(req, "previous stream did not release in time");
            return ESP_FAIL;
        }
    }

    httpd_req_t *areq = NULL;
    esp_err_t err = httpd_req_async_handler_begin(req, &areq);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "async begin: %s", esp_err_to_name(err));
        return err;
    }

    s_stream_stop = false;
    s_web.client_busy = true;
    /* Priority 5: above the idle/httpd background work but well below the UVC
     * task, so a browser can never slow the USB path down. */
    if (xTaskCreate(stream_task, "mjpeg", 6144, areq, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "cannot create stream task");
        s_web.client_busy = false;
        httpd_req_async_handler_complete(areq);
        return ESP_FAIL;
    }
    return ESP_OK;    /* httpd task is free again; stream_task owns the socket */
}

/* -------------------------------------------------------------------- init */

esp_err_t web_stream_start(camera_ctx_t *cam)
{
    ESP_RETURN_ON_FALSE(cam, ESP_ERR_INVALID_ARG, TAG, "no camera ctx");
    s_web.cam = cam;

    jpeg_encode_engine_cfg_t eng = {
        .intr_priority = 0,
        .timeout_ms    = 100,     /* >> the ~few ms a 384x288 grey encode takes */
    };
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&eng, &s_web.enc), TAG, "jpeg engine");

    /* Copy buffer for the observer tap, plus the pipeline's own tap storage. */
    ESP_RETURN_ON_ERROR(camera_tap_init(cam, (size_t)WEB_MAX_PIXELS * 2), TAG, "tap init");
    s_web.y16 = heap_caps_malloc((size_t)WEB_MAX_PIXELS * 2, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(s_web.y16, ESP_ERR_NO_MEM, TAG, "y16 buffer alloc");

    size_t got = 0;
    jpeg_encode_memory_alloc_cfg_t in_cfg  = { .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER };
    jpeg_encode_memory_alloc_cfg_t out_cfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };

    s_web.gray = jpeg_alloc_encoder_mem(WEB_MAX_PIXELS, &in_cfg, &got);
    ESP_RETURN_ON_FALSE(s_web.gray, ESP_ERR_NO_MEM, TAG, "grey buffer alloc");

    /* Half a byte per pixel is generous for quality-80 grey; a noisy thermal
     * frame compresses far better than that, and the encoder errors out rather
     * than overrunning if it ever did not. */
    s_web.jpg = jpeg_alloc_encoder_mem(WEB_MAX_PIXELS / 2, &out_cfg, &got);
    ESP_RETURN_ON_FALSE(s_web.jpg, ESP_ERR_NO_MEM, TAG, "jpeg buffer alloc");
    s_web.jpg_cap = got;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.ctrl_port        = 32768;
    cfg.stack_size       = 8192;   /* default 4 k is tight once TLS-free httpd + our locals stack up */
    /* The MJPEG response never ends, so its socket is occupied for the whole
     * session while /stats polls beside it and the browser may hold a spare
     * keep-alive. Too few sockets here and lru_purge starts reclaiming the
     * stream itself to serve the polls that are reporting on it. */
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;   /* a browser tab left open must not wedge the listener */

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "httpd_start");

    const httpd_uri_t routes[] = {
        { .uri = "/",         .method = HTTP_GET, .handler = index_handler    },
        { .uri = "/stream",   .method = HTTP_GET, .handler = stream_handler   },
        { .uri = "/snapshot", .method = HTTP_GET, .handler = snapshot_handler },
        { .uri = "/stats",    .method = HTTP_GET, .handler = stats_handler    },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG,
                            "register %s", routes[i].uri);
    }

    ESP_LOGI(TAG, "HTTP server up on :80  (/ = viewer, /stream = MJPEG, /snapshot = one JPEG)");
    return ESP_OK;
}
