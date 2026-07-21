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
#include "camera_uart.h"
#include "uvc_frame_config.h"
#include "uvc_streaming.h"   /* uvc_stream_is_active() — picks tap vs direct */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "driver/jpeg_encode.h"

/* LWIP socket options for the stream socket — TCP_NODELAY in particular is
 * the single biggest anti-stutter change over Wi-Fi: without it the multipart
 * header goes out, then Nagle waits for an ACK before releasing the JPEG
 * body, so every frame eats one round-trip of Wi-Fi jitter as visible pause.
 * lwip/sockets.h provides TCP_NODELAY, IPPROTO_TCP, SO_SNDTIMEO, setsockopt,
 * and struct timeval; nothing else is needed. */
#include "lwip/sockets.h"

static const char *TAG = "web_stream";

/* Buffers are sized once for the largest frame the pipeline can deliver, so a
 * camera_set_resolution() while the server is up cannot outgrow them. Track
 * THERMAL_MAX_* (uvc_frame_config.h) so bumping the pixel budget in one place
 * propagates here. PSRAM. */
#define WEB_MAX_W        THERMAL_MAX_WIDTH
#define WEB_MAX_H        THERMAL_MAX_HEIGHT
#define WEB_MAX_PIXELS   THERMAL_MAX_PIXELS

#define WEB_JPEG_QUALITY 80
#define WEB_FRAME_TIMEOUT_MS 500

/*
 * How long capture_jpeg() waits on the observer tap once it has established
 * (via uvc_stream_is_active()) that a UVC host really is consuming frames.
 * A comfortable multiple of the ~40 ms phase period so a live session's tap
 * signal never times out spuriously. Reaching this timeout now means UVC went
 * idle during the wait — we lose that one frame and take the direct path next
 * time round.
 *
 * WEB_FRAME_TIMEOUT_MS above (500 ms) is used by the direct-capture path and
 * by the stream_task's "no frames" break condition.
 */
#define WEB_TAP_PROBE_MS  120

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
 * Percentile-clipped linear stretch. This is the firmware equivalent of
 * robust_range() in cam_viewer.py, which is:
 *
 *     lo = np.percentile(frame14, CLIP_PCT)          # 0.5
 *     hi = np.percentile(frame14, 100 - CLIP_PCT)    # 99.5
 *
 * CLIP_PCT (0.5%) and WEB_CLIP_PERMILLE (5/1000) are the same number, and the
 * bin interpolation below reproduces np.percentile's linear interpolation, so
 * the two agree to within a count or so on the same frame.
 *
 * A plain min/max stretch does not work on this sensor: a single dead pixel
 * near 0 and one hot pixel near full scale drag the endpoints to the edges of
 * the 14-bit space, so the actual scene - which occupies a few hundred counts
 * around a ~7000 baseline - compresses into a narrow mid-grey band. That
 * renders exactly as "bright and low contrast, as if it were not normalised at
 * all".
 *
 * Trimming each tail is done from a 1024-bin histogram rather than a sort: two
 * linear passes and no allocation, where NumPy can afford to partition the
 * whole array.
 *
 * TWO DELIBERATE DIFFERENCES from the viewer, both for reasons the viewer does
 * not have on a PC:
 *   - the histogram subsamples (WEB_AGC_SAMPLE_STRIDE); a percentile does not
 *     need every pixel, and this is the expensive pass. Unbiased, marginally
 *     noisier.
 *   - the window is IIR-smoothed across frames (see below). The viewer
 *     recomputes per frame and is happy to flicker; on the MJPEG stream that
 *     flicker was a reported symptom.
 *
 * Per-frame adaptive, so a hot object entering the scene rescales everything -
 * fine for "is it working", wrong for radiometry. Anyone who needs real
 * numbers uses the Y16 UVC stream, not this.
 */
#define WEB_AGC_BINS      1024
#define WEB_AGC_SHIFT     4          /* 14-bit value >> 4 == 1024 bins */
#define WEB_CLIP_PERMILLE 5          /* trim 0.5% off each tail, as the host does */

/* Smoothed AGC window, carried across frames (see the IIR note below). Touched
 * only from the single mjpeg task, so no locking. */
static uint16_t s_agc_lo, s_agc_hi;
static bool     s_agc_valid;

/*
 * Histogram subsample stride. The histogram exists only to locate two
 * percentiles, and a percentile does not need every pixel — at 384x288 every
 * 4th pixel is still 27 k samples across 1024 bins. The bins are 16 counts
 * wide, so the sampling error is far below one bin and the chosen [lo,hi] is
 * identical in practice, at a quarter of the cost. This is the expensive pass
 * (a strided read of the whole frame out of PSRAM); the mapping pass below
 * must stay at full rate because it writes every output pixel.
 *
 * Must not share a common factor with the image width, or the samples land in
 * a few fixed columns instead of spreading over the frame. 3 is coprime with
 * both 384 and 640.
 */
#define WEB_AGC_SAMPLE_STRIDE 3

static void agc_y16_to_gray(const uint16_t *src, uint8_t *dst, size_t pixels)
{
    uint32_t histo[WEB_AGC_BINS] = { 0 };
    uint32_t sampled = 0;
    for (size_t i = 0; i < pixels; i += WEB_AGC_SAMPLE_STRIDE) {
        histo[(src[i] & 0x3FFF) >> WEB_AGC_SHIFT]++;
        sampled++;
    }

    /* Walk in from both ends until the clipped quota of samples is consumed.
     * Quota is a fraction of the SAMPLED count, not of `pixels` — using the
     * full pixel count here would over-trim by the stride factor. */
    uint32_t quota = (sampled * WEB_CLIP_PERMILLE) / 1000u;
    uint32_t acc = 0;
    int lo_bin = 0, hi_bin = WEB_AGC_BINS - 1;
    for (; lo_bin < hi_bin; lo_bin++) {
        acc += histo[lo_bin];
        if (acc > quota) break;
    }
    uint32_t lo_below = acc - histo[lo_bin];    /* samples strictly below this bin */

    acc = 0;
    for (; hi_bin > lo_bin; hi_bin--) {
        acc += histo[hi_bin];
        if (acc > quota) break;
    }
    uint32_t hi_above = acc - histo[hi_bin];    /* samples strictly above this bin */

    /*
     * Interpolate WITHIN the boundary bin, so lo/hi come out as real sensor
     * counts rather than bin edges.
     *
     * This is what makes the result match robust_range() in cam_viewer.py,
     * which is np.percentile(frame14, 0.5 / 99.5) — and np.percentile
     * interpolates linearly between order statistics. Taking the raw bin edges
     * instead (lo = bin<<4, hi = bin<<4 | 15) quantised the window to 16
     * counts and always rounded OUTWARD, so the firmware window came out up to
     * 32 counts wider than the viewer's. On this sensor the scene occupies a
     * few hundred counts around the ~7000 baseline, so that was a >10%
     * contrast loss against the viewer on the same frame.
     *
     * The cost is two divides PER FRAME — everything here is arithmetic on
     * counters the histogram walk already produced. The per-pixel work is
     * untouched, so this is free in the sense that matters.
     *
     * Both fractions are guaranteed < 1: each loop broke on
     * acc > quota, i.e. quota - (acc - histo[bin]) < histo[bin], so the
     * interpolated value cannot escape its own bin.
     */
#define WEB_AGC_BIN_SPAN (1u << WEB_AGC_SHIFT)
    uint32_t lo_val = (uint32_t)lo_bin << WEB_AGC_SHIFT;
    if (histo[lo_bin]) {
        lo_val += ((quota - lo_below) * WEB_AGC_BIN_SPAN) / histo[lo_bin];
    }
    /* hi walks DOWNWARD from the top of its bin, mirroring the descending loop. */
    uint32_t hi_val = ((uint32_t)hi_bin << WEB_AGC_SHIFT) + WEB_AGC_BIN_SPAN - 1u;
    if (histo[hi_bin]) {
        hi_val -= ((quota - hi_above) * WEB_AGC_BIN_SPAN) / histo[hi_bin];
    }

    uint16_t lo = (uint16_t)lo_val;
    uint16_t hi = (uint16_t)hi_val;
    if (hi <= lo) {                         /* flat frame - avoid a divide by zero */
        memset(dst, 0, pixels);
        s_agc_valid = false;                /* don't smooth across a dropout */
        return;
    }

    /*
     * Temporal smoothing of the window.
     *
     * The window is re-derived from scratch every frame, so anything that
     * perturbs one frame's histogram moves the whole tone curve for exactly
     * that frame and then snaps back — which is what "the AGC changes contrast
     * for 1..2 frames" looks like. Sensor noise alone wobbles the endpoints by
     * a bin or two; a real transient (an FFC, a hand entering the scene, a
     * capture the phase classifier let through) throws them much further.
     *
     * A one-pole IIR over lo and hi keeps the steady-state window exactly
     * where the percentiles put it, but spreads any step over several frames
     * so a single odd frame can no longer flash the whole image. At 1/4 weight
     * a genuine scene change still settles in ~5 frames (~200 ms at 25 fps),
     * which reads as responsive.
     *
     * The window is reset rather than smoothed when it moves further than
     * AGC_JUMP counts — that is a resolution switch or a phase change, not
     * drift, and easing into it would show a few badly-scaled frames.
     */
#define AGC_JUMP 2048
    if (s_agc_valid &&
        (uint32_t)abs((int)lo - (int)s_agc_lo) < AGC_JUMP &&
        (uint32_t)abs((int)hi - (int)s_agc_hi) < AGC_JUMP) {
        lo = (uint16_t)((s_agc_lo * 3u + lo) / 4u);
        hi = (uint16_t)((s_agc_hi * 3u + hi) / 4u);
    }
    s_agc_lo    = lo;
    s_agc_hi    = hi;
    s_agc_valid = true;

    if (hi <= lo) {                         /* smoothing collapsed the window */
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
    const uint16_t *src = NULL;
    void *hold = NULL;          /* non-NULL = we hold a DVP buffer to release */

    /*
     * Path selection is a straight ASK, not a probe.
     *
     *   UVC active   — a primary consumer signals the tap ~25 times/s, so
     *     camera_tap_borrow() returns almost immediately. A direct claim here
     *     would lose the buffer race against the UVC task (prio 23 vs our 5)
     *     and deliver zero frames — see the tap rationale in camera_pipeline.c.
     *   UVC idle     — nothing ever signals the tap, so claim directly. That
     *     is uncontended by definition in this case.
     *
     * This used to guess, by probing the tap for WEB_TAP_PROBE_MS and counting
     * misses, and that guessing was itself a major source of the uneven frame
     * rate: with no UVC host, every WEB_TAP_RETRY_EVERY (25) frames it spent a
     * full 120 ms blocked on a tap that nothing was ever going to fill, just to
     * re-check whether UVC had come online. One dead 120 ms window per ~25
     * frames is a visible hitch about once a second, plus 5 x 120 ms of dead
     * probing at the start of every stream.
     *
     * uvc_stream_is_active() answers the same question for free — it is
     * exactly "a host has accepted a frame in the last 500 ms", i.e. "someone
     * is calling camera_get_frame() at priority 23 right now". If it flips
     * between the check and the call we lose one frame and correct on the
     * next, which is the same cost the probe paid every single time.
     */
    if (uvc_stream_is_active()) {
        src = camera_tap_borrow(cam, &w, &h, WEB_TAP_PROBE_MS);
    } else {
        src = camera_get_frame(cam, &hold, WEB_FRAME_TIMEOUT_MS);
        w = cam->width;
        h = cam->height;
    }
    if (!src) {
        return 0;
    }

    size_t pixels = (size_t)w * h;
    if (!pixels || pixels > WEB_MAX_PIXELS) {
        if (hold) camera_release_frame(cam, hold);
        ESP_LOGE(TAG, "bad frame geometry %"PRIu32"x%"PRIu32, w, h);
        return 0;
    }

    /*
     * AGC reads `src` in place — no staging copy in either path.
     *
     * Direct path: we AGC straight out of the DVP buffer and release it
     * immediately after, rather than memcpy'ing it to a private buffer first.
     * Holding it for the duration of one AGC pass is safe for the pool: the
     * worst case is 1 WRITING + 2 READY + 1 UVC READING + 1 us = 5 of 6, which
     * still leaves the FREE buffer that on_get_new_trans needs.
     *
     * Tap path: camera_tap_borrow() hands back a pointer into the tap buffer
     * instead of copying it out.
     *
     * Between them that removes one full-frame PSRAM->PSRAM copy per web
     * frame (614 KB at 640x480) and the whole s_web.y16 staging buffer.
     */
    agc_y16_to_gray(src, s_web.gray, pixels);

    if (hold) {
        camera_release_frame(cam, hold);     /* AGC was the last read of src */
    }

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
    /*
     * Single-page viewer. Rendering pipeline in the browser:
     *
     *   <img src="/stream" hidden>  --drawImage-->  <canvas>
     *   <canvas>  --getImageData -> LUT -> putImageData-->  visible frame
     *
     * The MJPEG server ships an 8-bit grey JPEG (see agc_y16_to_gray) and the
     * canvas re-colours it with one of a few 256-entry palettes plus an
     * optional inversion. Grayscale-no-invert is fast-pathed to just display
     * the img directly, so on that default the browser does zero pixel work.
     *
     * The MJPEG socket stays open forever, so polls here are deliberately slow
     * (500 ms for /log, 1 s for /stats): httpd has only a handful of sockets
     * and a fast poll can starve the stream that is reporting on it.
     */
    static const char page[] =
"<!doctype html><meta charset=utf-8>"
"<meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>P4 Thermal</title>"
"<style>"
"body{margin:0;background:#111;color:#ccc;font:14px system-ui;"
"display:flex;flex-direction:column;align-items:center;gap:10px;padding:12px}"
"h3{margin:0}"
/* No hidden <img> here — even at opacity 0 or off-screen, Chrome/Firefox
   throttle the multipart decode and drawImage keeps returning frame #1.
   The stream is consumed via fetch()+ReadableStream in JS and each JPEG is
   turned into an ImageBitmap manually; see drawStream() below. */
"#viewwrap{width:min(96vw,768px);border:1px solid #333;line-height:0}"
"#view{width:100%;image-rendering:pixelated;display:block}"
".row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;"
"width:min(96vw,768px);font:12px ui-monospace,monospace}"
".row label{display:flex;gap:4px;align-items:center}"
"select,input,button{background:#1a1a1a;color:#eee;border:1px solid #333;"
"padding:4px 6px;font:12px ui-monospace,monospace;border-radius:3px}"
"button{cursor:pointer}"
"button:hover{background:#242424}"
"#cmdform{display:flex;gap:6px;width:100%}"
"#cmd{flex:1}"
".dpad{display:grid;grid-template-columns:repeat(3,60px);gap:4px;"
"margin:6px 0;justify-content:start}"
".dpad button{padding:8px 4px;min-height:38px}"
".dpad .sp{background:transparent;border:none;pointer-events:none}"
".bitrow{display:flex;gap:6px;margin:6px 0}"
".bitrow button{flex:1;padding:8px}"
"#sbtn{width:184px;padding:6px;margin-bottom:4px}"
".hint{font:11px ui-monospace,monospace;color:#888;margin:2px 0 10px;"
"white-space:pre-line}"
"#modev{color:#7ac8ff;font:12px ui-monospace,monospace}"
"#log{width:min(96vw,768px);height:180px;overflow-y:auto;background:#0a0a0a;"
"border:1px solid #222;padding:6px 8px;font:12px ui-monospace,monospace;"
"color:#9adf9a;border-radius:3px;white-space:pre-wrap;box-sizing:border-box}"
"#log .tx{color:#ffa34d}"
"#log .sys{color:#7ac8ff}"
"#s{display:grid;grid-template-columns:auto 1fr auto;gap:4px 10px;"
"width:min(96vw,768px);font:12px ui-monospace,monospace}"
".b{background:#222;border-radius:3px;overflow:hidden;height:14px}"
".f{height:100%;background:#ffa34d;width:0;transition:width .3s}"
".v{text-align:right;color:#eee;white-space:nowrap}"
"details{width:min(96vw,768px)}"
"summary{cursor:pointer;padding:4px 0;color:#eee}"
"</style>"

"<h3>ESP32-P4 thermal</h3>"

"<div id=viewwrap>"
"<canvas id=view width=384 height=288></canvas>"
"</div>"

"<div class=row>"
"<label>Palette "
"<select id=pal>"
"<option value=grayscale>Grayscale</option>"
"<option value=inferno>Inferno</option>"
"<option value=magma>Magma</option>"
"<option value=hot>Hot</option>"
"<option value=jet>Jet</option>"
"<option value=viridis>Viridis</option>"
"<option value=turbo>Turbo</option>"
"</select></label>"
"<label><input type=checkbox id=inv> Invert</label>"
"<span id=fv style='margin-left:auto'>-</span>"
"</div>"

"<details open><summary>Camera console <span id=modev>(mode: 14-bit thermal)</span></summary>"

/* D-pad — the same "control cross" the Python viewer draws (see
   cam_viewer.py:638). Menu keys S and M auto-send BIT,8 first when the
   camera is in 14-bit mode, because the on-screen menu is only drawn into
   the 8-bit video phase. */
"<div class=dpad>"
"<button class=sp></button>"
"<button data-key='+'>&#9650; +</button>"
"<button class=sp></button>"
"<button data-key='F'>&#9664; F</button>"
"<button data-key='C'>C</button>"
"<button data-key='M'>M &#9654;</button>"
"<button class=sp></button>"
"<button data-key='-'>&#9660; -</button>"
"<button class=sp></button>"
"</div>"
"<button id=sbtn data-key='S'>S &mdash; system menu</button>"
"<div class=hint>Passwords (enter on the cross):\n"
"  System menu:  + - M C + -\n"
"  Other menus:  + - + - + -</div>"

/* Phase switch — matches the Python viewer's two-button row. */
"<div class=bitrow>"
"<button data-cmd='BIT,8'>8-bit (menu)</button>"
"<button data-cmd='BIT,14'>14-bit thermal</button>"
"</div>"

/* Resolution switch. When a py viewer is attached it listens for RES,W,H on
   the CDC port and renegotiates UVC — otherwise the P4 applies the new size
   locally so the web MJPEG picks it up on the next frame. Wrong size vs the
   physical camera model produces garbage frames (see CLAUDE.md). */
"<div class=bitrow>"
"<button data-cmd='RES,384,288'>384 x 288</button>"
"<button data-cmd='RES,640,480'>640 x 480</button>"
"</div>"

"<form id=cmdform autocomplete=off>"
"<input id=cmd placeholder='CMD or CMD,VAL  (e.g. GCO, KBD,C, SSM,1)' spellcheck=false>"
"<button type=submit>Send</button>"
"</form>"
"<div id=log></div>"
"<div class=hint>BIT/PCK are handled on the P4; anything else goes to the camera UART.</div>"
"</details>"

"<details><summary>System stats</summary>"
"<div id=s>"
"<span>CPU0</span><div class=b><div class=f id=c0></div></div><span class=v id=c0v>-</span>"
"<span>CPU1</span><div class=b><div class=f id=c1></div></div><span class=v id=c1v>-</span>"
"<span>Internal</span><div class=b><div class=f id=ib></div></div><span class=v id=iv>-</span>"
"<span>PSRAM</span><div class=b><div class=f id=pb></div></div><span class=v id=pv>-</span>"
"<span>Uptime</span><div></div><span class=v id=upv>-</span>"
"</div>"
"</details>"

"<p style='font-size:12px;color:#888'>"
"Raw 14-bit thermal is on the USB UVC stream; this view is the 8-bit AGC preview.</p>"

"<script>"
/* ---------- Palettes: match cam_viewer.py (OpenCV colormaps, RGB order). ----
   Piecewise linear over ~11 control points per palette — enough to be
   visually indistinguishable from the OpenCV output at 384x288 preview. */
"const PAL={"
"grayscale:[[0,0,0,0],[1,255,255,255]],"
"inferno:["
"[0,0,0,4],[.1,17,11,55],[.2,56,11,96],[.3,95,20,111],[.4,133,32,110],"
"[.5,169,46,100],[.6,204,62,82],[.7,232,89,55],[.8,249,128,20],"
"[.9,250,179,20],[1,252,255,164]],"
"magma:["
"[0,0,0,4],[.1,17,11,57],[.2,52,15,105],[.3,91,22,128],[.4,129,36,129],"
"[.5,167,52,122],[.6,204,72,111],[.7,236,100,100],[.8,253,141,106],"
"[.9,254,189,131],[1,252,253,191]],"
/* OpenCV COLORMAP_HOT: black -> red @0.375 -> yellow @0.75 -> white. */
"hot:[[0,0,0,0],[.375,255,0,0],[.75,255,255,0],[1,255,255,255]],"
"jet:["
"[0,0,0,128],[.11,0,0,255],[.36,0,255,255],[.5,128,255,128],"
"[.64,255,255,0],[.89,255,0,0],[1,128,0,0]],"
"viridis:["
"[0,68,1,84],[.1,72,35,116],[.2,64,67,135],[.3,52,94,141],[.4,41,121,142],"
"[.5,32,144,141],[.6,34,167,132],[.7,68,190,112],[.8,121,209,81],"
"[.9,189,222,38],[1,253,231,37]],"
"turbo:["
"[0,48,18,59],[.1,64,67,168],[.2,67,111,219],[.3,39,155,232],[.4,30,195,202],"
"[.5,89,220,138],[.6,177,234,66],[.7,238,208,47],[.8,250,149,34],"
"[.9,219,84,15],[1,122,4,3]]"
"};"
"function buildLut(stops,invert){"
"const out=new Uint8ClampedArray(768);"
"for(let i=0;i<256;i++){"
"const t=invert?1-i/255:i/255;"
"let a=stops[0],b=stops[stops.length-1];"
"for(let k=0;k<stops.length-1;k++){"
"if(t>=stops[k][0]&&t<=stops[k+1][0]){a=stops[k];b=stops[k+1];break;}"
"}"
"const f=(b[0]===a[0])?0:(t-a[0])/(b[0]-a[0]);"
"out[i*3]=a[1]+f*(b[1]-a[1]);"
"out[i*3+1]=a[2]+f*(b[2]-a[2]);"
"out[i*3+2]=a[3]+f*(b[3]-a[3]);"
"}"
"return out;"
"}"

/* ---------- Render: consume /stream directly via fetch() ------------------ */
/* Using an <img src=/stream> and drawImage() does not work: at opacity 0,
   display:none, or off-screen, Chromium/Firefox throttle the multipart
   decoder and drawImage returns the same first frame forever while the fps
   counter (which just counts JPEGs the server sends) keeps climbing.
   Fetching /stream ourselves and parsing the multipart boundaries in JS
   bypasses that entirely and pins the decode to the tab's lifetime. */
"const cvs=document.getElementById('view');"
"const cctx=cvs.getContext('2d',{willReadFrequently:true});"
"let curName='grayscale',invert=false;"
"let curLut=buildLut(PAL[curName],invert);"
"document.getElementById('pal').addEventListener('change',e=>{"
"curName=e.target.value;curLut=buildLut(PAL[curName],invert);"
"});"
"document.getElementById('inv').addEventListener('change',e=>{"
"invert=e.target.checked;curLut=buildLut(PAL[curName],invert);"
"});"
"function drawFrame(bmp){"
"if(cvs.width!==bmp.width||cvs.height!==bmp.height){"
"cvs.width=bmp.width;cvs.height=bmp.height;"
"}"
"cctx.drawImage(bmp,0,0);"
/* Fast path: grayscale + no invert = the raw JPEG already IS what we want. */
"if(curName!=='grayscale'||invert){"
"const img=cctx.getImageData(0,0,cvs.width,cvs.height);"
"const d=img.data,lut=curLut;"
"for(let i=0;i<d.length;i+=4){"
"const g=d[i];"
"d[i]=lut[g*3];d[i+1]=lut[g*3+1];d[i+2]=lut[g*3+2];"
"}"
"cctx.putImageData(img,0,0);"
"}"
"}"
/* Boundary matches the server side (BOUNDARY macro in web_stream.c). */
"const BND=new TextEncoder().encode('\\r\\n--thermalframe\\r\\n');"
"const HDR_END=new TextEncoder().encode('\\r\\n\\r\\n');"
"function findSeq(hay,needle,from){"
"outer:for(let i=from;i<=hay.length-needle.length;i++){"
"for(let j=0;j<needle.length;j++)if(hay[i+j]!==needle[j])continue outer;"
"return i;"
"}"
"return -1;"
"}"
"function concat(a,b){"
"const c=new Uint8Array(a.length+b.length);"
"c.set(a);c.set(b,a.length);return c;"
"}"
/* Stall watchdog: if no JPEG has been decoded in STALL_MS, abort the fetch so
   the outer loop reconnects. Covers three failure shapes seen in the wild:
   (1) TCP half-open where read() never resolves, (2) server stuck between
   frames, (3) Wi-Fi drop mid-stream that leaves the socket dangling. */
"const STALL_MS=2500;"
"let lastFrameTs=performance.now();"
"async function drawStream(){"
"const ac=new AbortController();"
"const wd=setInterval(()=>{"
"if(performance.now()-lastFrameTs>STALL_MS)ac.abort();"
"},500);"
"try{"
"const resp=await fetch('/stream',{cache:'no-store',signal:ac.signal});"
"if(!resp.body)throw new Error('no stream body');"
"const reader=resp.body.getReader();"
"const td=new TextDecoder();"
"let buf=new Uint8Array(0);"
"lastFrameTs=performance.now();"
"while(true){"
"const {value,done}=await reader.read();"
"if(done)break;"
"buf=concat(buf,value);"
"for(;;){"
"const b0=findSeq(buf,BND,0);"
"if(b0<0)break;"
"const hs=b0+BND.length;"
"const he=findSeq(buf,HDR_END,hs);"
"if(he<0)break;"
"const headers=td.decode(buf.subarray(hs,he));"
"const m=headers.match(/Content-Length:\\s*(\\d+)/i);"
"if(!m){buf=buf.subarray(he+4);continue;}"
"const len=+m[1];"
"const bs=he+4,be=bs+len;"
"if(buf.length<be)break;"
"const jpeg=buf.slice(bs,be);"
"buf=buf.subarray(be);"
"try{"
"const bmp=await createImageBitmap(new Blob([jpeg],{type:'image/jpeg'}));"
"drawFrame(bmp);bmp.close();"
"lastFrameTs=performance.now();"
"}catch(e){}"
"}"
/* Keep the working buffer from growing unbounded on partial matches (shouldn't
   happen with a healthy stream, but be safe). */
"if(buf.length>200000)buf=buf.slice(-100000);"
"}"
"}finally{clearInterval(wd);}"
"}"
"(async function loop(){"
"while(true){"
"try{await drawStream();}catch(e){}"
"await new Promise(r=>setTimeout(r,500));"
"}"
"})();"

/* ---------- Camera console ------------------------------------------------ */
"let logSeq=0;"
/* Track the camera's delivered phase so menu keys (S/M) can auto-send BIT,8
   first — the on-screen menu is only drawn into the 8-bit video phase.
   14-bit is the firmware boot default (see camera_open() in camera_pipeline.c). */
"let webMode='14';"
"const modev=document.getElementById('modev');"
"function setMode(m){"
"webMode=m;"
"modev.textContent='(mode: '+(m==='8'?'8-bit video (menu)':'14-bit thermal')+')';"
"}"
"function checkModeFromLine(line){"
/* Both the TX echo ('TX -> BIT,8,') and the local ack ('BIT,8 (video phase)')
   flow through the log, so matching the digits after 'BIT,' catches both. */
"const m=line.match(/BIT,(8|14)/);"
"if(m)setMode(m[1]);"
"}"
"const logEl=document.getElementById('log');"
"function addLog(text){"
"const d=document.createElement('div');"
"if(text.startsWith('TX ->'))d.className='tx';"
"else if(text.startsWith('BIT')||text.startsWith('PCK'))d.className='sys';"
"d.textContent=text;"
"logEl.appendChild(d);"
"while(logEl.childElementCount>200)logEl.removeChild(logEl.firstChild);"
"logEl.scrollTop=logEl.scrollHeight;"
"checkModeFromLine(text);"
"}"
"async function pollLog(){"
"try{"
"const r=await fetch('/log?since='+logSeq);"
"const j=await r.json();"
"if(j.lines)for(const l of j.lines)addLog(l);"
"if(j.seq)logSeq=j.seq;"
"}catch(e){}"
"}"
"setInterval(pollLog,500);pollLog();"
"async function sendCmd(text){"
"if(!text)return;"
"try{await fetch('/cmd?c='+encodeURIComponent(text));}catch(e){}"
"pollLog();"
"}"
/* Menu keys need the 8-bit phase to actually see the OSD. Await the BIT,8
   so the camera has a moment to switch before the KBD press lands. */
"async function sendKey(k){"
"if((k==='S'||k==='M')&&webMode!=='8'){"
"await sendCmd('BIT,8');"
"setMode('8');"
"await new Promise(r=>setTimeout(r,60));"
"}"
"await sendCmd('KBD,'+k);"
"}"
"document.querySelectorAll('[data-key]').forEach(b=>{"
"b.addEventListener('click',()=>sendKey(b.dataset.key));"
"});"
"document.querySelectorAll('[data-cmd]').forEach(b=>{"
"b.addEventListener('click',()=>sendCmd(b.dataset.cmd));"
"});"
"document.getElementById('cmdform').addEventListener('submit',e=>{"
"e.preventDefault();"
"const inp=document.getElementById('cmd');"
"sendCmd(inp.value);inp.value='';"
"});"

/* ---------- Stats poll ---------------------------------------------------- */
"const kb=b=>b>1048576?(b/1048576).toFixed(1)+' MB':(b/1024).toFixed(0)+' kB';"
"function set(bar,val,pct,txt){document.getElementById(bar).style.width=pct+'%';"
"document.getElementById(val).textContent=txt;}"
"async function updateStats(){try{const r=await fetch('/stats');const d=await r.json();"
"set('c0','c0v',d.cpu0,d.cpu0.toFixed(0)+'%');"
"set('c1','c1v',d.cpu1,d.cpu1.toFixed(0)+'%');"
"const iu=d.int_total-d.int_free,pu=d.psram_total-d.psram_free;"
"set('ib','iv',100*iu/d.int_total,kb(iu)+' / '+kb(d.int_total)+'  (min free '+kb(d.int_min)+')');"
"set('pb','pv',100*pu/d.psram_total,kb(pu)+' / '+kb(d.psram_total)+'  (min free '+kb(d.psram_min)+')');"
"document.getElementById('upv').textContent=d.uptime+' s';"
"document.getElementById('fv').textContent=d.fps.toFixed(1)+' fps';"
"}catch(e){}}"
"setInterval(updateStats,1000);updateStats();"
"</script>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

/* ---------------------------------------------------------- UART bridge */

/* URL-decode `src` in place into `dst` (dst may equal src). Truncates at
 * cap-1 bytes and NUL-terminates. */
static void url_decode(const char *src, char *dst, size_t cap)
{
    size_t o = 0;
    for (const char *p = src; *p && o + 1 < cap; ) {
        if (*p == '+') { dst[o++] = ' '; p++; continue; }
        if (*p == '%' && p[1] && p[2]) {
            char h[3] = { p[1], p[2], 0 };
            char *end = NULL;
            unsigned v = (unsigned)strtoul(h, &end, 16);
            if (end == h + 2) {
                dst[o++] = (char)v;
                p += 3;
                continue;
            }
        }
        dst[o++] = *p++;
    }
    dst[o] = '\0';
}

/* GET /cmd?c=<CMD[,VAL]>  — forward a command to the camera. Reuses the same
 * parser the CDC console uses, so BIT/PCK still work locally. */
static esp_err_t cmd_handler(httpd_req_t *req)
{
    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "missing ?c=<command>");
    }
    char raw[80];
    if (httpd_query_key_value(query, "c", raw, sizeof(raw)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "missing ?c=<command>");
    }
    char line[80];
    url_decode(raw, line, sizeof(line));

    esp_err_t err = camera_uart_submit_line(line);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char body[64];
    int n = snprintf(body, sizeof(body), "{\"ok\":%s,\"err\":%d}",
                     err == ESP_OK ? "true" : "false", (int)err);
    return httpd_resp_send(req, body, n);
}

/* GET /log?since=<seq>  — poll for new UART log lines. */
static esp_err_t log_handler(httpd_req_t *req)
{
    uint32_t since = 0;
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char s[16];
        if (httpd_query_key_value(query, "since", s, sizeof(s)) == ESP_OK) {
            since = (uint32_t)strtoul(s, NULL, 10);
        }
    }
    /* Size the buffer generously: 32 lines * up to ~2 chars/byte after JSON
     * escaping + quotes/commas ~= 8 KB max. */
    static char buf[8192];
    size_t len = camera_uart_log_snapshot(since, buf, sizeof(buf));
    if (len == 0) {
        strcpy(buf, "{\"seq\":0,\"lines\":[]}");
        len = strlen(buf);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, len);
}

/* --------------------------------------------------------------- routes */

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

    /*
     * Anti-stutter socket tuning for the streaming client.
     *
     *   TCP_NODELAY: httpd_resp_send_chunk() sends the multipart header and the
     *     JPEG body as two separate calls. With Nagle on, the second send is
     *     coalesced/delayed until the first is ACKed — under Wi-Fi jitter that
     *     turns into a ~40 ms wait per frame that reads as a hitch even with
     *     good RSSI. Nodelay removes it.
     *
     *   SO_SNDTIMEO: a browser that closes its tab but leaves the TCP FIN
     *     unread (or a Wi-Fi stall of many seconds) would otherwise block the
     *     stream task inside send() forever, holding the buffers and the
     *     client_busy flag. A ~5 s cap turns those into a clean disconnect.
     */
    int fd = httpd_req_to_sockfd(req);
    if (fd >= 0) {
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    ESP_LOGI(TAG, "stream client connected");

    s_agc_valid = false;   /* start this client's AGC from its own first frame */

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

    /* The pipeline's tap storage. We AGC straight out of it (camera_tap_borrow
     * lends a pointer), so there is no second staging buffer on this side. */
    ESP_RETURN_ON_ERROR(camera_tap_init(cam, (size_t)WEB_MAX_PIXELS * 2), TAG, "tap init");

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
        { .uri = "/cmd",      .method = HTTP_GET, .handler = cmd_handler      },
        { .uri = "/log",      .method = HTTP_GET, .handler = log_handler      },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG,
                            "register %s", routes[i].uri);
    }

    ESP_LOGI(TAG, "HTTP server up on :80  (/ = viewer, /stream = MJPEG, /snapshot = one JPEG)");
    return ESP_OK;
}
