/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * DVP capture for a 14-bit monochrome thermal camera that emits two
 * interleaved frame phases at 50 Hz (real content rate 25 Hz):
 *   - EVEN captures: 8-bit visible-light video (value in the low 8 bits)
 *   - ODD  captures: 14-bit thermal data     (value in bits [13:0])
 * The device drives PCLK, HSYNC, VSYNC — ESP32-P4 is purely a receiver.
 * Data pins D0–D13 carry the sample; D14/D15 are tied to GND.  Each captured
 * 16-bit word is therefore already Y16 (bits[15:14] always 0).
 *
 * Capture uses ISR callbacks (on_get_new_trans / on_trans_finished) feeding a
 * small pool of DMA buffers guarded by a spinlock — there is no capture task.
 * A content-based parity filter delivers only the 14-bit thermal phase (fixed:
 * parity_filter=true, keep_parity=1), giving a steady ~25 fps; the 8-bit video
 * phase is discarded. This selection is hardwired in camera_open().
 */

#pragma once

#include "esp_err.h"
#include "esp_cam_ctlr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * SIX buffers: the driver calls on_get_new_trans (start the NEXT capture)
 * BEFORE on_trans_finished (publish the just-finished one), so at that instant
 * up to five buffers are spoken for — one being READ by the UVC task, up to
 * TWO published READY (see below), and the just-finished (still WRITING) —
 * and one more must be FREE to hand to the DMA.
 *
 * Two READY slots (not one) are required by the interleaved source: phases
 * alternate every 20 ms, so with a single READY slot the 8-bit video capture
 * evicts the thermal frame 20 ms after it lands. If the consumer's per-frame
 * cycle (cache msync + classify + memcpy + USB bulk xfer) exceeds 20 ms — as
 * it does at 640x480 — it misses every other thermal frame and output halves
 * to 12.5 fps. With two READY slots the thermal frame survives the intervening
 * video capture, giving the consumer the full 40 ms phase period.
 */
#define DVP_BUFFER_COUNT  6

/* Max frames published READY at once; oldest is recycled beyond this. */
#define DVP_READY_SLOTS   2

/*
 * Capture MORE lines than the active image. If the DMA buffer is sized to
 * exactly one frame (width×height×2), the GDMA descriptor chain fills and ENDS
 * at the last active line — before the VSYNC EOF (cam_vs_eof_en=1) fires — so
 * the EOF lands with no active descriptor. That race is the root of the
 * intermittent freeze and the diagonal frame-start shear. Sizing the DMA chain
 * taller than any possible per-frame data means it never fills early, so VSYNC
 * is the ONLY frame delimiter and every frame starts cleanly at the VSYNC
 * boundary. The delivered Y16 frame is still the first width×height pixels.
 * (Matters most when there's little vertical blanking — e.g. the 640×480 sensor
 * packs 480 active lines into ~19.95 ms of its 20 ms VSYNC period.)
 */
#define DVP_VBLANK_CAPTURE_MARGIN  64   /* extra capture lines past the active image */

/* Per-buffer ownership state. Transitions happen only under ctx->lock. */
enum { BUF_FREE = 0, BUF_WRITING, BUF_READY, BUF_READING };

typedef struct {
    esp_cam_ctlr_handle_t ctlr;
    void          *bufs[DVP_BUFFER_COUNT]; /* DMA frame buffers in PSRAM */
    volatile uint8_t  state[DVP_BUFFER_COUNT];
    volatile uint32_t buf_seq[DVP_BUFFER_COUNT]; /* capture seq tag per buffer */
    volatile uint32_t buf_recv[DVP_BUFFER_COUNT]; /* bytes actually DMA'd into this buffer */
    size_t         buf_size;               /* DMA capture size = w * cap_height * 2 */
    size_t         frame_size;             /* delivered Y16 frame = w * h * 2 */
    uint32_t       width;
    uint32_t       height;                 /* active/delivered image height */
    uint32_t       cap_height;             /* DMA capture height (= height + margin) */

    portMUX_TYPE      lock;
    SemaphoreHandle_t frame_sem;

    /* Interleaved-source parity filter (the A/B switch). */
    volatile uint32_t cap_seq;             /* ++ per completed capture */
    volatile bool     parity_filter;       /* true = deliver one phase only */
    volatile uint32_t keep_parity;         /* deliver captures with seq&1 == this */

    /* Framing signal config — frames complete on the VSYNC edge, so polarity
     * here is what makes captures complete at all (see camera_set_vsync_invert). */
    volatile bool     vsync_invert;        /* GPIO-matrix inversion on VSYNC */
    volatile bool     hsync_invert;        /* GPIO-matrix inversion on HSYNC */
    volatile bool     pclk_invert;         /* GPIO-matrix inversion on PCLK (sampling edge) */
    volatile int      de_mode;             /* 0=HSYNC HREF, 1=HSYNC inverted, 2=free-run */

    /* Debug counters incremented in ISR — read from task on timeout. */
    volatile uint32_t dbg_get_new_calls;
    volatile uint32_t dbg_finished;
    volatile uint32_t dbg_no_free;
    volatile uint32_t dbg_recv_size;   /* received_size of the last completed frame */

    /* Freeze watchdog: a thermal-cam FFC (or any PCLK/VSYNC glitch) can wedge
     * the CAM so it keeps EOF-ing but only redelivers the few stale capture
     * buffers (their hashes cycle). A live sensor's noise makes every frame's
     * hash unique, so a hash that REPEATS one of the last few means we're
     * frozen — count those and auto-resync. */
    uint32_t       frz_ring[8];        /* recent frame hashes */
    uint32_t       frz_idx;            /* ring write index */
    uint32_t       frz_stale;          /* consecutive frames whose hash repeats */

    /* Hard-stall watchdog: the P4 CAM can stop issuing EOFs entirely after some
     * seconds (dbg_finished freezes, get_frame times out). Unlike a static
     * scene this means NO captures complete — recover with a stop/start. */
    uint32_t       stall_last_finished; /* dbg_finished snapshot at last timeout */
    uint32_t       stall_count;         /* consecutive timeouts with no progress */

    /* Observer tap: a copy of the last delivered frame, for viewers that must
     * not steal a buffer from the primary consumer (see camera_tap_get). */
    uint16_t         *tap_buf;
    size_t            tap_cap;
    volatile size_t   tap_bytes;
    volatile uint32_t tap_width;
    volatile uint32_t tap_height;
    volatile bool     tap_pending;
    SemaphoreHandle_t tap_sem;
} camera_ctx_t;

/* No-op; actual init is in camera_open. */
esp_err_t camera_init(void);

/* Create the DVP controller, allocate DMA buffers, register ISR callbacks. */
esp_err_t camera_open(camera_ctx_t *ctx);

/* Start DVP DMA. Must call camera_open first. */
esp_err_t camera_start(camera_ctx_t *ctx);

/* Stop DVP DMA and reset all buffers to FREE. */
esp_err_t camera_stop(camera_ctx_t *ctx);

/*
 * Block until the next frame of the selected parity is ready (up to
 * timeout_ms milliseconds).  Returns a pointer to the 16-bit DVP buffer
 * (Y16) or NULL on timeout.  *out_buf receives the opaque buffer pointer that
 * must be passed to camera_release_frame() when the consumer is done with it.
 */
const uint16_t *camera_get_frame(camera_ctx_t *ctx, void **out_buf,
                                  uint32_t timeout_ms);

/* Return a buffer obtained from camera_get_frame() for reuse. */
void camera_release_frame(camera_ctx_t *ctx, void *buf);

/* ---- Observer tap ------------------------------------------------------- */
/*
 * A second consumer calling camera_get_frame() does NOT work: the UVC task
 * (priority 23) calls straight back in after every transfer, so it claims each
 * published buffer before a lower-priority viewer is ever scheduled — the
 * viewer gets zero frames, not half. The tap instead copies the frame that the
 * primary consumer is already taking, so nothing is stolen.
 *
 * camera_tap_init() allocates the copy buffer (PSRAM). camera_tap_get() asks
 * for the next frame and blocks; it returns false on timeout, which is what
 * happens when NOTHING is consuming frames (e.g. no USB host attached) — in
 * that case the caller should fall back to camera_get_frame(), which is
 * uncontended precisely because there is no other consumer.
 */
esp_err_t camera_tap_init(camera_ctx_t *ctx, size_t max_bytes);
bool camera_tap_get(camera_ctx_t *ctx, uint16_t *dst, size_t dst_bytes,
                    uint32_t *out_w, uint32_t *out_h, uint32_t timeout_ms);

/* ---- Framing routing (fixed; applied once by camera_open) --------------- */
/*
 * These set the GPIO-matrix routing / CAM framing for the proven-working
 * configuration. They are called once from camera_open() with the hardwired
 * values (VSYNC inv=1, HSYNC inv=0, PCLK inv=0, DE mode 0) and are not exposed
 * as runtime controls.
 */
void camera_set_vsync_invert(camera_ctx_t *ctx, bool invert);
void camera_set_hsync_invert(camera_ctx_t *ctx, bool invert);
void camera_set_pclk_invert(camera_ctx_t *ctx, bool invert);
void camera_set_de_mode(camera_ctx_t *ctx, int mode);

/* Select the delivered interleaved phase: 1 = 14-bit thermal, 0 = 8-bit video
 * (the camera's on-screen menu is only drawn in the 8-bit phase). */
void camera_set_keep_parity(camera_ctx_t *ctx, uint32_t keep);

/* Set the delivered frame size (w*h*2 must fit the max-sized capture buffer).
 * Buffers are not reallocated; call while capture is stopped. */
esp_err_t camera_set_resolution(camera_ctx_t *ctx, uint32_t w, uint32_t h);
