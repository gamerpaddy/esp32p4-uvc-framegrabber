/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "camera_pipeline.h"
#include "sdkconfig.h"
#include "uvc_frame_config.h"   /* THERMAL_MAX_WIDTH / THERMAL_MAX_HEIGHT */
#include "esp_log.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_private/esp_cam_dvp.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_pins.h"      /* GPIO_MATRIX_CONST_ONE_INPUT */
#include "hal/cam_periph.h"     /* cam_periph_signals (vsync_sig/de_sig) */
#include "hal/cam_ll.h"         /* cam_ll_set_input_data_width (force 16-bit) */
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "thermal_dvp";

/* ---- ISR callbacks (must be in IRAM, no PSRAM access) ------------------- */

/*
 * Hand the DMA a FREE buffer for the next capture and mark it WRITING.
 * With DVP_BUFFER_COUNT buffers there should always be one free; if not we
 * skip the transaction safely (driver discards it, bk_buffer absorbs nothing).
 */
static bool IRAM_ATTR camera_get_new_trans_cb(esp_cam_ctlr_handle_t handle,
                                              esp_cam_ctlr_trans_t *trans,
                                              void *user_data)
{
    camera_ctx_t *ctx = (camera_ctx_t *)user_data;
    ctx->dbg_get_new_calls++;

    int b = -1;
    portENTER_CRITICAL_ISR(&ctx->lock);
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        if (ctx->state[i] == BUF_FREE) {
            b = i;
            ctx->state[i] = BUF_WRITING;
            break;
        }
    }
    portEXIT_CRITICAL_ISR(&ctx->lock);

    if (b < 0) {
        ctx->dbg_no_free++;
        trans->buffer = NULL;
        trans->buflen = 0;
        return false;
    }
    trans->buffer = ctx->bufs[b];
    trans->buflen = ctx->buf_size;
    return false;
}

/*
 * Publish the just-completed buffer as a READY frame.  Up to DVP_READY_SLOTS
 * frames stay published at once (newest-first); beyond that the OLDEST ready
 * frame is recycled.  Two slots let a thermal frame survive the interleaved
 * 8-bit video capture that completes 20 ms later — with a single slot the
 * video phase evicted it, halving delivered fps whenever the consumer's cycle
 * exceeded 20 ms.  Tags the capture with a sequence number so the consumer
 * can pick the newest frame and track age.
 * Returns true to wake a higher-priority task waiting on frame_sem.
 */
static bool IRAM_ATTR camera_trans_finished_cb(esp_cam_ctlr_handle_t handle,
                                               esp_cam_ctlr_trans_t *trans,
                                               void *user_data)
{
    camera_ctx_t *ctx = (camera_ctx_t *)user_data;
    BaseType_t hp_woken = pdFALSE;

    portENTER_CRITICAL_ISR(&ctx->lock);
    uint32_t seq = ++ctx->cap_seq;
    int ready_cnt = 0, oldest = -1;
    uint32_t oldest_seq = UINT32_MAX;
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        if (ctx->state[i] == BUF_READY) {
            ready_cnt++;
            if (ctx->buf_seq[i] < oldest_seq) {
                oldest_seq = ctx->buf_seq[i];
                oldest = i;
            }
        }
    }
    if (ready_cnt >= DVP_READY_SLOTS && oldest >= 0) {
        ctx->state[oldest] = BUF_FREE;      /* recycle the oldest ready frame */
    }
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        if (ctx->bufs[i] == trans->buffer) {
            ctx->state[i] = BUF_READY;      /* publish the freshly filled one */
            ctx->buf_seq[i] = seq;
            ctx->buf_recv[i] = trans->received_size;
            break;
        }
    }
    ctx->dbg_finished++;
    ctx->dbg_recv_size = trans->received_size;
    portEXIT_CRITICAL_ISR(&ctx->lock);

    xSemaphoreGiveFromISR(ctx->frame_sem, &hp_woken);
    return hp_woken == pdTRUE;
}

/* ---- Public API --------------------------------------------------------- */

esp_err_t camera_init(void)
{
    return ESP_OK;
}

esp_err_t camera_open(camera_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->width      = CONFIG_THERMAL_WIDTH;
    ctx->height     = CONFIG_THERMAL_HEIGHT;
    ctx->frame_size = ctx->width * ctx->height * 2;   /* delivered Y16 frame */
    /* Capture taller than the active image so the GDMA chain never fills before
     * VSYNC (see DVP_VBLANK_CAPTURE_MARGIN). Buffers are sized for the LARGEST
     * advertised frame so runtime RES,W,H switches don't overrun them; only the
     * fraction actually filled changes with the active resolution. */
    /* Size the CAM controller's expected v_res AND the DMA buffer to the tallest
     * advertised frame — v_res is a one-shot config on the DVP peripheral, so
     * switching to a larger frame at runtime with v_res still sized to the
     * smaller one truncates every line past the smaller height. */
    ctx->cap_height = THERMAL_MAX_HEIGHT + DVP_VBLANK_CAPTURE_MARGIN;
    ctx->buf_size   = THERMAL_MAX_WIDTH * ctx->cap_height * 2;
    portMUX_INITIALIZE(&ctx->lock);

    ctx->parity_filter = true;   /* always drop the 8-bit video phase */
    ctx->keep_parity   = 1u;     /* always deliver the 14-bit thermal phase */

    ctx->frame_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(ctx->frame_sem, ESP_ERR_NO_MEM, TAG, "frame_sem alloc failed");

    /*
     * Three-phase DVP init (workaround for the IDF 6.x driver limiting pin init
     * to 8 data lines while the ESP32-P4 hardware supports 16).
     *
     * Phase 1: call esp_cam_ctlr_dvp_init() with an 8-bit config — satisfies
     *   the driver's data_width==8 check, enables the peripheral RCC clock, and
     *   routes D[7:0] + VSYNC/DE/PCLK through the GPIO matrix.
     * Phase 2: manually route D[13:8] to GPIO-matrix signals IN8..IN13.
     * Phase 3: create the controller with pin_dont_init=1 and cam_data_width=16
     *   so the DMA samples full 16-bit words per PCLK.  D[15:14] are grounded.
     */

    /* Phase 1 */
    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            CONFIG_DVP_D0_GPIO, CONFIG_DVP_D1_GPIO,
            CONFIG_DVP_D2_GPIO, CONFIG_DVP_D3_GPIO,
            CONFIG_DVP_D4_GPIO, CONFIG_DVP_D5_GPIO,
            CONFIG_DVP_D6_GPIO, CONFIG_DVP_D7_GPIO,
        },
        .vsync_io = CONFIG_DVP_VSYNC_GPIO,
        .de_io    = CONFIG_DVP_HSYNC_GPIO,  /* HREF/HSYNC = data-enable */
        .pclk_io  = CONFIG_DVP_PCLK_GPIO,
        .xclk_io  = GPIO_NUM_NC,
    };
    ESP_RETURN_ON_ERROR(
        esp_cam_ctlr_dvp_init(0, CAM_CLK_SRC_DEFAULT, &pin_cfg),
        TAG, "DVP clock/GPIO init failed");

    /*
     * Set a valid CAM module clock divider. With external_xtal=1 the driver
     * SKIPS esp_cam_ctlr_dvp_output_clock(), leaving cam_clk_div_num=0 — an
     * undefined module clock. The controller then free-runs (sampling the bus
     * asynchronously instead of latching on the external PCLK), producing
     * thousands of garbage "frames"/sec that don't track the camera. Setting a
     * defined cam_clk (no XCLK pin is routed — xclk_io=NC) makes the CAM sample
     * correctly off the incoming PCLK. This only programs the divider; it does
     * not output a clock anywhere.
     */
    ESP_RETURN_ON_ERROR(
        esp_cam_ctlr_dvp_output_clock(0, CAM_CLK_SRC_DEFAULT, 40000000),
        TAG, "DVP cam_clk divider set failed");

    /* Phase 2 */
    static const struct { int gpio; int sig; } high_pins[] = {
        { CONFIG_DVP_D8_GPIO,  CAM_DATA_IN_PAD_IN8_IDX  },
        { CONFIG_DVP_D9_GPIO,  CAM_DATA_IN_PAD_IN9_IDX  },
        { CONFIG_DVP_D10_GPIO, CAM_DATA_IN_PAD_IN10_IDX },
        { CONFIG_DVP_D11_GPIO, CAM_DATA_IN_PAD_IN11_IDX },
        { CONFIG_DVP_D12_GPIO, CAM_DATA_IN_PAD_IN12_IDX },
        { CONFIG_DVP_D13_GPIO, CAM_DATA_IN_PAD_IN13_IDX },
    };
    for (int i = 0; i < (int)(sizeof(high_pins) / sizeof(high_pins[0])); i++) {
        ESP_RETURN_ON_ERROR(gpio_set_direction(high_pins[i].gpio, GPIO_MODE_INPUT),
                            TAG, "D%d direction failed", i + 8);
        ESP_RETURN_ON_ERROR(gpio_set_pull_mode(high_pins[i].gpio, GPIO_FLOATING),
                            TAG, "D%d pull_mode failed", i + 8);
        esp_rom_gpio_connect_in_signal(high_pins[i].gpio, high_pins[i].sig, false);
    }

    /* Phase 3 */
    esp_cam_ctlr_dvp_config_t dvp_cfg = {
        .ctlr_id                = 0,
        .clk_src                = CAM_CLK_SRC_DEFAULT,
        .h_res                  = THERMAL_MAX_WIDTH,   /* sized for the biggest advertised frame */
        .v_res                  = ctx->cap_height,     /* max height + vblank margin */
        .input_data_color_type  = CAM_CTLR_COLOR_RGB565,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .cam_data_width         = 16,
        .dma_burst_size         = 64,
        .bk_buffer_dis          = 1,   /* we supply DVP_BUFFER_COUNT buffers */
        /*
         * NO byte swap. The 16-bit DMA word is already native Y16: D0-D13 carry
         * the thermal value, D14/D15 are grounded, so the word is the thermal
         * count in natural little-endian order. Swapping the bytes (a) corrupts
         * the Y16 the host decodes as <u2 little-endian, and (b) breaks the
         * content-based phase classifier in camera_get_frame() — it pushes the
         * 8-bit video phase's value into the high byte so it reads >0xFF and is
         * mistaken for thermal, while the real thermal phase gets dropped.
         */
        .byte_swap_en           = false,
        .bit_swap_en            = false,
        .pic_format_jpeg        = false,
        .external_xtal          = 1,
        .pin_dont_init          = 1,
        .pin                    = NULL,
    };
    ESP_RETURN_ON_ERROR(esp_cam_new_dvp_ctlr(&dvp_cfg, &ctx->ctlr),
                        TAG, "DVP controller init failed");

    /* Allocate cache-aligned, DMA-capable PSRAM buffers via the driver so the
     * cache line / burst alignment matches what the DMA expects. */
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        ctx->bufs[i] = esp_cam_ctlr_alloc_buffer(
            ctx->ctlr, ctx->buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        ESP_RETURN_ON_FALSE(ctx->bufs[i], ESP_ERR_NO_MEM, TAG,
                            "frame buffer %d alloc failed (%zu B)", i, ctx->buf_size);
        ctx->state[i] = BUF_FREE;
    }

    /* Callbacks must be registered while dvp_fsm == INIT (before enable). */
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = camera_get_new_trans_cb,
        .on_trans_finished = camera_trans_finished_cb,
    };
    ESP_RETURN_ON_ERROR(
        esp_cam_ctlr_register_event_callbacks(ctx->ctlr, &cbs, ctx),
        TAG, "DVP callback registration failed");

    /*
     * NOTE: enable() is deliberately NOT called here. esp_cam_ctlr_enable()
     * does cam_ll_reset()+cam_ll_start() (the CAM block starts streaming), and
     * esp_cam_ctlr_start() only arms the DMA. If enable() runs at open() but
     * start() runs much later (on the host's stream commit), the CAM streams
     * into an unarmed DMA and wedges its FIFO → no frame ever completes.
     * So the enable→start pair is kept tight together in camera_start(), and
     * camera_stop() does stop→disable, giving a clean restart per stream.
     */

    /* Apply framing-signal polarity (GPIO matrix — independent of enable
     * state). The driver wires VSYNC inv=true / DE inv=false; if the source
     * disagrees, no frame completes. Defaults from Kconfig; sweep live with
     * the console V / D keys. */
    /*
     * Fixed framing config — these are the values confirmed to give a clean
     * 25 fps thermal feed on this camera (combined with the VSYNC-as-sole-
     * delimiter capture fix). No runtime sweeping; hardwired:
     *   - VSYNC inverted (matches the source's active-low VSYNC).
     *   - DE mode 0: DE driven active-high from the HSYNC pin (HSYNC is high
     *     for the 25.6 µs of pixel data, low for the 6 µs sync gap).
     *   - HSYNC NON-inverted (invert=0): the polarity that yields a clean feed.
     *   - PCLK non-inverted (rising-edge sampling).
     */
    camera_set_vsync_invert(ctx, true);
    camera_set_de_mode(ctx, 0);
    camera_set_hsync_invert(ctx, false);
    camera_set_pclk_invert(ctx, false);

    ESP_LOGI(TAG, "DVP open: %"PRIu32"x%"PRIu32" @%d fps out, %zu bytes/frame, "
             "parity_filter=%d keep_parity=%"PRIu32" vsync_inv=%d de_mode=%d",
             ctx->width, ctx->height, CONFIG_THERMAL_FPS, ctx->buf_size,
             ctx->parity_filter, ctx->keep_parity, ctx->vsync_invert, ctx->de_mode);
    return ESP_OK;
}

esp_err_t camera_start(camera_ctx_t *ctx)
{
    ctx->dbg_get_new_calls = 0;
    ctx->dbg_finished      = 0;
    ctx->dbg_no_free       = 0;
    ctx->dbg_torn          = 0;

    portENTER_CRITICAL(&ctx->lock);
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        ctx->state[i] = BUF_FREE;
    }
    portEXIT_CRITICAL(&ctx->lock);

    /*
     * Defensive reset to INIT so this function is IDEMPOTENT. The stall/freeze
     * watchdogs in camera_get_frame() call stop()+start() on their own; if one
     * fires between a host stream stop and the next commit (e.g. across a USB
     * suspend), the controller is left ENABLED/STARTED and a plain enable()
     * here returns ESP_ERR_INVALID_STATE — after which every subsequent stream
     * commit fails and the device is dead until reboot. stop()/disable() are
     * harmless no-ops when the FSM is already in the corresponding state, so
     * ignore their return values.
     */
    esp_cam_ctlr_stop(ctx->ctlr);
    esp_cam_ctlr_disable(ctx->ctlr);

    /* enable() resets and starts the CAM block (cam_ll_reset + cam_ll_start). */
    ESP_RETURN_ON_ERROR(esp_cam_ctlr_enable(ctx->ctlr), TAG, "DVP enable failed");

    /*
     * ROOT-CAUSE FIX (2026-06-22): order of operations during bring-up.
     * cam_hal_start() (inside enable()) immediately calls cam_ll_start(), so the
     * CAM begins streaming pixels BEFORE esp_cam_ctlr_start() arms the DMA — and
     * we then rewrite CAM config (16-bit, vh_de_mode, vsync filter) WHILE it is
     * already running, with no cam_update_reg pulse to commit them. The CAM ran
     * a frame or two into an unarmed/half-configured FIFO, then wedged its data
     * feed while VSYNC kept generating EOFs → continuous "completed" frames with
     * frozen pixel data, recoverable only by a full stop/start.
     *
     * Correct sequence: stop the CAM, apply ALL config, arm the DMA, then start
     * the CAM LAST (cam_ll_start pulses cam_update_reg, committing the config)
     * so it only ever feeds an armed, correctly-configured DMA.
     */
    lcd_cam_dev_t *cam = CAM_LL_GET_HW(0);

    cam_ll_stop(cam);   /* halt the feed cam_hal_start() just turned on */

    /* 16-bit sampling: cam_hal hardcodes 8-bit (cam_2byte_en=0); without this we
     * only sample 8 data lines/PCLK and no full frame ever forms. */
    cam_ll_set_input_data_width(cam, 16);

    /* VSYNC glitch filter (cam_hal leaves it OFF): edge ringing on 50 Hz VSYNC
     * otherwise fires thousands of false EOFs/sec. */
    cam_ll_enable_vsync_filter(cam, 1);
    cam_ll_set_vsync_filter_thres(cam, 7);

    /* vh_de framing (cam_hal sets mode 0, ignoring HSYNC). Mode 1 frames on
     * VSYNC + HSYNC + DE — matches this VSYNC/HSYNC/PCLK source. */
    cam_ll_set_vh_de_mode(cam, 1);

    /* Arm the DMA BEFORE the CAM feeds it. */
    esp_err_t ret = esp_cam_ctlr_start(ctx->ctlr);
    if (ret != ESP_OK) {
        esp_cam_ctlr_disable(ctx->ctlr);
        ESP_RETURN_ON_ERROR(ret, TAG, "DVP start failed");
    }

    /*
     * Start the CAM LAST, mirroring cam_hal_start_streaming() exactly:
     * reset the state machine + FIFO, THEN start. The FIFO reset is the bit my
     * earlier reorder missed — without it the CAM starts from whatever stale
     * data sat in the async RX FIFO from the brief run during enable(), which
     * desyncs it from the frame boundary (feeds 1-2 frames then stops feeding
     * while VSYNC keeps EOF-ing → frozen content). reset/fifo_reset are
     * self-clearing cam_ctrl1 bits and do NOT clear the data_width / vh_de_mode
     * / vsync_filter config set above. cam_ll_start pulses cam_update_reg to
     * commit that config. Now the CAM streams from a clean state into the
     * already-armed DMA.
     */
    cam_ll_reset(cam);
    cam_ll_fifo_reset(cam);
    cam_ll_start(cam);

    ESP_LOGI(TAG, "DVP capture running (DMA armed, CAM reset+fifo_reset+start)");
    return ESP_OK;
}

esp_err_t camera_stop(camera_ctx_t *ctx)
{
    /* stop() halts DMA + CAM (STARTED->ENABLED); disable() returns to INIT so
     * the next camera_start() does a clean enable→start (re-runs cam_ll_start).
     * Without the disable, a second stream commit would never restart the CAM. */
    esp_cam_ctlr_stop(ctx->ctlr);
    esp_cam_ctlr_disable(ctx->ctlr);

    portENTER_CRITICAL(&ctx->lock);
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        ctx->state[i] = BUF_FREE;
    }
    portEXIT_CRITICAL(&ctx->lock);

    ESP_LOGI(TAG, "DVP capture stopped");
    return ESP_OK;
}

const uint16_t *camera_get_frame(camera_ctx_t *ctx, void **out_buf,
                                  uint32_t timeout_ms)
{
    /*
     * timeout_ms is an OVERALL deadline, not a per-wait timeout.
     *
     * It used to be per-wait, and that hung the web stream for as long as a USB
     * host was attached. The ISR gives frame_sem on EVERY capture (50/s), so a
     * consumer that keeps losing the buffer race — the prio-5 mjpeg task against
     * the prio-23 UVC task — woke, found nothing READY, and re-armed the FULL
     * timeout on the next take, which the next give satisfied ~20 ms later. The
     * "timed out" branch needs `timeout_ms` of total semaphore SILENCE to be
     * reached, which never happens while the DVP is running, so the starved
     * caller spun in here indefinitely. It only escaped when the UVC host went
     * away and stopped claiming buffers.
     *
     * That wedged the mjpeg task with s_web.client_busy still set, so every
     * later /stream request 503'd after its 2 s wait for the incumbent to
     * release — the ~2 s "httpd_uri: uri handler execution failed" cadence.
     *
     * A single deadline captured at entry also bounds the wrong-phase retry
     * path below, which re-armed the timeout the same way on every drop.
     */
    const bool           infinite = (timeout_ms == portMAX_DELAY);
    const TickType_t     budget   = infinite ? 0 : pdMS_TO_TICKS(timeout_ms);
    const TickType_t     started  = xTaskGetTickCount();

    int idx = -1;
    for (;;) {
        /* Claim the NEWEST ready buffer (highest seq) and mark it READING.
         * Under the lock the ISR can't hand it back to the DMA while we hold
         * it. Claiming before blocking on the semaphore matters: after we
         * drop a wrong-phase frame there may already be another READY frame
         * (two slots) whose semaphore give we consumed earlier — we must not
         * stall waiting for a fresh give. */
        portENTER_CRITICAL(&ctx->lock);
        uint32_t best_seq = 0;
        idx = -1;
        for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
            if (ctx->state[i] == BUF_READY && ctx->buf_seq[i] >= best_seq) {
                best_seq = ctx->buf_seq[i];
                idx = i;
            }
        }
        if (idx >= 0) {
            ctx->state[idx] = BUF_READING;
        }
        portEXIT_CRITICAL(&ctx->lock);

        if (idx < 0) {
            /* Time left on the deadline. Tick subtraction wraps correctly. */
            TickType_t left = portMAX_DELAY;
            if (!infinite) {
                TickType_t spent = xTaskGetTickCount() - started;
                left = (spent >= budget) ? 0 : (budget - spent);
            }
            if (left > 0 && xSemaphoreTake(ctx->frame_sem, left) == pdTRUE) {
                continue;   /* a capture completed — go claim it */
            }

            /*
             * Out of time. Two very different reasons land here, and only one
             * of them is a fault:
             *
             *   captures ADVANCED  — the DVP is healthy, this caller just lost
             *     every buffer race (a low-priority consumer under UVC load).
             *     Normal contention; the caller retries. Debug level only.
             *   captures did NOT advance — the CAM really has gone quiet.
             *     That is the fault case, and the stall watchdog below owns it.
             */
            bool progressing = (ctx->dbg_finished != ctx->stall_last_finished);
            if (progressing) {
                ESP_LOGD(TAG, "starved (finished=%"PRIu32", no buffer won this window)",
                         ctx->dbg_finished);
            } else {
                /* Throttle: at 25 fps this would spam ~25 lines/s and bury the
                 * console prompt. Log roughly once a second. */
                static uint32_t to_count;
                if ((to_count++ % 25u) == 0) {
                    ESP_LOGW(TAG, "frame timeout — get_new=%"PRIu32" finished=%"PRIu32" no_free=%"PRIu32
                             " torn=%"PRIu32" recv=%"PRIu32"/%zu (vsync_inv=%d de_mode=%d)",
                             ctx->dbg_get_new_calls, ctx->dbg_finished, ctx->dbg_no_free,
                             ctx->dbg_torn, ctx->dbg_recv_size, ctx->buf_size,
                             ctx->vsync_invert, ctx->de_mode);
                }
            }

            /*
             * Hard-stall watchdog. A timeout means no frame completed this
             * window. If dbg_finished has NOT advanced since the previous
             * timeout, the CAM has stopped issuing EOFs (a true hang, not a
             * static scene — static scenes still complete captures). After two
             * consecutive no-progress timeouts (~2 frame-timeout windows) bounce
             * capture to recover. This only fires on a genuine stall, so it
             * cannot thrash during normal streaming.
             */
            if (ctx->dbg_finished == ctx->stall_last_finished) {
                if (++ctx->stall_count >= 2) {
                    ESP_LOGW(TAG, "capture STALLED (finished stuck at %"PRIu32") — resyncing CAM",
                             ctx->dbg_finished);
                    ctx->stall_count = 0;
                    camera_stop(ctx);
                    camera_start(ctx);
                    ctx->stall_last_finished = 0;
                }
            } else {
                ctx->stall_last_finished = ctx->dbg_finished;
                ctx->stall_count = 0;
            }
            return NULL;
        }

        /*
         * NO cache invalidate here — it would be pure duplicated work.
         *
         * The IDF 6.0.1 DVP ISR already invalidated this ENTIRE buffer before
         * it published it: esp_cam_ctlr_recv_frame_done_isr() calls
         * esp_cam_ctlr_dvp_get_recved_size(), which does an unconditional
         * esp_cache_msync(rx_buffer, fb_size_in_bytes, M2C) for any buffer in
         * external RAM (ours are all PSRAM) — and only then invokes
         * on_trans_finished, i.e. the callback that marked this buffer READY.
         * So by the time we can possibly see BUF_READY, the data is coherent.
         *
         * The msync that used to be here re-invalidated frame_size (221 KB at
         * 384x288) on the consumer's core, inside esp_cache_msync's spinlock
         * with interrupts disabled, once per delivered frame, for no effect.
         *
         * (If the driver is ever updated such that it no longer syncs before
         * the callback, this must come back — the check is the msync call in
         * esp_cam_ctlr_dvp_get_recved_size() in esp_cam_ctlr_dvp_cam.c.)
         */

        /*
         * Interleaved-phase (A/B) selection BY CONTENT, not by capture parity.
         * The source alternates two phases at 50 Hz: a 14-bit thermal frame
         * (pixel values use the full range, >0xFF) and an 8-bit video frame
         * (values fit in the low 8 bits, <=0xFF). Classifying by content is
         * immune to capture-counter drift, so an FFC/glitch/resync can't flip
         * which phase we deliver. keep_parity: 1 = thermal (14-bit), 0 = video.
         */
        if (ctx->parity_filter) {
            const uint16_t *p = (const uint16_t *)ctx->bufs[idx];
            uint32_t n = ctx->frame_size / 2;   /* classify on the active image only */
            uint32_t step = (n / 512u) ? (n / 512u) : 1u;
            /*
             * Classify by a MAJORITY of pixels, not a single one. The 14-bit
             * thermal phase has a high baseline (~7000 counts) so virtually
             * EVERY pixel is >0xFF; the 8-bit video phase is <=0xFF — EXCEPT
             * when the scene contains a hot, visibly-glowing object, which makes
             * a few bright pixels in the video frame exceed 0xFF. The old
             * "any pixel >0xFF" test then let the video frame sneak through, so
             * the view flipped to the 8-bit range whenever something got hot
             * enough to glow. A majority test separates the two phases cleanly
             * regardless of a few bright spots.
             */
            uint32_t hot = 0, total = 0;
            for (uint32_t i = 0; i < n; i += step) {
                total++;
                if ((p[i] & 0x3FFFu) > 0xFFu) {
                    hot++;
                }
            }
            /*
             * DEAD BAND around the 50% line. A bare majority (hot*2 > total)
             * classifies every frame as one phase or the other, including a
             * TORN one — a capture whose DMA was rearmed late, so it holds the
             * tail of one phase followed by the head of the next. Those land
             * near 50/50 and the vote becomes a coin flip, which is how a
             * half-video/half-thermal frame reaches the consumer.
             *
             * It is visible in two ways, and both are on the reported symptom
             * list: the image appears shifted sideways (the seam), and the web
             * AGC sees a bimodal histogram — an 8-bit lobe near 0 plus the
             * ~7000-count thermal lobe — which drags `lo` to the bottom of the
             * range and washes the frame out for the one or two frames it
             * takes to pass through.
             *
             * A clean phase is overwhelmingly one-sided (thermal has a ~7000
             * baseline so virtually every pixel is >0xFF; video is <=0xFF apart
             * from a few glowing spots), so demanding 75% costs nothing on
             * healthy frames and rejects the mixed ones outright. Anything in
             * the band is dropped like a wrong-phase frame — the next capture
             * is 20 ms away.
             */
            uint32_t phase;
            if (hot * 4u > total * 3u) {
                phase = 1u;                   /* >75% high — clean thermal */
            } else if (hot * 4u < total) {
                phase = 0u;                   /* <25% high — clean 8-bit video */
            } else {
                ctx->dbg_torn++;              /* mixed — a torn capture */
                phase = !ctx->keep_parity;    /* force the drop path below */
            }
            if (phase != ctx->keep_parity) {
                portENTER_CRITICAL(&ctx->lock);
                ctx->state[idx] = BUF_FREE;   /* wrong phase — drop, wait for the other */
                portEXIT_CRITICAL(&ctx->lock);
                idx = -1;
                continue;
            }
        }
        /*
         * NOTE: an earlier shear-guard that dropped frames whose received_size
         * was less than frame_size regressed the web viewer to ~9 fps and
         * caused "handler execution failed" spam — the driver's received_size
         * field is not populated the way that check assumed, so it fired on
         * healthy captures. Left as a diagnostic log only until a real shear
         * signature is worked out.
         */
        {
            static uint32_t log_gate;
            if ((log_gate++ % 250u) == 0) {
                ESP_LOGD(TAG, "recv=%"PRIu32" B (frame=%zu B, buf=%zu B)",
                         ctx->buf_recv[idx], ctx->frame_size, ctx->buf_size);
            }
        }
        break;
    }

    /*
     * ---- Static-content detector feeding the freeze watchdog below ----
     * Count consecutive byte-identical delivered frames (live sensor noise makes
     * every real frame's hash unique, so a long identical run == frozen).
     */
    {
        const uint32_t *w = (const uint32_t *)ctx->bufs[idx];
        uint32_t words = ctx->frame_size / 4;    /* hash the active image only */
        uint32_t step = (words / 1024u) ? (words / 1024u) : 1u;
        uint32_t hash = 2166136261u;             /* FNV-1a over sparse samples */
        for (uint32_t i = 0; i < words; i += step) {
            hash = (hash ^ w[i]) * 16777619u;
        }
        if (hash == ctx->frz_ring[0]) {
            ctx->frz_stale++;
        } else {
            ctx->frz_ring[0] = hash;
            ctx->frz_stale = 0;
        }
    }

    /*
     * Content-freeze auto-resync. Each cam_ll_start lands EITHER in a healthy
     * "live" state (frames keep updating, frz_stale stays ~0) OR a "frozen"
     * state (CAM feeds 1-2 frames then stops feeding while VSYNC keeps EOF-ing,
     * so the same buffer is redelivered — frz_stale climbs without bound). It's
     * a start-time race; the live state is STABLE once entered. So if we have
     * seen ~45 consecutive byte-identical delivered frames (~1.8 s at 25 fps —
     * far beyond any live-but-static scene, which still has sensor noise),
     * bounce capture to re-roll the race. This self-terminates: once a start
     * lands live, frz_stale stays low and we never resync again.
     */
    if (ctx->frz_stale >= 45) {
        ESP_LOGW(TAG, "content frozen (%"PRIu32" identical frames) — re-rolling CAM start",
                 ctx->frz_stale);
        ctx->frz_stale = 0;
        ctx->frz_ring[0] = 0;
        portENTER_CRITICAL(&ctx->lock);
        ctx->state[idx] = BUF_FREE;
        portEXIT_CRITICAL(&ctx->lock);
        camera_stop(ctx);
        camera_start(ctx);
        return NULL;   /* consumer emits one keep-alive; next call is fresh */
    }

    /*
     * Observer tap. Copy the frame we are about to hand out if someone has
     * asked for one, so a second viewer never has to claim a buffer.
     *
     * A second *consumer* does not work here: the UVC task runs at priority 23
     * and calls straight back into camera_get_frame() after each transfer, so
     * it wins the buffer every time the ISR publishes one. A lower-priority
     * observer (the HTTP task, priority 5) is never scheduled in the gap and
     * gets literally zero frames, not a fair share. Copying costs one memcpy
     * on whoever is already consuming, and only while a tap is pending.
     */
    /*
     * CLAIM the request under the lock before copying, and hold tap_busy for
     * the duration. Testing tap_pending and clearing it after the memcpy (as
     * this did) races with camera_tap_get()'s timeout path: the reader could
     * time out mid-copy, clear tap_pending, return false, then come straight
     * back, drain tap_sem, re-arm — and catch the give from the copy that was
     * still in flight, reading tap_buf while we were writing it. That hands
     * the JPEG encoder a frame seamed from two captures, which shows up in the
     * browser as a sideways tear and a one-frame AGC lurch.
     */
    bool do_tap = false;
    portENTER_CRITICAL(&ctx->lock);
    if (ctx->tap_pending && ctx->tap_buf) {
        ctx->tap_pending = false;
        ctx->tap_busy    = true;
        do_tap = true;
    }
    portEXIT_CRITICAL(&ctx->lock);

    if (do_tap) {
        memcpy(ctx->tap_buf, ctx->bufs[idx], ctx->frame_size);
        ctx->tap_bytes  = ctx->frame_size;
        ctx->tap_width  = ctx->width;
        ctx->tap_height = ctx->height;
        ctx->tap_busy   = false;
        xSemaphoreGive(ctx->tap_sem);
    }

    *out_buf = ctx->bufs[idx];
    return (const uint16_t *)ctx->bufs[idx];
}

esp_err_t camera_tap_init(camera_ctx_t *ctx, size_t max_bytes)
{
    if (ctx->tap_buf) {
        return ESP_OK;                       /* already set up */
    }
    ctx->tap_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(ctx->tap_sem, ESP_ERR_NO_MEM, TAG, "tap_sem alloc failed");
    ctx->tap_buf = heap_caps_malloc(max_bytes, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(ctx->tap_buf, ESP_ERR_NO_MEM, TAG, "tap buffer alloc failed");
    ctx->tap_cap = max_bytes;
    return ESP_OK;
}

const uint16_t *camera_tap_borrow(camera_ctx_t *ctx, uint32_t *out_w,
                                   uint32_t *out_h, uint32_t timeout_ms)
{
    if (!ctx->tap_buf) {
        return NULL;
    }
    /*
     * Never re-arm while a copy from the previous round is still running —
     * that is the other half of the claim in camera_get_frame(). A short
     * bounded wait is enough: tap_busy only spans one memcpy.
     */
    for (int i = 0; i < 10 && ctx->tap_busy; i++) {
        vTaskDelay(1);
    }

    /* Drain any stale give so we wait for a frame captured after this call. */
    xSemaphoreTake(ctx->tap_sem, 0);
    ctx->tap_pending = true;

    if (xSemaphoreTake(ctx->tap_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        /*
         * Withdraw the request — but only if it has not already been claimed.
         * If a consumer took it while we were timing out, the copy is in
         * flight and tap_buf is being written; give it a brief grace period
         * rather than returning to a caller that would re-arm on top of it.
         */
        bool claimed;
        portENTER_CRITICAL(&ctx->lock);
        claimed = !ctx->tap_pending;         /* consumer already took it */
        ctx->tap_pending = false;
        portEXIT_CRITICAL(&ctx->lock);

        if (claimed && xSemaphoreTake(ctx->tap_sem, pdMS_TO_TICKS(20)) == pdTRUE) {
            goto have_frame;                 /* the copy landed after all */
        }
        return NULL;                         /* nobody is consuming frames */
    }

have_frame:
    /*
     * BORROW, don't copy. tap_buf is written only by a consumer that has
     * claimed a tap_pending request, and only this reader ever sets that flag
     * — so once we are here, tap_buf is stable until our next call, and the
     * tap_busy spin at the top of this function guarantees the previous copy
     * has finished before we re-arm.
     *
     * This used to memcpy tap_buf into a caller-owned buffer, which meant the
     * frame crossed PSRAM twice on its way to the encoder: once when the
     * consumer copied the DMA buffer into tap_buf, once again here. At 640x480
     * that second copy was 614 KB per web frame for no benefit.
     */
    if (out_w) *out_w = ctx->tap_width;
    if (out_h) *out_h = ctx->tap_height;
    return (const uint16_t *)ctx->tap_buf;
}

void camera_release_frame(camera_ctx_t *ctx, void *buf)
{
    portENTER_CRITICAL(&ctx->lock);
    for (int i = 0; i < DVP_BUFFER_COUNT; i++) {
        if (ctx->bufs[i] == buf && ctx->state[i] == BUF_READING) {
            ctx->state[i] = BUF_FREE;
            break;
        }
    }
    portEXIT_CRITICAL(&ctx->lock);
}

/* ---- Framing routing (fixed; applied once at open) ---------------------- */

void camera_set_vsync_invert(camera_ctx_t *ctx, bool invert)
{
    ctx->vsync_invert = invert;
    esp_rom_gpio_connect_in_signal(CONFIG_DVP_VSYNC_GPIO,
                                   cam_periph_signals.buses[0].vsync_sig, invert);
    ESP_LOGI(TAG, "VSYNC invert = %d", invert);
}

void camera_set_pclk_invert(camera_ctx_t *ctx, bool invert)
{
    ctx->pclk_invert = invert;
    /* Re-route the PCLK pin through the GPIO matrix with optional inversion so
     * we can latch pixel data on the opposite clock edge. If the bus is sampled
     * on the wrong edge the captured words collapse to a near-constant value
     * (uniform "white" frame), since every latch grabs a held/settling value
     * instead of the freshly-driven pixel. */
    esp_rom_gpio_connect_in_signal(CONFIG_DVP_PCLK_GPIO,
                                   cam_periph_signals.buses[0].pclk_sig, invert);
    ESP_LOGI(TAG, "PCLK invert = %d (sampling on %s edge)",
             invert, invert ? "falling" : "rising");
}

void camera_set_hsync_invert(camera_ctx_t *ctx, bool invert)
{
    ctx->hsync_invert = invert;
    esp_rom_gpio_connect_in_signal(CONFIG_DVP_HSYNC_GPIO,
                                   cam_periph_signals.buses[0].hsync_sig, invert);
    ESP_LOGI(TAG, "HSYNC invert = %d (routed to controller HSYNC input)", invert);
}

void camera_set_de_mode(camera_ctx_t *ctx, int mode)
{
    ctx->de_mode = mode;
    switch (mode) {
    case 1: /* HSYNC pin as HREF, inverted */
        esp_rom_gpio_connect_in_signal(CONFIG_DVP_HSYNC_GPIO,
                                       cam_periph_signals.buses[0].de_sig, true);
        ESP_LOGI(TAG, "DE mode 1: HSYNC as HREF (inverted)");
        break;
    case 2: /* free-run: tie DE permanently high, capture every PCLK */
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
                                       cam_periph_signals.buses[0].de_sig, false);
        ESP_LOGI(TAG, "DE mode 2: free-run (DE tied high)");
        break;
    case 0: /* HSYNC pin as HREF, active-high (driver default) */
    default:
        ctx->de_mode = 0;
        esp_rom_gpio_connect_in_signal(CONFIG_DVP_HSYNC_GPIO,
                                       cam_periph_signals.buses[0].de_sig, false);
        ESP_LOGI(TAG, "DE mode 0: HSYNC as HREF (active-high)");
        break;
    }
}

esp_err_t camera_set_resolution(camera_ctx_t *ctx, uint32_t w, uint32_t h)
{
    /* Change only the DELIVERED frame size. The DVP/DMA buffers stay allocated
     * for the maximum (primary) resolution, so a smaller frame is just captured
     * contiguously and VSYNC-delimited — no controller/buffer rebuild needed.
     * Must be called while capture is stopped (on_stream_start does this). */
    size_t need = (size_t)w * h * 2;
    if (need == 0 || need > ctx->buf_size) {
        ESP_LOGE(TAG, "resolution %ux%u (%zu B) exceeds capture buffer (%zu B)",
                 (unsigned)w, (unsigned)h, need, ctx->buf_size);
        return ESP_ERR_INVALID_ARG;
    }
    ctx->width      = w;
    ctx->height     = h;
    ctx->frame_size = need;
    ESP_LOGI(TAG, "delivered resolution %ux%u (%zu B/frame)", (unsigned)w, (unsigned)h, need);
    return ESP_OK;
}

void camera_set_keep_parity(camera_ctx_t *ctx, uint32_t keep)
{
    /* Which interleaved phase to deliver over UVC: 1 = 14-bit thermal,
     * 0 = 8-bit visible video. The camera's on-screen menu is only drawn into
     * the 8-bit video phase, so switch to 0 to see/drive the menu. */
    ctx->keep_parity = keep ? 1u : 0u;
    ESP_LOGI(TAG, "deliver %s phase",
             ctx->keep_parity ? "14-bit thermal" : "8-bit video (menu visible)");
}
