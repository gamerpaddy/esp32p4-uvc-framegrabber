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
#include "esp_cache.h"
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
    TickType_t ticks = (timeout_ms == portMAX_DELAY)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(timeout_ms);

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
            if (xSemaphoreTake(ctx->frame_sem, ticks) == pdTRUE) {
                continue;   /* a capture completed — go claim it */
            }
            /* Throttle: at 25 fps this would spam ~25 lines/s and bury the
             * console prompt. Log roughly once a second. */
            static uint32_t to_count;
            if ((to_count++ % 25u) == 0) {
                ESP_LOGW(TAG, "frame timeout — get_new=%"PRIu32" finished=%"PRIu32" no_free=%"PRIu32
                         " recv=%"PRIu32"/%zu (vsync_inv=%d de_mode=%d)",
                         ctx->dbg_get_new_calls, ctx->dbg_finished, ctx->dbg_no_free,
                         ctx->dbg_recv_size, ctx->buf_size,
                         ctx->vsync_invert, ctx->de_mode);
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

        /* Invalidate the cache so the CPU sees the DMA-written PSRAM data.
         * Only the active image (frame_size) is ever read — the classifier,
         * the freeze hash, and the USB copy all stop there — so skip the
         * DVP_VBLANK_CAPTURE_MARGIN tail to shave time off the cycle. */
        esp_cache_msync(ctx->bufs[idx], ctx->frame_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

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
            bool is_thermal = (hot * 2u) > total;   /* >50% of pixels are high */
            uint32_t phase = is_thermal ? 1u : 0u;
            if (phase != ctx->keep_parity) {
                portENTER_CRITICAL(&ctx->lock);
                ctx->state[idx] = BUF_FREE;   /* wrong phase — drop, wait for the other */
                portEXIT_CRITICAL(&ctx->lock);
                idx = -1;
                continue;
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
    if (ctx->tap_pending && ctx->tap_buf) {
        memcpy(ctx->tap_buf, ctx->bufs[idx], ctx->frame_size);
        ctx->tap_bytes   = ctx->frame_size;
        ctx->tap_width   = ctx->width;
        ctx->tap_height  = ctx->height;
        ctx->tap_pending = false;
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

bool camera_tap_get(camera_ctx_t *ctx, uint16_t *dst, size_t dst_bytes,
                    uint32_t *out_w, uint32_t *out_h, uint32_t timeout_ms)
{
    if (!ctx->tap_buf) {
        return false;
    }
    /* Drain any stale give so we wait for a frame captured after this call. */
    xSemaphoreTake(ctx->tap_sem, 0);
    ctx->tap_pending = true;

    if (xSemaphoreTake(ctx->tap_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ctx->tap_pending = false;
        return false;                        /* nobody is consuming frames */
    }
    size_t n = ctx->tap_bytes < dst_bytes ? ctx->tap_bytes : dst_bytes;
    memcpy(dst, ctx->tap_buf, n);
    if (out_w) *out_w = ctx->tap_width;
    if (out_h) *out_h = ctx->tap_height;
    return true;
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
