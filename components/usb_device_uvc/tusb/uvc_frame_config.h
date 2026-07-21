/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frame configuration for the thermal camera Y16 UVC bridge.
 *
 * Single format: Y16 (16-bit little-endian grayscale).
 * Bits [13:0] of each uint16_t carry the raw 14-bit thermal ADC value.
 * Bits [15:14] are always zero (D14/D15 grounded on the DVP connector).
 *
 * Host decode (Python / NumPy):
 *   raw_u16 = np.frombuffer(frame_bytes, dtype='<u2').reshape(H, W)
 *   thermal14 = raw_u16 & 0x3FFF   # mask upper 2 zero bits
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "sdkconfig.h"

/*
 * Two advertised resolutions so the host can switch sensors without reflashing:
 *   Frame 1 = primary (Kconfig, currently 640x480) — also the MAX, used to size
 *             the capture/DMA/USB buffers.
 *   Frame 2 = 384x288 (the older sensor).
 * The host (viewer) selects which one; the ESP delivers that many bytes/frame.
 * The primary MUST be the larger of the two (buffers are sized from it).
 */
#define THERMAL_WIDTH    CONFIG_THERMAL_WIDTH    /* frame 1 = boot/default */
#define THERMAL_HEIGHT   CONFIG_THERMAL_HEIGHT
#define THERMAL_WIDTH2   640                     /* frame 2 = alternate    */
#define THERMAL_HEIGHT2  480
#define THERMAL_FPS      CONFIG_THERMAL_FPS

/*
 * Runtime resolution budget for RES,W,H and UVC re-commits. Buffer allocations
 * (capture DMA, UVC transfer, nosignal, tap) are sized once to these hard
 * caps so any accepted W x H fits without reallocation:
 *   - THERMAL_MAX_WIDTH / _HEIGHT bound each dimension individually (they set
 *     the DVP h_res/v_res, which are fixed at controller-init and truncate
 *     anything larger).
 *   - THERMAL_MAX_PIXELS additionally bounds the product so unusual aspect
 *     ratios don't blow past the intended memory footprint.
 * ~500k pixels covers 640x480 with comfortable headroom; bump the per-dim
 * caps if a wider or taller sensor shows up.
 */
#define THERMAL_MAX_WIDTH   800
#define THERMAL_MAX_HEIGHT  640
#define THERMAL_MAX_PIXELS  500000

/* Bytes per pixel for the Y16 format (2 bytes = 16-bit). */
#define THERMAL_BPP     2

/* ---- Format / frame tables used by usb_device_uvc.c -------------------- */

typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t  max_fps;
} uvc_frame_info_t;

/* One format (Y16 uncompressed), two frame sizes (640x480 and 384x288). */
#define UVC_NUM_FORMATS   1
#define Y16_FRAME_COUNT   2

static const uvc_frame_info_t uvc_y16_frames[Y16_FRAME_COUNT] = {
    { THERMAL_WIDTH,  THERMAL_HEIGHT,  THERMAL_FPS },   /* frame 1 */
    { THERMAL_WIDTH2, THERMAL_HEIGHT2, THERMAL_FPS },   /* frame 2 */
};

/*
 * Look up frame info by (bFormatIndex, bFrameIndex) — both 1-based per UVC spec.
 * Format 1 = Y16.  Frame 1 = primary (640x480), Frame 2 = 384x288.
 */
static inline const uvc_frame_info_t *uvc_get_frame_info(uint8_t format_index,
                                                           uint8_t frame_index)
{
    if (format_index == 1 && frame_index >= 1 && frame_index <= Y16_FRAME_COUNT) {
        return &uvc_y16_frames[frame_index - 1];
    }
    return NULL;
}
