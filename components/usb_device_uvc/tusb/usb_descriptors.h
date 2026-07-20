/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * UVC 1.5 descriptor for ESP32-P4 thermal camera bridge (Bulk, HS).
 *
 * Single format: Y16 uncompressed (16-bit grayscale, 14 thermal bits in [13:0]).
 *
 * Descriptor hierarchy:
 *   VideoControl Interface
 *     +-- Camera Terminal (IT)
 *     +-- Processing Unit (PU)   — no controls (thermal, read-only sensor)
 *     +-- Output Terminal (OT)
 *   VideoStreaming Interface (Bulk, single alt-setting with endpoint)
 *     +-- VS Input Header (bNumFormats=1)
 *     +-- Format 1: Y16 Uncompressed
 *     +-- Frame 1:  THERMAL_WIDTH x THERMAL_HEIGHT @ THERMAL_FPS
 *     +-- Bulk Endpoint IN
 */

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "uvc_frame_config.h"

/* ---------- Entity IDs --------------------------------------------------- */
#define UVC_ENTITY_CAP_INPUT_TERMINAL   0x01
#define UVC_ENTITY_PROCESSING_UNIT      0x02
#define UVC_ENTITY_CAP_OUTPUT_TERMINAL  0x03

/* ---------- Clock --------------------------------------------------------- */
#define UVC_CLOCK_FREQUENCY  27000000UL

/* ---------- Interface / endpoint numbers --------------------------------- */
enum {
    ITF_NUM_VIDEO_CONTROL,
    ITF_NUM_VIDEO_STREAMING,
    ITF_NUM_CDC,            /* CDC-ACM control (camera UART bridge) */
    ITF_NUM_CDC_DATA,       /* CDC-ACM data                          */
    ITF_NUM_TOTAL
};
#define EPNUM_VIDEO_IN    0x81
#define EPNUM_CDC_NOTIF   0x82   /* CDC notification IN */
#define EPNUM_CDC_OUT     0x02   /* CDC data OUT        */
#define EPNUM_CDC_IN      0x83   /* CDC data IN         */

/* ---------- Helper: frame interval in 100 ns units ----------------------- */
#define FI(fps)  (10000000UL / (fps))

/* ---------- Y16 format GUID ---------------------------------------------- */
/*
 * Y16 FourCC = 'Y','1','6',' '
 * GUID = {20363159-0000-0010-8000-00AA00389B71}
 * Byte order in USB descriptor (Data1 as LE, Data4 as-is):
 *   59 31 36 20  00 00  10 00  80 00 00 AA 00 38 9B 71
 */
#define TUD_VIDEO_GUID_Y16 \
    0x59U, 0x31U, 0x36U, 0x20U, 0x00U, 0x00U, 0x10U, 0x00U, \
    0x80U, 0x00U, 0x00U, 0xAAU, 0x00U, 0x38U, 0x9BU, 0x71U

/* Convenience wrapper (mirrors TUD_VIDEO_DESC_CS_VS_FMT_YUY2 pattern). */
#define TUD_VIDEO_DESC_CS_VS_FMT_Y16(_fmtidx, _numfr, _frmidx, \
                                      _ax, _ay, _interlace, _cp) \
    TUD_VIDEO_DESC_CS_VS_FMT_UNCOMPR(_fmtidx, _numfr, TUD_VIDEO_GUID_Y16, \
                                      16, _frmidx, _ax, _ay, _interlace, _cp)

/* ---------- Processing Unit (no controls for read-only thermal sensor) --- */
#define TUD_VIDEO_DESC_PROCESSING_UNIT_LEN  13
#define TUD_VIDEO_DESC_PROCESSING_UNIT(_uid, _src) \
    TUD_VIDEO_DESC_PROCESSING_UNIT_LEN, \
    TUSB_DESC_CS_INTERFACE, VIDEO_CS_ITF_VC_PROCESSING_UNIT, \
    _uid, _src, \
    U16_TO_U8S_LE(0x0000), /* wMaxMultiplier */ \
    0x03,                   /* bControlSize   */ \
    0x00, 0x00, 0x00,       /* bmControls: none */ \
    0x00,                   /* iProcessing    */ \
    0x00                    /* bmVideoStandards */

/* ---------- VS Input Header for 1 format --------------------------------- */
/*
 * bLength = 13 + (bNumFormats × bControlSize) = 13 + 1×1 = 14
 * wTotalLength = VS_INPUT_HDR_LEN + VS_TOTAL_INNER_LEN
 */
#define VS_INPUT_HDR_LEN  (13 + UVC_NUM_FORMATS * 1)   /* = 14 */

#define TUD_VIDEO_DESC_CS_VS_INPUT_1FMT(_totallen, _epin, _termlnk) \
    VS_INPUT_HDR_LEN, \
    TUSB_DESC_CS_INTERFACE, \
    0x01,                   /* VIDEO_CS_ITF_VS_INPUT_HEADER */ \
    0x01,                   /* bNumFormats = 1              */ \
    U16_TO_U8S_LE((_totallen) + VS_INPUT_HDR_LEN), /* wTotalLength */ \
    _epin,                  /* bEndpointAddress             */ \
    0x00,                   /* bmInfo                       */ \
    _termlnk,               /* bTerminalLink                */ \
    0x00,                   /* bStillCaptureMethod          */ \
    0x00,                   /* bTriggerSupport              */ \
    0x00,                   /* bTriggerUsage                */ \
    0x01,                   /* bControlSize = 1             */ \
    0x00                    /* bmaControls[0] (no controls) */

/* ---------- Descriptor length calculations ------------------------------- */

#define VC_TOTAL_INNER_LEN ( \
    TUD_VIDEO_DESC_CAMERA_TERM_LEN        + \
    TUD_VIDEO_DESC_PROCESSING_UNIT_LEN    + \
    TUD_VIDEO_DESC_OUTPUT_TERM_LEN          \
)

#define VS_TOTAL_INNER_LEN ( \
    TUD_VIDEO_DESC_CS_VS_FMT_UNCOMPR_LEN                        + \
    (Y16_FRAME_COUNT * TUD_VIDEO_DESC_CS_VS_FRM_UNCOMPR_CONT_LEN) \
)

/* Full UVC function length: IAD + VC iface + VS iface + bulk endpoint. */
#define UVC_DESC_TOTAL_LEN ( \
    TUD_VIDEO_DESC_IAD_LEN              + \
    TUD_VIDEO_DESC_STD_VC_LEN           + \
    (TUD_VIDEO_DESC_CS_VC_LEN + 1)      + \
    VC_TOTAL_INNER_LEN                  + \
    TUD_VIDEO_DESC_STD_VS_LEN           + \
    VS_INPUT_HDR_LEN                    + \
    VS_TOTAL_INNER_LEN                  + \
    7   /* Bulk endpoint descriptor */    \
)

/* Composite configuration: UVC function + CDC-ACM function. */
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + UVC_DESC_TOTAL_LEN + TUD_CDC_DESC_LEN)

/* ---------- Full descriptor macro ---------------------------------------- */
/*
 * _stridx : string descriptor index for the UVC interface
 * _itf    : first interface number (VC); VS = _itf+1
 * _epin   : bulk IN endpoint address (e.g. 0x81)
 * _epsize : max packet size (512 for HS bulk)
 */
#define TUD_VIDEO_CAPTURE_DESCRIPTOR_THERMAL_BULK(_stridx, _itf, _epin, _epsize) \
    /* ---- Interface Association Descriptor ---- */ \
    TUD_VIDEO_DESC_IAD(_itf, 0x02, _stridx), \
    \
    /* ==== Video Control Interface ==== */ \
    TUD_VIDEO_DESC_STD_VC(_itf, 0, _stridx), \
    TUD_VIDEO_DESC_CS_VC( \
        0x0150,             /* bcdUVC 1.50           */ \
        VC_TOTAL_INNER_LEN, \
        UVC_CLOCK_FREQUENCY, \
        _itf + 1            /* bInCollection: 1 VS   */ \
    ), \
    /* Camera Terminal (IT): no exposure controls */ \
    TUD_VIDEO_DESC_CAMERA_TERM( \
        UVC_ENTITY_CAP_INPUT_TERMINAL, 0, 0, \
        0, 0, 0, 0x00 \
    ), \
    /* Processing Unit: source=IT, no controls */ \
    TUD_VIDEO_DESC_PROCESSING_UNIT( \
        UVC_ENTITY_PROCESSING_UNIT, \
        UVC_ENTITY_CAP_INPUT_TERMINAL \
    ), \
    /* Output Terminal: source=PU */ \
    TUD_VIDEO_DESC_OUTPUT_TERM( \
        UVC_ENTITY_CAP_OUTPUT_TERMINAL, \
        VIDEO_TT_STREAMING, 0, \
        UVC_ENTITY_PROCESSING_UNIT, 0 \
    ), \
    \
    /* ==== Video Streaming Interface (Bulk, single alt-setting) ==== */ \
    TUD_VIDEO_DESC_STD_VS(_itf + 1, 0, 1, _stridx), \
    \
    /* VS Input Header: 1 format */ \
    TUD_VIDEO_DESC_CS_VS_INPUT_1FMT( \
        VS_TOTAL_INNER_LEN, \
        _epin, \
        UVC_ENTITY_CAP_OUTPUT_TERMINAL \
    ), \
    \
    /* Format 1: Y16 uncompressed */ \
    TUD_VIDEO_DESC_CS_VS_FMT_Y16(1, Y16_FRAME_COUNT, 1, 0, 0, 0, 0), \
    \
    /* Frame 1: primary resolution (640x480), fixed frame rate */ \
    TUD_VIDEO_DESC_CS_VS_FRM_UNCOMPR_CONT( \
        1,                                              /* bFrameIndex         */ \
        0,                                              /* bmCapabilities      */ \
        THERMAL_WIDTH, THERMAL_HEIGHT,                  /* wWidth, wHeight     */ \
        THERMAL_WIDTH * THERMAL_HEIGHT * 2,             /* dwMinBitRate        */ \
        THERMAL_WIDTH * THERMAL_HEIGHT * 2 * THERMAL_FPS, /* dwMaxBitRate    */ \
        THERMAL_WIDTH * THERMAL_HEIGHT * 2,             /* dwMaxFrameBufferSz  */ \
        FI(THERMAL_FPS),                                /* dwDefaultInterval   */ \
        FI(THERMAL_FPS),                                /* dwMinInterval       */ \
        FI(THERMAL_FPS),                                /* dwMaxInterval       */ \
        0                                               /* dwFrameIntervalStep */ \
    ), \
    \
    /* Frame 2: alternate resolution (384x288), same frame rate */ \
    TUD_VIDEO_DESC_CS_VS_FRM_UNCOMPR_CONT( \
        2,                                              /* bFrameIndex         */ \
        0,                                              /* bmCapabilities      */ \
        THERMAL_WIDTH2, THERMAL_HEIGHT2,                /* wWidth, wHeight     */ \
        THERMAL_WIDTH2 * THERMAL_HEIGHT2 * 2,           /* dwMinBitRate        */ \
        THERMAL_WIDTH2 * THERMAL_HEIGHT2 * 2 * THERMAL_FPS, /* dwMaxBitRate  */ \
        THERMAL_WIDTH2 * THERMAL_HEIGHT2 * 2,           /* dwMaxFrameBufferSz  */ \
        FI(THERMAL_FPS),                                /* dwDefaultInterval   */ \
        FI(THERMAL_FPS),                                /* dwMinInterval       */ \
        FI(THERMAL_FPS),                                /* dwMaxInterval       */ \
        0                                               /* dwFrameIntervalStep */ \
    ), \
    \
    /* Bulk endpoint (512-byte packets on HS) */ \
    TUD_VIDEO_DESC_EP_BULK(_epin, 512, 1)

#endif /* _USB_DESCRIPTORS_H_ */
