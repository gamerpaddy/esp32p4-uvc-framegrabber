/*
 * SPDX-FileCopyrightText: 2019 Ha Thach (tinyusb.org)
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: MIT
 *
 * TinyUSB configuration for multi-format UVC webcam.
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#include "sdkconfig.h"
#include "uvc_frame_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Board Specific ---- */
#ifdef CONFIG_TINYUSB_RHPORT_HS
#   define CFG_TUSB_RHPORT1_MODE    (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#else
#   define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#endif

/* ---- Common ---- */
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef ESP_PLATFORM
#define ESP_PLATFORM 1
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_FREERTOS
#endif

#if TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3, OPT_MCU_ESP32P4)
#define CFG_TUSB_OS_INC_PATH    freertos/
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

#define CFG_TUD_ENABLED       1

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

/* ---- Device ---- */
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

/*
 * DWC2 internal DMA. Without this TinyUSB defaults to SLAVE mode, where the
 * CPU hand-feeds every 512-byte packet through the FIFO in the USB interrupt
 * handler — measured ceiling ~8 MB/s, which caps a 640x480 Y16 stream at
 * ~13 fps. With internal DMA the controller streams whole transfers itself.
 * Setting this =1 also flips the port defaults: SLAVE_ENABLE goes to 0 and
 * CFG_TUD_MEM_DCACHE_ENABLE goes to 1 with 64-byte lines (matches the P4 L1
 * cache), so endpoint buffers get cache-line alignment + msync automatically.
 */
#define CFG_TUD_DWC2_DMA_ENABLE  1

/* ---- CDC-ACM (virtual COM port bridged to the camera UART) ---- */
#define CFG_TUD_CDC              1
#define CFG_TUD_CDC_RX_BUFSIZE   512
#define CFG_TUD_CDC_TX_BUFSIZE   512
#define CFG_TUD_CDC_EP_BUFSIZE   512   /* HS bulk packet size */

/* ---- Video Class ---- */
#define CFG_TUD_VIDEO            1
#define CFG_TUD_VIDEO_STREAMING  1

/* Bulk transfer for HS */
#define CFG_TUD_VIDEO_STREAMING_BULK  1

/*
 * Internal buffer for video payload assembly. For bulk mode, this also caps
 * dwMaxPayloadTransferSize in PROBE responses. Must be >= wMaxPacketSize.
 * The actual endpoint descriptor uses 512 (hardcoded in usb_descriptors.h).
 */
/* 32 KB halves the number of payload chunks per frame vs 16 KB (a 600 KB
 * frame is ~19 chunks instead of 38), cutting per-chunk task/IRQ round-trips.
 * Do not exceed 32 KB: usbd_edpt_xfer takes a uint16_t length. */
#ifdef CONFIG_TINYUSB_RHPORT_HS
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE  (32 * 1024)
#else
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE  64
#endif

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
