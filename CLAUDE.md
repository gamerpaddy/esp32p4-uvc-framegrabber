# ESP32-P4 Thermal Camera UVC Bridge — AI Session Documentation

## Project Goal
Pure parallel-video-to-USB-UVC bridge. A 14-bit monochrome thermal camera outputs raw parallel video (DVP interface). The ESP32-P4 captures it via DVP and streams it as **Y16 uncompressed UVC** over USB to a host PC. **Zero processing on the ESP32** — no AGC, no encoding, no colormap. The host PC does all of that in Python/OpenCV.

---

## Hardware

### Thermal Camera Interface
| Signal | Value | Notes |
|--------|-------|-------|
| Data width | 14-bit monochrome | D[13:0], each pixel = thermal ADC count |
| PCLK | ~15 MHz | Camera drives it (ESP32-P4 is slave) |
| VSYNC | ~50 Hz | = frame rate |
| HSYNC | ~14 kHz | 14000/50 = 280 total lines (active + blanking) |
| Resolution | ~384×288 | 288×50=14400 Hz HSYNC, 15M/14400≈1042 clocks/line |
| Data format | 14-bit unsigned, no I2C/control | Device is a pure video source |

### GPIO Pin Assignments (user-confirmed, hardwired)
| Signal | GPIO |
|--------|------|
| PCLK | 48 |
| VSYNC | 49 |
| HSYNC | 50 |
| D0 (LSB, bit 0) | 15 |
| D1 (bit 1) | 14 |
| D2 (bit 2) | 13 |
| D3 (bit 3) | 12 |
| D4 (bit 4) | 11 |
| D5 (bit 5) | 10 |
| D6 (bit 6) | 9 |
| D7 (bit 7) | 6 |
| D8 (bit 8) | 5 |
| D9 (bit 9) | 4 |
| D10 (bit 10) | 3 |
| D11 (bit 11) | 54 |
| D12 (bit 12) | 53 |
| D13 (MSB, bit 13) | 52 |
| D14 | GND (not connected to ESP32) |
| D15 | GND (not connected to ESP32) |

DVP configured in **16-bit mode** (`cam_data_width=16`, `CAM_CTLR_COLOR_RGB565`). D14/D15 are grounded so each captured `uint16_t` = `0x0000 | thermal14`. This is natively Y16 format. No bit manipulation needed.

### Board
- **SoC:** ESP32-P4 (dual-core RISC-V 400 MHz)
- **PSRAM:** 32 MB (hex mode 200 MHz) — frame buffers live here
- **USB:** High-Speed bulk via dedicated DP/DN pins + external USB PHY
- **IDF version in use:** v6.0.1

---

## Software Architecture

### Data Flow
```
Thermal cam → PCLK/VSYNC/HSYNC + D[13:0]
  → esp_cam_ctlr_dvp (16-bit DMA)
  → PSRAM frame buffer (uint16_t per pixel, bits[13:0] = thermal ADC)
  → UVC transfer buffer (memcpy by TinyUSB driver)
  → USB HS bulk → host PC
```

### Task Architecture
| Task | Core | Priority | Role |
|------|------|----------|------|
| `UVC` (in usb_device_uvc.c) | Kconfig | configurable | Calls `on_fb_get()` → blocks on `ready_q`; calls `tud_video_n_frame_xfer()` |
| `TinyUSB` (in usb_device_uvc.c) | Kconfig | configurable | Runs `tud_task()` loop |
| DVP ISR callbacks | — | ISR | `on_get_new_trans` feeds DMA; `on_trans_finished` pushes to `ready_q` |

Note: there is **no capture task**. `esp_cam_ctlr_receive()` is not implemented by the IDF 6.x DVP driver — it is callback-only.

### Buffer Management (camera_pipeline.c)
```
free_q  [buf0, buf1]  ←───────────────────────────────────────────┐
    │                                                               │
    ▼ on_get_new_trans ISR pops void* from free_q                  │
DVP DMA writes frame into buf                                      │
    │ frame complete                                               │
    ▼ on_trans_finished ISR                                        │
ready_q  [void*]  (size=1, drops stale frame back to free_q)      │
    │                                                              │
    ▼ on_fb_get() (UVC task) pops                                 │
UVC transmits buf over USB                                        │
    │                                                             │
on_fb_return() ────────────────────────────────────────────────►─┘
```
- Queues hold `void *` buffer pointers (not indices) — ISR-safe by value
- `bk_buffer_dis = 0`: driver keeps a backup buffer; when `free_q` is empty the ISR lets `trans->buffer` stay NULL and the driver uses the backup. `on_trans_finished` detects backup frames (pointer not in `ctx->bufs[]`) and discards them
- Both callbacks are `IRAM_ATTR` (required by the driver — validated at registration)

---

## File Map

### Files You Own (main/)
| File | Purpose |
|------|---------|
| `Kconfig.projbuild` | Thermal geometry (width/height/fps) + all 17 DVP GPIO numbers |
| `camera_pipeline.h` | `camera_ctx_t` struct + API: `camera_open/start/stop/get_frame/release_frame` |
| `camera_pipeline.c` | DVP init (3-phase), ISR callbacks, queue-based double-buffering; no capture task |
| `uvc_streaming.h` | `uvc_stream_ctx_t` — camera ctx + UVC fb + perf counters |
| `uvc_streaming.c` | UVC callbacks: `on_stream_start/stop/fb_get/fb_return`; no processing |
| `app_main.c` | `camera_init()` then `uvc_stream_init()` then returns |
| `CMakeLists.txt` | `SRCS`: app_main, camera_pipeline, uvc_streaming. `PRIV_REQUIRES`: esp_driver_cam, esp_driver_gpio, espressif__tinyusb, usb_device_uvc, esp_timer, esp_hw_support |
| `idf_component.yml` | Depends on `idf>=5.4` and local `usb_device_uvc`; **no esp_video** |
| `uvc_controls.h/.c` | **NOT compiled** (not in SRCS). Dead code from old ISP design. Ignore. |
| `uvc_descriptors.h` | **NOT used**. Dead code from old multi-format design. Ignore. |

### Files You Own (components/usb_device_uvc/tusb/)
| File | Purpose |
|------|---------|
| `uvc_frame_config.h` | Defines `THERMAL_WIDTH/HEIGHT/FPS`, `Y16_FRAME_COUNT=1`, `uvc_get_frame_info()` — used by both `usb_device_uvc.c` and `uvc_streaming.c` |
| `usb_descriptors.h` | Defines `TUD_VIDEO_CAPTURE_DESCRIPTOR_THERMAL_BULK` macro + `CONFIG_TOTAL_LEN` + `UVC_DESC_TOTAL_LEN`. **This is the real descriptor**, not `main/uvc_descriptors.h`. |
| `usb_descriptors.c` | Calls `TUD_VIDEO_CAPTURE_DESCRIPTOR_THERMAL_BULK` for the config descriptor; string "Thermal Bridge" |
| `tusb_config.h` | TinyUSB compile-time config (EP sizes, task priorities, HS mode) |

### Component Files (components/usb_device_uvc/)
| File | Purpose |
|------|---------|
| `CMakeLists.txt` | `REQUIRES esp_timer esp_hw_support`. Adds `tusb/` to TinyUSB's PUBLIC include dirs; compiles `tusb/usb_descriptors.c` as part of TinyUSB. |
| `usb_device_uvc.c` | TinyUSB task + video task + Probe/Commit handling. Calls `usb_new_phy(USB_PHY_TARGET_UTMI)` before `tusb_init()` — required to enable the USB OTG-HS clock on ESP32-P4. |
| `include/usb_device_uvc.h` | Public API: `uvc_device_config/init/deinit`, `uvc_fb_t`, `uvc_format_t`, `uvc_xu_set_default` |

---

## UVC Descriptor Design

### Format Advertised
**Single format: Y16 uncompressed**
- GUID: `59 31 36 20 00 00 10 00 80 00 00 AA 00 38 9B 71` (FourCC "Y16 ")
- 16 bits per pixel, little-endian
- Bits [13:0] = thermal ADC value (0–16383)
- Bits [15:14] = always 0 (D14/D15 grounded)
- One frame size: `THERMAL_WIDTH × THERMAL_HEIGHT` at `THERMAL_FPS`

### Descriptor Hierarchy (bulk, single alt-setting)
```
IAD
VideoControl Interface
  Camera Terminal (IT, no controls)
  Processing Unit (PU, bmControls=0x00,0x00,0x00 — no controls)
  Output Terminal (OT → PU)
VideoStreaming Interface alt-0 (with bulk endpoint, 1 endpoint = bulk-only mode)
  VS Input Header (bNumFormats=1, bControlSize=1)
  Format 1: Y16 Uncompressed (GUID above)
  Frame 1: THERMAL_WIDTH × THERMAL_HEIGHT @ THERMAL_FPS
  Bulk Endpoint IN 0x81 (512 byte packets)
```

### Key Length Constants (in usb_descriptors.h)
```c
VS_INPUT_HDR_LEN = 13 + UVC_NUM_FORMATS*1 = 14   // UVC_NUM_FORMATS=1
VC_TOTAL_INNER_LEN = CAMERA_TERM + PU + OT        // no XU
VS_TOTAL_INNER_LEN = FMT_UNCOMPR + 1*FRM_UNCOMPR_CONT
UVC_DESC_TOTAL_LEN = IAD + STD_VC + CS_VC+1 + VC_INNER + STD_VS + VS_HDR + VS_INNER + 7
CONFIG_TOTAL_LEN   = TUD_CONFIG_DESC_LEN + UVC_DESC_TOTAL_LEN
```

### Y16 GUID Macro (defined in usb_descriptors.h)
```c
#define TUD_VIDEO_GUID_Y16 \
    0x59U,0x31U,0x36U,0x20U,0x00U,0x00U,0x10U,0x00U, \
    0x80U,0x00U,0x00U,0xAAU,0x00U,0x38U,0x9BU,0x71U
```

---

## Key Design Decisions

1. **Y16 over MJPEG/H264**: Zero processing on ESP32. The host gets exact 14-bit thermal ADC counts. MJPEG would be lossy. H264 would be lossy.

2. **16-bit DVP mode**: `cam_data_width=16` + `CAM_CTLR_COLOR_RGB565` makes the DMA transfer 2 bytes/pixel. D14/D15 are grounded so the captured words are already Y16-compatible. `external_xtal=1` suppresses XCLK output — the thermal cam drives PCLK itself. `HSYNC` maps to `de_io` (HREF/data-enable) because the IDF 6.x DVP driver has no separate HSYNC pin concept.

3. **Drop stale frames, not new frames**: `ready_q` size=1 with drop-old policy. If UVC is momentarily slow, we prefer fresh thermal data over old data.

4. **USB OTG-HS UTMI PHY must be initialized explicitly**: The managed TinyUSB's `dwc2_phy_init()` for ESP32-P4 is a no-op (source comment: "// maybe usb_utmi_hal_init()"). Without calling `usb_new_phy()` first, `dcd_init()` crashes with a load access fault at `0x50000048` (USB OTG-HS register base) because the peripheral clock is not enabled. Fix: call `usb_new_phy()` with `USB_PHY_TARGET_UTMI` / `USB_PHY_SPEED_HIGH` / `USB_OTG_MODE_DEVICE` in `uvc_device_init()` before `tusb_init()`. Store handle; call `usb_del_phy()` in deinit. Header: `esp_private/usb_phy.h`. The old `usb` component (IDF 5.x) is still gone — this uses the IDF 6.x `esp_hw_support` component.

5. **No `esp_video` / V4L2**: The old project used the high-level V4L2 framework which requires I2C sensor init. We use `esp_cam_ctlr_dvp` directly — the lowest-level DVP API, no sensor init needed.

6. **`camera_apply_isp_profile` removed**: Declared in old `camera_pipeline.h`, called from old `uvc_controls.c`. Both are gone. Don't re-add.

---

## Build System Notes

### IDF 6.0.1 Compatibility Issues Resolved
| Issue | Root Cause | Fix Applied |
|-------|-----------|-------------|
| `Failed to resolve component 'usb'` | `usb` component reorganized in IDF 6.x | Removed `usb` from `REQUIRES` in `usb_device_uvc/CMakeLists.txt`; removed all `usb_phy.h` usage |
| `has no member named 'queue_items'` / `has no member named 'io'` | IDF 6.x completely reorganized `esp_cam_ctlr_dvp_config_t` | Moved pins into separate `esp_cam_ctlr_dvp_pin_config_t pin_cfg`; removed `queue_items`; added `cam_data_width=16`, `output_data_color_type`, `external_xtal=1`, `bk_buffer_dis=1`; renamed `hsync→de_io`, `pclk→pclk_io`, `vsync→vsync_io` |
| `invalid argument: data_width != CAM_CTLR_DATA_WIDTH_8` | Driver enforces 8-bit pin config, but hardware has 16 data inputs | Three-phase init: call `esp_cam_ctlr_dvp_init()` with 8-bit config (D0-D7) to pass validation + enable RCC, manually route D8-D13 via GPIO matrix, then call `esp_cam_new_dvp_ctlr()` with `pin_dont_init=1` + `cam_data_width=16` |
| Load access fault at `0x50000048` in `dcd_init` / `dwc2_core_is_highspeed` | Managed TinyUSB `dwc2_phy_init()` for ESP32-P4 is a no-op — USB OTG-HS peripheral clock never enabled | Call `usb_new_phy()` with `USB_PHY_TARGET_UTMI` + `USB_PHY_SPEED_HIGH` before `tusb_init()` in `uvc_device_init()`; add `esp_hw_support` to `usb_device_uvc/CMakeLists.txt` REQUIRES |
| `assert failed: "no new buffer, and no driver internal buffer"` in `esp_cam_ctlr_dvp_start_trans` | DVP driver is callback-only — `esp_cam_ctlr_receive()` has no implementation; driver needs a buffer at `start()` time | Removed blocking capture task; switched to ISR callbacks (`on_get_new_trans` / `on_trans_finished`, both `IRAM_ATTR`); set `bk_buffer_dis=0` so driver has backup buffer when `free_q` is momentarily empty |
| `DVP callback registration failed: ESP_ERR_INVALID_STATE` | `esp_cam_ctlr_register_event_callbacks()` only succeeds when `dvp_fsm == INIT`; calling it after `esp_cam_ctlr_enable()` fails silently | Move callback registration to `camera_open()` — after `esp_cam_new_dvp_ctlr()` but BEFORE `esp_cam_ctlr_enable()` |

### IDF 6.x DVP API Shape (confirmed against `esp_cam_ctlr_dvp.h` + runtime)
The driver enforces `pin->data_width == CAM_CTLR_DATA_WIDTH_8` in the GPIO init path, even though the ESP32-P4 hardware exposes 16 data inputs in the GPIO matrix (`CAM_DATA_IN_PAD_IN0..15`). The required three-phase workaround in `camera_pipeline.c`:

```c
// Phase 1: 8-bit config passes validation, enables RCC clock, routes D0-D7 + controls
#include "esp_private/esp_cam_dvp.h"   // esp_cam_ctlr_dvp_init()
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"          // CAM_DATA_IN_PAD_IN8_IDX..13_IDX

esp_cam_ctlr_dvp_pin_config_t pin_cfg = {
    .data_width = CAM_CTLR_DATA_WIDTH_8,   // 8 = passes driver check
    .data_io    = { D0..D7 GPIOs },
    .vsync_io   = CONFIG_DVP_VSYNC_GPIO,
    .de_io      = CONFIG_DVP_HSYNC_GPIO,   // HREF/HSYNC = data-enable
    .pclk_io    = CONFIG_DVP_PCLK_GPIO,
    .xclk_io    = GPIO_NUM_NC,
};
esp_cam_ctlr_dvp_init(0, CAM_CLK_SRC_DEFAULT, &pin_cfg);

// Phase 2: manually route D8-D13 (same sequence the driver uses internally)
gpio_set_direction(gpio, GPIO_MODE_INPUT);
gpio_set_pull_mode(gpio, GPIO_FLOATING);
esp_rom_gpio_connect_in_signal(gpio, CAM_DATA_IN_PAD_IN8_IDX + offset, false);
// ... for D9..D13

// Phase 3: create controller with 16-bit DMA, skip pin init (done above)
esp_cam_ctlr_dvp_config_t dvp_cfg = {
    .cam_data_width  = 16,    // DMA captures 16 bits per PCLK from 16 GPIO signals
    .external_xtal   = 1,     // thermal cam drives PCLK
    .pin_dont_init   = 1,     // we did GPIO init above
    .pin             = NULL,
    .bk_buffer_dis   = 0,     // keep driver backup buffer for when free_q is momentarily empty
    .pin_dont_init   = 1,     // we did GPIO init above
    .pin             = NULL,
    // ... other fields
};

// Callbacks MUST be registered BEFORE esp_cam_ctlr_enable() — driver only
// accepts registration when dvp_fsm == INIT; enable() advances the FSM past it.
esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans  = camera_get_new_trans_cb,   // IRAM_ATTR
    .on_trans_finished = camera_trans_finished_cb,  // IRAM_ATTR
};
esp_cam_ctlr_register_event_callbacks(ctx->ctlr, &cbs, ctx);
esp_cam_ctlr_enable(ctx->ctlr);

// camera_start() just calls esp_cam_ctlr_start() — no task created
```

**Also add `esp_driver_gpio` to `main/CMakeLists.txt` PRIV_REQUIRES** for `driver/gpio.h`.

### Potential Future Build Issues
| Error | Likely Fix |
|-------|-----------|
| `TUD_VIDEO_DESC_EP_BULK` undefined | Check TinyUSB `video.h` for the correct bulk endpoint macro name |
| `tusb_teardown` undefined | Delete the `tusb_teardown()` call in `uvc_device_deinit()` |
| `MALLOC_CAP_DMA` rejected for PSRAM | On ESP32-P4, PSRAM is DMA-accessible; remove `MALLOC_CAP_DMA` flag if IDF rejects the combination |

### Component Dependency Chain
```
main → esp_driver_cam    (DVP driver, esp_cam_ctlr_dvp.h, esp_private/esp_cam_dvp.h)
     → esp_driver_gpio   (driver/gpio.h — for manual D8-D13 GPIO routing)
     → espressif__tinyusb
     → usb_device_uvc
     → esp_timer
     → esp_hw_support    (heap_caps)

usb_device_uvc → esp_timer         (get_time_millis)
              → esp_hw_support     (esp_private/usb_phy.h — usb_new_phy/usb_del_phy)
              → espressif__tinyusb (tusb.h, tud_video_*, etc.)
```

---

## Kconfig Symbols Reference

```
CONFIG_THERMAL_WIDTH       default 384
CONFIG_THERMAL_HEIGHT      default 288
CONFIG_THERMAL_FPS         default 50

CONFIG_DVP_PCLK_GPIO       default 48
CONFIG_DVP_VSYNC_GPIO      default 49
CONFIG_DVP_HSYNC_GPIO      default 50
CONFIG_DVP_D0_GPIO  = 15   CONFIG_DVP_D7_GPIO  = 6
CONFIG_DVP_D1_GPIO  = 14   CONFIG_DVP_D8_GPIO  = 5
CONFIG_DVP_D2_GPIO  = 13   CONFIG_DVP_D9_GPIO  = 4
CONFIG_DVP_D3_GPIO  = 12   CONFIG_DVP_D10_GPIO = 3
CONFIG_DVP_D4_GPIO  = 11   CONFIG_DVP_D11_GPIO = 54
CONFIG_DVP_D5_GPIO  = 10   CONFIG_DVP_D12_GPIO = 53
CONFIG_DVP_D6_GPIO  = 9    CONFIG_DVP_D13_GPIO = 52
```

---

## Runtime Debugging

### Serial Log on Success
```
I thermal_dvp: DVP open: 384x288 @50 fps, 221184 bytes/frame
I thermal_dvp: DVP capture running
I uvc_stream:  Thermal bridge ready: 384x288 Y16 @50 fps, 221184 B/frame
I usbd_uvc:    Mount                        ← USB connected
I usbd_uvc:    Commit: bFormatIndex=1 bFrameIndex=1 dwFrameInterval=200000
I uvc_stream:  Stream start: 384x288 @50fps (Y16)
```

### If No Frames Arrive — Diagnostic Counters

`camera_pipeline.c` logs three ISR counters on every `frame timeout`:
```
W thermal_dvp: frame timeout — get_new=N finished_ours=N finished_backup=N
```

| `get_new` | `finished_ours` | `finished_backup` | Meaning |
|-----------|-----------------|-------------------|---------|
| 0 | 0 | 0 | DMA ISR never fires. Camera not connected/powered, or VSYNC/PCLK polarity completely wrong so the DVP controller never detects a frame boundary. |
| 1 | 0 | 0 | `on_get_new_trans` fired once (initial start), but frame never completed. PCLK toggling but no valid VSYNC, or PCLK count wrong. |
| >1 | 0 | >0 | DMA completes but all frames land in backup buffer. `free_q` is always empty at callback time — logic bug in buffer management. |
| >1 | >0 | any | Frames reach `ready_q` correctly — problem is in the consumer path (UVC side). |

**VSYNC polarity:** The IDF DVP driver connects VSYNC with `inv=true` (GPIO-matrix inversion). If camera outputs active-HIGH VSYNC, the controller sees active-LOW, which may not match what the hardware expects. Try toggling VSYNC polarity by connecting it with `inv=false` in the `DVP_CAM_CONFIG_INPUT_PIN` call inside `esp_cam_ctlr_dvp_init()` — but that requires patching the private driver or pre-routing the pin with an inverted connection.

**HSYNC vs HREF:** We map `CONFIG_DVP_HSYNC_GPIO` to `de_io` (data-enable, expected active-HIGH during valid pixels). If the camera outputs HREF (active-HIGH during valid data), this is correct. If it outputs classic HSYNC (narrow active-LOW pulse at line start), `de_io` sees mostly-LOW and the DVP captures nothing. In that case, set `de_io = GPIO_NUM_NC` and rely solely on VSYNC + pixel count for framing.

**PCLK edge:** Data is sampled on rising PCLK by default. If the camera drives data on the rising edge and it should be sampled on the falling edge, try setting `byte_swap_en = true` in `dvp_cfg`.

**Verify D14/D15:** Must be tied to GND (not floating — floating injects noise into bits[15:14]).

### If Y16 Doesn't Show in Windows Camera App
Windows Camera app won't display Y16. Use Python (see below) or VLC or OBS with a custom source.

---

## Host PC Python Decode

```python
import cv2
import numpy as np

W, H = 384, 288  # match CONFIG_THERMAL_WIDTH/HEIGHT

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # get raw bytes, no RGB conversion
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

while True:
    ret, raw = cap.read()
    if not ret:
        break

    # raw is a flat byte array; interpret as uint16 little-endian
    frame14 = np.frombuffer(raw, dtype='<u2').reshape(H, W)
    frame14 = frame14 & 0x3FFF   # mask upper 2 zero bits (optional but explicit)

    # Linear AGC for display
    lo, hi = frame14.min(), frame14.max()
    if hi > lo:
        display = ((frame14 - lo) * 255 // (hi - lo)).astype(np.uint8)
    else:
        display = np.zeros((H, W), dtype=np.uint8)

    # False color (optional)
    color = cv2.applyColorMap(display, cv2.COLORMAP_INFERNO)

    cv2.imshow('Thermal 14-bit', color)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
```

On Linux with V4L2: `cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)` and set format via `v4l2-ctl --set-fmt-video=pixelformat=Y16`.

---

## What Was Removed vs Original Project
- OV5647 MIPI CSI camera → replaced with DVP thermal capture
- ISP / Bayer demosaic / color correction → removed entirely
- RTSP/Ethernet streaming → removed entirely
- 3 UVC formats (UYVY + MJPEG + H264) → 1 format (Y16)
- Extension Unit (ISP color profile XU) → removed
- PU controls (brightness/contrast/hue/saturation/WB) → removed (bmControls=0)
- `uvc_controls.c` → dead code, not compiled
- `esp_video` V4L2 framework → replaced with `esp_driver_cam` directly
- `esp_cam_sensor` component → removed (no I2C sensor)
- Explicit USB PHY init with old `usb_new_phy()` API → replaced with `usb_new_phy(USB_PHY_TARGET_UTMI)` from `esp_private/usb_phy.h` (IDF 6.x); call moved into `uvc_device_init()` before `tusb_init()`
