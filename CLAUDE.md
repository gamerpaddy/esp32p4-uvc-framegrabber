# ESP32-P4 Thermal Camera Framegrabber — AI Session Documentation

## Project Goal
Capture a thermal camera's raw parallel (DVP) video on an ESP32-P4 and stream it
to a PC as **Y16 uncompressed UVC**. The USB path does **zero processing**: no
AGC, no encoding, no colormap, so the host receives the camera's real sensor
counts. Host-side processing lives in `cam_viewer.py`.

Secondary outputs, added later, DO process the image because they have to:
- A **browser view over Wi-Fi** (MJPEG). Encodes because raw Y16 does not fit
  the radio and no browser renders Y16.
- A **UART control channel** to the camera, driven from the Python viewer.

---

## Hardware

### Camera
A **Dali D8X3C** thermal module (see `hardware/`). It is a pure video source:
it drives PCLK/HSYNC/VSYNC and has no I2C. Control is via UART only.

| Signal | Value | Notes |
|--------|-------|-------|
| Data width | 14-bit | D[13:0]; D14/D15 tied to GND so each captured `uint16_t` is already Y16 |
| PCLK | ~15 MHz | Camera drives it; the P4 is a slave (`external_xtal=1` suppresses XCLK) |
| VSYNC | ~50 Hz | One capture per VSYNC |
| Resolution | 384x288 | |

**The source is interleaved.** It alternates two frame phases every VSYNC:

- **even captures** = 8-bit visible-light video (values fit in the low 8 bits)
- **odd captures**  = 14-bit thermal data (values >0xFF, baseline ~7000 counts)

So the 50 Hz VSYNC rate yields a **real thermal rate of ~25 fps**. The camera's
on-screen menu is only drawn into the 8-bit video phase, which is why the viewer
switches phase when you press a menu key.

### GPIO Pin Assignments

**These are the values in `sdkconfig`, which is what the board actually uses.**
The `default`s in `main/Kconfig.projbuild` are stale leftovers from an earlier
board and do NOT match the hardware. Trust this table and `sdkconfig`.

| Signal | GPIO | | Signal | GPIO |
|--------|------|-|--------|------|
| PCLK   | 30 | | D4  | 45 |
| VSYNC  | 31 | | D5  | 46 |
| HSYNC  | 32 | | D6  | 47 |
| D0     | 41 | | D7  | 48 |
| D1     | 42 | | D8  | 40 |
| D2     | 43 | | D9  | 39 |
| D3     | 44 | | D10 | 38 |
|        |    | | D11 | 37 |
|        |    | | D12 | 34 |
|        |    | | D13 | 33 |
| D14/D15 | GND | | | |

| Other | Pins |
|-------|------|
| Camera UART1 | TX=29, RX=28 @ 38400 8N1 |
| SDIO to C6 | CLK=18 CMD=19 D0..D3=14,15,16,17, reset=54 |
| Console | USB-Serial-JTAG (NOT UART0: GPIO37/38 are DVP D11/D10) |

### Board
- **SoC:** ESP32-P4 (dual-core RISC-V, running at 360 MHz)
- **PSRAM:** 32 MB hex mode 200 MHz — all frame buffers live here
- **Radio:** on-module **ESP32-C6** over SDIO (the P4 has no Wi-Fi), via ESP-Hosted
- **USB:** High-Speed bulk, external UTMI PHY
- **IDF:** v6.0.1

---

## Software Architecture

### Data Flow
```
Thermal cam ──DVP──> esp_cam_ctlr_dvp (16-bit DMA) ──> PSRAM buffer pool (6)
                                                          │
                        ┌─────────────────────────────────┤
                        ▼                                 ▼
              UVC task (prio 23)                  observer tap (memcpy)
              raw Y16, no processing                      │
                        │                                 ▼
                        ▼                        mjpeg task (prio 5)
                  USB HS bulk                    AGC -> HW JPEG -> HTTP
                        │                                 │
                        ▼                                 ▼
                   PC (cam_viewer.py)              browser over Wi-Fi
```

### Tasks
| Task | Prio | Role |
|------|------|------|
| `UVC` (usb_device_uvc.c) | 23 | `on_fb_get()` -> `camera_get_frame()`, `tud_video_n_frame_xfer()` |
| `TinyUSB` | 24 | `tud_task()` loop |
| `mjpeg` (web_stream.c) | 5 | One per HTTP stream client; AGC + JPEG + send |
| `httpd` | default | Serves `/`, `/stats`, `/snapshot` |
| `cam_uart` | — | UART1 RX pump for camera replies |
| console REPL | low | `join`/`scan`/`sdiospeed`/`wifispeed`/`c6flash`/... |
| DVP ISR callbacks | ISR | `on_get_new_trans`, `on_trans_finished`, both `IRAM_ATTR` |

There is **no capture task**. `esp_cam_ctlr_receive()` is not implemented by the
IDF 6.x DVP driver; it is callback-only.

### Buffer Management (camera_pipeline.c)

**6 buffers, 2 READY slots.** Both numbers are load-bearing:

- **6 buffers**: the driver calls `on_get_new_trans` (start the NEXT capture)
  *before* `on_trans_finished` (publish the just-finished one), so up to five
  buffers are spoken for at that instant and one more must be FREE.
- **2 READY slots**: phases alternate every 20 ms. With a single slot the 8-bit
  video capture evicts the thermal frame 20 ms after it lands, so a consumer
  whose cycle exceeds 20 ms misses every other thermal frame and output halves
  to 12.5 fps. Two slots give the consumer the full 40 ms phase period.

States (`BUF_FREE/WRITING/READY/READING`) transition only under `ctx->lock`.

### Phase selection is by CONTENT, not capture parity
`parity_filter=true`, `keep_parity=1` are hardwired in `camera_open()`. The
classifier counts pixels >0xFF over a subsample and takes a **majority** vote.
An "any pixel >0xFF" test was not enough: a hot glowing object puts a few
>0xFF pixels into the *video* frame, which let the video phase through and
made the view flip whenever something got hot.

### Capture buffer is taller than the image
`DVP_VBLANK_CAPTURE_MARGIN = 64` extra lines. If the DMA buffer is exactly
`w*h*2`, the descriptor chain ends at the last active line *before* the VSYNC
EOF fires, so EOF lands with no active descriptor. That race caused intermittent
freezes and diagonal frame-start shear. Oversizing means VSYNC is the only frame
delimiter. Only the first `w*h*2` bytes are ever delivered.

### Two watchdogs
- **Freeze watchdog**: a camera FFC or PCLK/VSYNC glitch can wedge the CAM so it
  keeps EOF-ing but redelivers stale buffers. Live sensor noise makes every real
  frame's hash unique, so a hash repeating one of the last 8 means frozen.
- **Hard-stall watchdog**: the CAM can stop issuing EOFs entirely. If
  `dbg_finished` does not advance across two consecutive timeouts, stop/start.

### Observer tap (camera_pipeline.c)
A second *consumer* does not work. The UVC task at prio 23 calls straight back
into `camera_get_frame()` after each transfer, so it claims every published
buffer before the prio-5 HTTP task is scheduled. The web stream measured
**exactly zero frames**, not a fair share.

The tap instead **copies** the frame the primary consumer is already taking:
`camera_tap_get()` sets a pending flag and blocks; `camera_get_frame()` does the
memcpy on whoever is consuming. If nothing is consuming (no USB host attached)
the tap never fills and returns false, and the caller falls back to
`camera_get_frame()`, which is uncontended by definition in that case.

---

## File Map

### main/
| File | Purpose |
|------|---------|
| `app_main.c` | camera_init -> uvc_stream_init -> camera_uart_start -> wifi_console_start -> web_stream_start |
| `camera_pipeline.c/.h` | DVP 3-phase init, ISR callbacks, buffer pool, phase filter, watchdogs, observer tap |
| `uvc_streaming.c/.h` | UVC callbacks; no processing; NO SIGNAL checkerboard when DVP is dry |
| `camera_uart.c/.h` | UART1 channel to the camera; framed protocol (STX/len/payload/checksum/ETX) |
| `web_stream.c/.h` | HTTP server: `/`, `/stream` (MJPEG), `/snapshot`, `/stats` |
| `wifi_console.c/.h` | Wi-Fi bring-up over ESP-Hosted + console: `scan`/`join`/`leave`/`status` |
| `net_bench.c` | `sdiospeed` and `wifispeed` throughput benchmarks |
| `c6_debug.c` | `c6mon` (listen to C6 UART0), `c6boot` (reset, `-d` = download mode) |
| `c6_flash.c` | `c6flash` — programs the C6 from the `c6_fw` partition |
| `Kconfig.projbuild` | Geometry, phase filter, polarity, DVP pins. **Defaults are stale, see sdkconfig.** |

### components/usb_device_uvc/
| File | Purpose |
|------|---------|
| `tusb/uvc_frame_config.h` | `THERMAL_WIDTH/HEIGHT` (frame 1) + `THERMAL_WIDTH2/HEIGHT2` (frame 2), `Y16_FRAME_COUNT=2` |
| `tusb/usb_descriptors.h` | `TUD_VIDEO_CAPTURE_DESCRIPTOR_THERMAL_BULK`, length constants. **The real descriptor.** |
| `tusb/usb_descriptors.c` | Config descriptor; product string "Thermal Bridge" |
| `usb_device_uvc.c` | TinyUSB + video task, Probe/Commit. Calls `usb_new_phy(USB_PHY_TARGET_UTMI)` before `tusb_init()` |

### Host / tools
| File | Purpose |
|------|---------|
| `cam_viewer.py` | Tk viewer: palettes, AGC, histogram, PNG/TIFF save, camera UART control |
| `tcp_sink.py` | TCP sink for `wifispeed` |
| `tools/flash_all.ps1` | Build C6 -> pack -> flash P4 -> write `c6_fw` |
| `tools/pack_c6_fw.py` | Packs the four C6 images into `build/c6_fw.bin` |
| `hardware/` | EasyEDA PCB files: Dali-to-FFC adapter, ESP32-P4 backpack |

---

## UVC Descriptor

**Single format Y16, TWO frame sizes advertised.**

- GUID `59 31 36 20 00 00 10 00 80 00 00 AA 00 38 9B 71` ("Y16 ")
- 16 bpp little-endian; bits [13:0] = thermal count, [15:14] always 0
- Frame 1: `CONFIG_THERMAL_WIDTH x CONFIG_THERMAL_HEIGHT` (currently 384x288)
- Frame 2: 384x288 (`THERMAL_WIDTH2/HEIGHT2`, hardcoded)
- Bulk endpoint IN 0x81, single alt-setting

`on_stream_start()` takes whatever the host committed and calls
`camera_set_resolution()`.

Both 384x288 and 640x480 camera models are supported, selected by
`CONFIG_THERMAL_WIDTH/HEIGHT` in sdkconfig. What does NOT work is picking the
other frame size at runtime: a host committing 640x480 against a 384-wide
camera produces garbage, which is why `cam_viewer.py` offers only one entry.

---

## Wi-Fi (ESP-Hosted, C6 over SDIO)

esp_hosted **3.0.1** on both sides; host and coprocessor versions must match
exactly or the link never finishes handshaking. SDIO 4-bit @ 40 MHz, STREAM
mode (not SW_AGGR — that needs an IDF carrying commit `4814514`).

### Measured throughput

| Test | Result |
|------|--------|
| `sdiospeed` disconnected (bus only) | **56.6 Mbit/s** |
| `sdiospeed` connected (over the air) | **11.9 Mbit/s** |
| `wifispeed` (TCP end to end) | ~3.5 Mbit/s when measured |

The bus is not the limit. **The radio is**, and the current ceiling is largely
the improvised antenna. Do not go looking for a firmware cause for the
remaining gap.

### Things that were tried and did NOT help
Recorded so nobody burns another cycle on them:

- **`WIFI_PS_NONE`**: worth keeping (bus-powered board), but it did not fix the
  stall it was aimed at. The byte count came back *bit-identical* with PS off.
- **Disabling SW coexistence** (`ESP_COEX_SW_COEXIST_ENABLE=n`, BT was already
  off): 6.15 -> 6.21 Mbit/s. Noise. Kept because it is free.
  Note `ESP_COEX_ENABLED` cannot be turned off; Kconfig forces it back to `y`.
- **Queue depth beyond 64**: 128 measured 11.87 vs 11.69 Mbit/s. 64 is the knee.
- **Channel change (11 -> 1)**: `sdiospeed` returned *exactly* 5184 frames on
  both channels. Congestion was never the issue.

### The one change that DID help
`CONFIG_EH_TRANSPORT_CP_SDIO_RX_Q_SIZE` and the host TX queue, **20 -> 64**:
host-to-air went **6.21 -> 11.87 Mbit/s**.

Why: the host must see free slave buffers before writing a frame. When it does
not, `sdio_is_write_buffer_available()` backs off with `usleep(400)`, +400 us per
retry. IDF's `usleep` busy-waits below one tick but switches to `vTaskDelay`
above it, so at 1000 Hz the third retry jumps straight from 800 us to 2 ms. That
quantisation pinned TX at ~518 frames/s **regardless of frame size** (2.08 /
4.13 / 6.21 Mbit/s at 500 / 1000 / 1500 B). Size-independence is the tell: a
saturated radio would have dropped frames/s as frames got bigger.

### Diagnosing a TCP stall
A stall that reproduces **to the byte** (~68 KB, i.e. one send buffer, then no
ACKs ever) is not the board. It was `tcp_sink.py` wedged on a half-open
connection from a previous run: the PC kernel completes the handshake from the
listen backlog so `connect()` succeeds, but no application ever calls `recv()`,
the window shuts, and the sender stalls after exactly one buffer. `tcp_sink.py`
now has an idle timeout so it recovers by itself.

`wifispeed` prints the ESP-Hosted throttle state (`wifi_tx_throttling`). If that
ever reads LATCHED, the C6 missed a STOP_THROTTLE and every STA frame is being
silently dropped.

---

## Web Stream (web_stream.c)

`/` viewer page, `/stream` MJPEG, `/snapshot` one JPEG, `/stats` JSON.

### The stream MUST run in its own task
`esp_http_server` dispatches every request from a **single task**, so a handler
that never returns (which an endless multipart response is) blocks the whole
server. Symptom: `/stats` worked in isolation but the index page showed nothing
while video played. Fixed with `httpd_req_async_handler_begin()`, which detaches
the request to an `mjpeg` task so httpd stays free.

A browser reload opens the new stream before the old socket dies. Refusing it
with 503 made a reloaded tab lag, because the old task kept running and kept
taking frames. A new `/stream` now asks the incumbent to stop and waits for it.

`max_open_sockets = 7`: the MJPEG response occupies a socket for its whole life
while `/stats` polls beside it. Too few and `lru_purge` reclaims the stream to
serve the polls reporting on it.

### AGC must be percentile-clipped
Min/max stretch does not work on this sensor. One dead pixel near 0 and one hot
pixel near full scale pin the endpoints to the edges of the 14-bit space, so the
scene (a few hundred counts around a ~7000 baseline) compresses into a narrow
mid-grey band. It renders as "bright and low contrast, as if not normalised at
all" while normalisation is in fact running.

Now clips 0.5% off each tail via a 1024-bin histogram, matching `robust_range()`
in `cam_viewer.py`. Clamping in the mapping loop is mandatory: clipped tails mean
pixels outside `[lo,hi]` exist, and unsigned `(v - lo)` would wrap and speckle.

JPEG encoding uses the **P4 hardware encoder** (`JPEG_ENCODE_IN_FORMAT_GRAY`),
so it costs almost no CPU. The AGC pass is the expensive part.

### /stats
Per-core CPU load, internal heap, PSRAM, stream fps, uptime. Load comes from
FreeRTOS run-time counters (already enabled, already esp_timer-backed, so the
counters are microseconds and compare directly to wall time).
`ulTaskGetRunTimeCounter()` is not exposed in this kernel build, so it walks
`uxTaskGetSystemState()` and matches the idle handles from
`xTaskGetIdleTaskHandleForCore()`. Load is measured *between* polls; the first
reading is 0.

---

## Key Design Decisions

1. **Y16 over MJPEG/H264 for USB**: zero processing, exact counts, nothing lossy.
2. **16-bit DVP mode**: `cam_data_width=16` + `CAM_CTLR_COLOR_RGB565` gives
   2 bytes/pixel; D14/D15 grounded so words are natively Y16.
3. **Drop stale frames, not new ones**: prefer fresh thermal data.
4. **USB OTG-HS UTMI PHY must be initialised explicitly**: managed TinyUSB's
   `dwc2_phy_init()` for the P4 is a no-op, so `dcd_init()` faults at
   `0x50000048` because the peripheral clock is off. Call `usb_new_phy()` with
   `USB_PHY_TARGET_UTMI`/`HIGH`/`DEVICE` before `tusb_init()`.
5. **No `esp_video`/V4L2**: that framework requires I2C sensor init. This camera
   has no I2C, so `esp_cam_ctlr_dvp` is used directly.
6. **Framing polarity is hardwired in `camera_open()`**, NOT taken from Kconfig:
   `vsync_invert=true, de_mode=0, hsync_invert=false, pclk_invert=false`. The
   Kconfig values (notably `DE_MODE=2`) are ignored. Runtime log confirms
   `DE mode 0 / VSYNC invert = 1`.

---

## Build System Notes

### The three-phase DVP init
The IDF 6.x driver enforces `data_width == CAM_CTLR_DATA_WIDTH_8` in its GPIO
init path even though the P4 exposes 16 data inputs in the GPIO matrix:

```c
// Phase 1: 8-bit config passes validation, enables RCC, routes D0-D7 + controls
esp_cam_ctlr_dvp_init(0, CAM_CLK_SRC_DEFAULT, &pin_cfg);   // esp_private/esp_cam_dvp.h
// Phase 2: manually route D8-D13 through the GPIO matrix
esp_rom_gpio_connect_in_signal(gpio, CAM_DATA_IN_PAD_IN8_IDX + offset, false);
// Phase 3: create the controller with 16-bit DMA, pin_dont_init=1, pin=NULL
esp_cam_new_dvp_ctlr(&dvp_cfg, &ctx->ctlr);
```

Callbacks **must** be registered after `esp_cam_new_dvp_ctlr()` but **before**
`esp_cam_ctlr_enable()`; registration only succeeds while `dvp_fsm == INIT`.

### Resolved IDF 6.0.1 issues
| Issue | Fix |
|-------|-----|
| `Failed to resolve component 'usb'` | Reorganised in IDF 6.x; use `esp_hw_support` + `esp_private/usb_phy.h` |
| `esp_cam_ctlr_dvp_config_t` members missing | Pins moved to `pin_cfg`; added `cam_data_width`, `external_xtal`, `bk_buffer_dis`; `hsync`->`de_io` |
| `data_width != CAM_CTLR_DATA_WIDTH_8` | Three-phase init above |
| Load fault at `0x50000048` in `dcd_init` | `usb_new_phy(USB_PHY_TARGET_UTMI)` before `tusb_init()` |
| `assert "no new buffer, and no driver internal buffer"` | Driver is callback-only; use ISR callbacks, `bk_buffer_dis=0` |
| DVP callback registration `ESP_ERR_INVALID_STATE` | Register before `esp_cam_ctlr_enable()` |
| `WIFI_BW_HT40` undeclared | Renamed to `WIFI_BW40` in IDF 6.x |
| No `HTTPD_503_SERVICE_UNAVAILABLE` | Not in `httpd_err_code_t`; use `httpd_resp_set_status(req, "503 ...")` |

### Component dependencies
```
main -> esp_driver_cam, esp_driver_gpio, esp_driver_uart, esp_driver_jpeg,
        espressif__tinyusb, usb_device_uvc, esp_timer, esp_hw_support, esp_mm,
        esp_wifi, esp_netif, esp_event, nvs_flash, lwip, console,
        esp_http_server, esp_partition, espressif__esp-serial-flasher
usb_device_uvc -> esp_timer, esp_hw_support, espressif__tinyusb
```

### sdkconfig is generated and gitignored
`sdkconfig.defaults` (and `c6_firmware/sdkconfig.defaults`) are the source of
truth. Editing `sdkconfig` alone is lost on a `fullclean`; editing
`sdkconfig.defaults` alone has **no effect** while a `sdkconfig` already exists.
When changing config, do both, then verify the value survived the rebuild.

---

## Build and Flash

```powershell
.\tools\flash_all.ps1 -Port COM30      # C6 build -> pack -> P4 flash -> write c6_fw
```

Then program the C6 itself from the P4 console (the C6 has no USB of its own):
```
idf.py -p COM30 monitor
p4> c6boot -d      # confirm ROM download mode
p4> c6flash        # ~15 s
```
Only needed when the C6 firmware changed. P4-only rebuild:
`idf.py -p COM30 flash monitor`.

---

## Runtime Debugging

### Healthy boot
```
thermal_dvp: VSYNC invert = 1 / DE mode 0 / HSYNC invert = 0 / PCLK invert = 0
thermal_dvp: DVP open: 384x288 @50 fps out, 270336 bytes/frame, parity_filter=1 keep_parity=1
uvc_stream:  Thermal bridge ready: 384x288 Y16 @50 fps, 221184 B/frame
cam_uart:    UART1 up: TX=IO29 RX=IO28 @ 38400 8N1
wifi_con:    Wi-Fi (C6 over SDIO) up in STA mode, power save off
web_stream:  HTTP server up on :80
usbd_uvc:    Mount / Commit / Starting: 384x288 @50fps
```
Note 270336 (capture, includes the vblank margin) vs 221184 (delivered).

### No frames: the ISR counters
```
W thermal_dvp: frame timeout — get_new=N finished=N no_free=N recv=N/N
```
| `get_new` | `finished` | Meaning |
|-----------|-----------|---------|
| 0 | 0 | ISR never fires. Camera unpowered, or VSYNC polarity so wrong no frame boundary is seen. |
| 1 | 0 | Started but never completed. PCLK toggling, no valid VSYNC. |
| >1 | >1 | Captures work; the problem is downstream (consumer or phase filter). |

If `finished` climbs but a *viewer* sees nothing, suspect the consumer, not the
DVP. That is exactly how the web stream's zero-frame bug presented.

### Console commands
`scan` `join <ssid> [pass]` `leave` `status` `sdiospeed [-t s] [-l bytes]`
`wifispeed <ip> [-t s]` `c6mon` `c6boot [-d]` `c6flash`

`status` prints `phy: not reported by the coprocessor` — `esp_wifi_remote` does
not marshal the `phy_*` bits or `rssi` across the RPC. An all-zero PHY line
means "not forwarded", **not** "the link is 11b". Use `scan` for RSSI.

### Y16 in other apps
Windows Camera will not display Y16. Use `cam_viewer.py`, or the web view.
On Linux: `v4l2-ctl --set-fmt-video=pixelformat=Y16`.

---

## Host Viewer (cam_viewer.py)

Tk + OpenCV. Settings persist by rewriting the `SETTINGS` block in the file
itself, no external config.

- The viewer's resolution list is pinned to **384x288**. The firmware supports
  both 384x288 and 640x480 camera models (set `CONFIG_THERMAL_WIDTH/HEIGHT`),
  but selecting the 640x480 frame at runtime against a 384-wide camera crashes,
  so the picker was reduced to one entry rather than left as a trap. Switching
  models is an sdkconfig change plus editing `RESOLUTIONS` here.
- Histogram: log-scaled, selectable axis (Min/Max, Full range, Manual), and
  **Bars or Line**. Gaps are interpolated across empty bins *between* populated
  ones, because real values often land on a coarse lattice (8-bit mode, camera
  gain steps) which otherwise renders as a picket fence. Tails outside the
  occupied range are left empty since those genuinely are.
- `robust_range()` trims `CLIP_PCT` off each tail. The firmware's web AGC
  mirrors this; plain min/max looks broken (see Web Stream above).
- Serial: pyserial leaves `.is_open` True on a handle whose device vanished, so
  the read loop tears the port down on I/O error and a timer resyncs the button.
  `Reload` recycles both video and serial. A deliberate Disconnect stays down.
- Toolbar is two rows. On one row, left-packed widgets claim the width first and
  Tk clips right-packed ones off the end, so the Save buttons vanished entirely
  at the 560 px minimum width.

---

## What Was Removed vs the Original Project
- OV5647 MIPI CSI camera -> DVP thermal capture
- ISP / Bayer demosaic / colour correction -> gone
- RTSP/Ethernet streaming -> gone (replaced by the MJPEG HTTP view)
- 3 UVC formats (UYVY + MJPEG + H264) -> 1 format (Y16), 2 frame sizes
- Extension Unit and PU controls -> gone (`bmControls=0`)
- `esp_video` V4L2 framework -> `esp_cam_ctlr_dvp` directly
- `esp_cam_sensor` -> gone (no I2C sensor)
- `main/uvc_controls.c` and `main/uvc_descriptors.h` -> deleted, do not re-add
