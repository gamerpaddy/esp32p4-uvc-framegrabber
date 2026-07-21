# esp32p4-uvc-framegrabber
**Unfinished** current state works but needs more refinement, pcbs and code can change at any time and may not be compatible with each other. 
wait for a release if everything should play together.


Turns a parallel-output thermal camera into a USB webcam.

An ESP32-P4 captures the camera's raw video and streams it to a PC as a standard UVC device. No drivers needed, the PC sees a normal camera. The image is sent as unprocessed 16-bit greyscale, so you get the camera's real sensor values instead of a picture that has already been adjusted.
this not only includes Thermal cameras but all parallel video devices  that output hsync, vsync, pixelclock and d0 to d16 data.


**IT IS CURRENTLY** adjusted to work with the Dali D8X3C cameras, i plan on making adapter boards for FLIR tau2 soon and adjust the software to work with it.
Both the 384x288 and 640x480 Dali cams work out of the same firmware now — the delivered frame size is switched at runtime with `RES,W,H` (from the web UI buttons, the Python viewer, or the CDC console) and the choice is persisted in NVS so the board comes back on the same resolution after a reboot. The USB host can also just commit whichever advertised UVC frame size it wants and the firmware follows. The old sdkconfig-only limit is gone; any `W x H` up to ~500k pixels is accepted, so bringing up a different parallel sensor later is a runtime setting rather than a rebuild.

<img width="902" height="770" alt="python_Bm7e6duyqK" src="https://github.com/user-attachments/assets/4aec61b1-696f-41f5-a50b-d30f818b4785" />


Also included:

- A live view in a web browser over WiFi. (add a wifi antenna to it or solder a 30mm wire to the LAN pin. 
- A Python viewer for the USB stream with false colour palettes, contrast control, a histogram, and saving to PNG or TIFF.
- Camera control (menu keys, calibration, settings) from the Python viewer. "BIT" switches between 8bit processed / osd and 14bit raw frame.
- PCB design files in `hardware/`. for easyeda and  gerbers

The WiFi side uses an ESP32-C6 as a companion radio chip, since the P4 has no WiFi of its own.

<img width="1681" height="867" alt="python_O5OSTv43hd" src="https://github.com/user-attachments/assets/253fc59e-c83c-42e6-b41a-ce9cbd33bac9" />


## Hardware

- ESP32-P4 board with an ESP32-C6 (this project targets the JC-ESP32P4-M3)
- A thermal camera module with a parallel (DVP) video output
- The adapter and backpack boards in `hardware/`, in EasyEDA format
<img width="426" height="396" alt="dllhost_eG323UHLDy" src="https://github.com/user-attachments/assets/e4402bc7-16b3-49dc-9595-201dec9a8353" />

<img width="1033" height="892" alt="i_view64_hrC4MG2juw" src="https://github.com/user-attachments/assets/e4b13157-1bfd-4292-b980-a05a3c9026eb" />
<img width="848" height="603" alt="i_view64_7ESoIM6SvV" src="https://github.com/user-attachments/assets/33219a2a-dd90-4e3e-bed7-25f00c959eec" />

<img width="952" height="870" alt="i_view64_vHnbMCcFzk" src="https://github.com/user-attachments/assets/88b8e6ac-2efd-4fd4-85ad-bfdf2a412ddb" />

## Building

You need ESP-IDF v6.0.1.

Everything builds and flashes with one script. Replace `COM30` with your board's port:

```
.\tools\flash_all.ps1 -Port COM30
```

This builds the C6 firmware, builds and flashes the P4, and writes the C6 image into the P4's flash.

The C6 is programmed by the P4, not directly. After the script finishes, open the console and run:

```
idf.py -p COM30 monitor

p4> c6boot -d
p4> c6flash
```

Then reset the board. You only need to do the `c6flash` step when the C6 firmware itself has changed.

To rebuild just the P4 app later:

```
idf.py -p COM30 flash monitor
```

## Using it

Plug the board into a PC. It shows up as a camera called "Thermal Bridge".

For the Python viewer:

```
pip install opencv-python numpy pyserial
python cam_viewer.py
```

For the web view, connect the board to WiFi from the console:

```
p4> join <ssid> <password>
p4> status
```

then open the IP it reports in a browser.
