# esp32p4-uvc-framegrabber

Turns a parallel-output thermal camera into a USB webcam.

An ESP32-P4 captures the camera's raw video and streams it to a PC as a standard UVC device. No drivers needed, the PC sees a normal camera. The image is sent as unprocessed 16-bit greyscale, so you get the camera's real sensor values instead of a picture that has already been adjusted.

Also included:

- A live view in a web browser over WiFi.
- A Python viewer for the USB stream with false colour palettes, contrast control, a histogram, and saving to PNG or TIFF.
- Camera control (menu keys, calibration, settings) from the Python viewer.
- PCB design files in `hardware/`.

The WiFi side uses an ESP32-C6 as a companion radio chip, since the P4 has no WiFi of its own.

## Hardware

- ESP32-P4 board with an ESP32-C6 (this project targets the JC-ESP32P4-M3)
- A thermal camera module with a parallel (DVP) video output
- The adapter and backpack boards in `hardware/`, in EasyEDA format

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
