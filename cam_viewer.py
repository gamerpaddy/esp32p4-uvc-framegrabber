# Y16 thermal viewer for the ESP32-P4 thermal UVC bridge.
#
# Renders the 384x288 Y16 stream (low 14 bits = thermal count) with false color,
# an adjustable display window, one-shot auto-scaling, invert, frame capture
# (raw 16-bit TIFF or post-processed PNG), a resizable aspect-correct view, and
# automatic reconnect when the camera is unplugged / replugged.
#
# It also has an ESP command panel: pick the ESP's USB-Serial-JTAG COM port to
# send camera commands (control cross -> KBD,<key>; free-form GCO/SSM,1/...; and
# BIT,8 / BIT,14 to switch the delivered phase) and watch the camera's replies.
#
# Requires: opencv-python, numpy, pillow, pyserial
#   pip install opencv-python numpy pillow pyserial

import os
import re
import time
import threading
from datetime import datetime

import cv2
import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk
except ImportError as e:
    raise SystemExit(
        "This viewer needs tkinter and Pillow.\n"
        "  pip install pillow\n"
        f"(import error: {e})"
    )

try:
    import serial                       # pyserial, for the ESP command channel
    import serial.tools.list_ports as list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False                 # COM panel disabled; pip install pyserial

DEVICE_INDEX = 0
MAXV = 16383  # 14-bit full scale

# Selectable resolutions (must match the frames advertised by the firmware).
RESOLUTIONS = [("384 x 288", 384, 288), ("640 x 480", 640, 480)]
CLIP_PCT = 0.5  # ignore the outer 5% of pixels (each tail) when measuring the range

# ==== SAVED SETTINGS (rewritten in place by the GUI "Save settings" button) ====
SETTINGS = {
    'palette': 'Inferno',
    'normalize': True,
    'hist_scale': 'Full range',
    'invert': False,
    'offset': 0,
    'span': 16383,
    'res': '384 x 288',
    'enhance': 'Off',
    'histogram': False,
    'hist_style': 'Bars',
}
# ==== END SAVED SETTINGS ====

# USB IDs of the bridge (CONFIG_TUSB_VID/PID) — used to auto-pick the CDC
# command port of the composite device; the USB-Serial-JTAG port is 303A:1001.
BRIDGE_USB_ID = "303A:8000"
BRIDGE_PORT_NAME = "Thermal Camera"   # matches the CDC interface string


def robust_range(frame14):
    """Range with the outer CLIP_PCT% of pixels trimmed off each tail, so a few
    dead/hot pixels don't dominate the min/max or the auto-scale window."""
    lo = int(np.percentile(frame14, CLIP_PCT))
    hi = int(np.percentile(frame14, 100 - CLIP_PCT))
    return lo, max(lo + 1, hi)

ENHANCERS = ["Off", "Equalize", "CLAHE"]   # contrast enhancement on the 8-bit image

# Histogram X-axis scaling modes:
#   Min/Max    — axis spans the (outlier-trimmed) min/max of the current frame
#   Full range — axis spans the whole data range (0..16383, or 0..255 in 8-bit mode)
#   Manual     — axis spans the offset/span display window
HIST_SCALE_MODES = ["Min/Max", "Full range", "Manual"]
HIST_STYLES = ["Bars", "Line"]

PALETTES = [
    ("Grayscale", None),
    ("Inferno", cv2.COLORMAP_INFERNO),
    ("Magma", cv2.COLORMAP_MAGMA),
    ("Hot", cv2.COLORMAP_HOT),
    ("Jet", cv2.COLORMAP_JET),
    ("Viridis", cv2.COLORMAP_VIRIDIS),
    ("Turbo", cv2.COLORMAP_TURBO),
]

# Camera command reference: (CMD, value-hint, description). Empty hint = no arg.
COMMANDS = [
    ("KBD", "F S M C + -", "Keyboard button"),
    ("SSM", "0-1", "Image filtering off/on"),
    ("GSM", "", "Image filter enabled?"),
    ("SEH", "0-1", "Image enhancement off/on"),
    ("GEH", "", "Image enhancement enabled?"),
    ("SCO", "0-255", "Set gain"),
    ("GCO", "", "Get gain"),
    ("INT", "10-320", "Set integration time"),
    ("GIT", "", "Get current integration time"),
    ("IBR", "0-255", "Set brightness"),
    ("GBR", "", "Get brightness"),
    ("SAG", "0-2", "Set auto gain mode"),
    ("GAG", "", "Get auto gain mode"),
    ("SMR", "0-3", "Mirror: bit0=horizontal, bit1=vertical"),
    ("GMR", "", "Get rotation/flip"),
    ("SNU", "0-1", "Shutter close/open"),
    ("GNU", "", "Get shutter closed/open"),
    ("STD", "0-1", "Set range 0:-20..180C  1:100..600C"),
    ("GTD", "", "Get range"),
    ("SET", "degC", "Set ambient temperature"),
    ("GET", "", "Get ambient temperature"),
    ("SEM", "0-100", "Set emissivity x100"),
    ("GEM", "", "Get emissivity"),
    ("SRT", "degC", "Set revise temp (-10..10C)"),
    ("GRT", "", "Get revise temp"),
    ("SRD", "x100", "Set distance (4.4 m -> 440)"),
    ("GRD", "", "Get distance"),
    ("SHD", "0-100", "Set humidity"),
    ("GHD", "", "Get humidity"),
    ("SWP", "0-1", "Invert polarity of image"),
    ("GWP", "", "Get polarity inverted"),
    ("SPA", "0-10", "Set color palette (3 = white hot)"),
    ("GPA", "", "Get palette"),
    ("SMV", "0-1", "Show temperature 3x3 grid? (with GTV?)"),
    ("GMV", "", "Get SMV"),
    ("SMS", "0-1", "Show 'collected 0/SCT' at SCP delay (daily max temps?)"),
    ("GMS", "", "Get SMS"),
    ("SCT", "1-500", "Amount of collected points (SMS)"),
    ("GCT", "", "Get SCT (e.g. 80)"),
    ("SCP", "5-60", "Collected delay seconds (SMS)"),
    ("GCP", "", "Get SCP"),
    ("SLR", "0-9", "??"),
    ("GLR", "", "Get SLR"),
    ("SRP", "x,y", "Set crosshair position (0,0-639,479)"),
    ("GRP", "", "Get crosshair position"),
    ("SCX", "+-1", "Set x axis range"),
    ("SCY", "+-1", "Set y axis range"),
    ("DRC", "0-1", "Show crosshair"),
    ("SLG", "-500..500", "??"),
    ("GLG", "", "Get SLG"),
    ("SCE", "text", "??"),
    ("GCE", "", "Get SCE"),
    ("SCB", "text", "??"),
    ("GCB", "", "Get SCB"),
    ("SVB", "0-16383", "Adjust brightness value (abs or +1/-1)"),
    ("GVB", "", "Get brightness value (16383 - SVB)"),
    ("SVC", "1-1023", "Adjust contrast value (abs or +1/-1)"),
    ("GVC", "", "Get contrast value"),
    ("SVF", "100+", "Set VF voltage (~3400 good)"),
    ("SVS", "100+", "Set VS voltage (~3300 good)"),
    ("SOT", "0-1", "Output image: 0 original, 1 calibrated"),
    ("MSV", "", "Menu save value"),
    ("SZM", "1-4", "Set zoom value"),
    ("GZM", "", "Get zoom value"),
    ("ASN", "val", "Set partition number"),
    ("ASV", "val", "Save area data of area <val>?"),
    ("ALD", "0/2?", "Load area data? (blanks, EEPROM activity, reload pixel map?)"),
    ("CBP", "B,X,Y", "Dead pixel cal (B,100,100); S,<val> saves to area"),
    ("C2P", "L/H/B/S", "Two point cal: L low, H high, B two point, S,<val> save"),
    ("GCS", "", "Get core status & faults (0=OK 1=SDRAM 2=Flash 3=video 4=SW 5=FPGA 6=SW abn 7=detector 8=power 9=cooler)"),
    ("SAV", "0-9999", "Set fire threshold"),
    ("GAV", "", "Get fire threshold (e.g. 2100)"),
    ("BAC", "?", "Change UART protocol? CAREFUL"),
    ("CCM", "", "Communication test"),
    ("GFV", "", "Get clarity value"),
    ("QYA", "", "Get average grey level"),
    ("QYI", "", "Get min grey level"),
    ("QYM", "", "Get max grey level"),
    ("MED", "", "Turn on indication info"),
    ("MEH", "", "Turn off indication info"),
    ("GME", "", "Get active menu 0-7"),
    ("PWI", "0-1", "Palette white-hot <-> fire"),
    ("CBD", "", "Recovery successful?"),
    ("CBS", "", "Saved successfully"),
    ("LRD", "", "Recovery successful?"),
    ("LRS", "", "Saved successfully?"),
    ("LRE", "", "Temperature repair successful?"),
    ("CBT", "", "Calibration successful?"),
    ("DBP", "", "??? shutter clicks"),
    ("DPI", "", "Show weird pixels"),
    ("API", "", "Hide weird pixels"),
    ("GAB", "", "?? 4 vals (3,-10,50,150)"),
    ("GAN", "", "?? (1)"),
    ("GAR", "", "?? 11 vals (10,10,5,20,35,...)"),
    ("GCM", "", "?? (6 values)"),
    ("GTV", "", "Get collected temp values?"),
    ("PTS", "", "Echoes with extra stuff?"),
    ("VTP", "400-4000", "Set iVtemp?"),
    ("MTD", "U/S,0,0", "?? S answers bytes, U uploads bytes"),
    ("MTC", "U/S", "?? U -> MTC,U,0,0,I; S -> MTC,U,0,11,A000..."),
    ("SDT", "0-1", "Show temp=44.65C in OSD"),
    ("GPT", "", "Display temp of SDT"),
    ("GDT", "0-1", "Get temp display active?"),
]

# Dark, low-clutter theme.
BG = "#1e1f24"
PANEL = "#2a2c33"
FG = "#e6e6e6"
ACCENT = "#4da3ff"


def open_camera(index, w, h):
    cap = cv2.VideoCapture(index + cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y', '1', '6', ' '))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)     # selects the matching UVC frame
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    return cap


class CameraThread(threading.Thread):
    """Reads Y16 frames in the background and auto-reconnects on unplug."""

    def __init__(self, index, w, h):
        super().__init__(daemon=True)
        self.index = index
        self.w = w
        self.h = h
        self._lock = threading.Lock()
        self._frame = None
        self._running = True
        self.connected = False
        self.fps = 0.0
        self.mode = "?"   # "14-bit thermal" or "8-bit video", from frame content

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stop(self):
        self._running = False

    # A freshly (re)opened handle needs time for USB re-enumeration + UVC
    # stream negotiation + the ESP's first capture, so allow a long window for
    # the FIRST frame. Once frames are flowing, a short gap means it's gone.
    FIRST_FRAME_TIMEOUT = 8.0
    STALE_TIMEOUT = 2.5

    def run(self):
        cap = None
        session_start = 0.0
        got_frame = False
        last_ok = time.time()
        t0, n = time.time(), 0
        while self._running:
            if cap is None:
                self.connected = False
                self.fps = 0.0
                cap = open_camera(self.index, self.w, self.h)
                if cap is None:
                    time.sleep(0.5)
                    continue
                session_start = time.time()
                got_frame = False

            ret, raw = cap.read()
            buf = None if raw is None else np.frombuffer(raw, dtype='<u2')
            now = time.time()
            npix = self.w * self.h

            if ret and buf is not None and buf.size >= npix:
                got_frame = True
                last_ok = now
                self.connected = True
                frame = (buf[:npix].reshape(self.h, self.w) & 0x3FFF).copy()
                with self._lock:
                    self._frame = frame
                # The camera interleaves two phases; the ESP delivers one. Tell
                # which from the data itself: 8-bit video never exceeds 255.
                self.mode = "14-bit thermal" if int(frame.max()) > 255 else "8-bit video"
                n += 1
                if now - t0 >= 0.5:
                    self.fps = n / (now - t0)
                    t0, n = now, 0
            else:
                time.sleep(0.03)

            # Watchdog. Key off frame freshness, not cap.isOpened() (which can
            # stay True on a dead device after an ESP reboot). A newly opened
            # handle gets the long FIRST_FRAME_TIMEOUT to deliver anything; a
            # streaming handle gets the short STALE_TIMEOUT. Either expiry drops
            # the handle and reopens — recovering without restarting the viewer.
            limit = self.STALE_TIMEOUT if got_frame else self.FIRST_FRAME_TIMEOUT
            ref = last_ok if got_frame else session_start
            if now - ref > limit:
                self.connected = False
                self.fps = 0.0
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                cap = None
                time.sleep(0.6)             # let Windows finish re-enumerating

        if cap is not None:
            cap.release()


class SerialConsole:
    """Talks to the ESP over its USB-Serial-JTAG COM port: sends typed commands,
    and collects only the lines tagged 'cam_uart' (the camera command channel)."""

    def __init__(self):
        self.ser = None
        self._running = False
        self._lock = threading.Lock()
        self._lines = []

    @staticmethod
    def ports():
        if not HAVE_SERIAL:
            return []
        return [p.device for p in list_ports.comports()]

    @staticmethod
    def find_bridge():
        """Best-guess COM port of the bridge's CDC interface: prefer a port
        whose strings name the thermal camera, else match the USB VID:PID
        (excluding the 303A:1001 USB-Serial-JTAG port)."""
        if not HAVE_SERIAL:
            return None
        best, best_score = None, 0
        for p in list_ports.comports():
            text = " ".join(str(s) for s in
                            (p.description, p.product, p.interface, p.manufacturer) if s)
            score = 0
            if BRIDGE_PORT_NAME.lower() in text.lower():
                score = 2
            elif BRIDGE_USB_ID in (p.hwid or "").upper():
                score = 1
            if score > best_score:
                best, best_score = p.device, score
        return best

    @staticmethod
    def decode_reply(line):
        """Extract 'CMD,VAL1[,VAL2...]' from a raw reply frame rendered as
        ASCII (STX/len/checksum/ETX bytes show as '.' or arbitrary chars).
        The payload is 'CMD,' or 'CMD,values,' — i.e. it always ends with a
        comma, followed by the checksum byte and ETX. Strip that tail."""
        m = re.search(r"[A-Z]{3}", line)
        if not m:
            return line.strip()
        s = line[m.start():].strip()
        # Framed replies end "...,<checksum><ETX>" — i.e. a short 1-3 char tail
        # after the payload's final comma. Strip only that; leave longer tails
        # (e.g. the local "BIT,14 (thermal phase)" confirmation) intact.
        last = s.rfind(",")
        if last >= 3:
            tail = s[last + 1:]
            # Framing bytes come through as '.' (RX task replaces non-printables)
            # or as literal control/replacement chars. A short tail without any
            # of those markers is a REAL value (RES,640,480 → tail "480") and
            # must not be stripped — the previous len(tail)<=3 heuristic ate
            # 3-digit resolutions and broke the web-issued RES round-trip.
            if len(tail) <= 3 and ("." in tail or re.search(r"[\x00-\x1f\x7f�]", tail)):
                s = s[:last]
        return s.rstrip(",")

    @staticmethod
    def decode_replies(line):
        """A single serial line can carry several back-to-back frames (e.g.
        'KBD,1,Z<chk><ETX><STX>..BPM,0<chk><ETX>' when the shutter closes).
        Split at each 'XXX,' command start and decode each frame on its own."""
        starts = [m.start() for m in re.finditer(r"[A-Z]{3},", line)]
        if not starts:
            return [SerialConsole.decode_reply(line)]
        out = []
        for i, st in enumerate(starts):
            end = starts[i + 1] if i + 1 < len(starts) else len(line)
            s = SerialConsole.decode_reply(line[st:end])
            # scrub any leftover framing bytes rendered as junk
            out.append(re.sub(r"[\x00-\x1f\x7f�.]+$", "", s))
        return out

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port):
        self.disconnect()
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def disconnect(self):
        self._running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    @staticmethod
    def normalize(text):
        """Split input into CMD and values, dropping empty comma segments so
        ',,' or a trailing ',' is never sent (the bridge appends its own
        checksum after the last comma). Values may be numbers or text and
        multiple values stay comma-separated: 'SEH,1,' -> 'SEH,1'."""
        parts = [p.strip() for p in text.strip().upper().split(",")]
        parts = [p for p in parts if p]
        return ",".join(parts)

    def send(self, text):
        text = self.normalize(text)
        if text and self.is_open():
            try:
                self.ser.write((text + "\r\n").encode("ascii", "ignore"))
                return True
            except Exception:
                return False
        return False

    def _read_loop(self):
        buf = b""
        while self._running and self.ser is not None:
            try:
                data = self.ser.read(256)
            except Exception:
                # The device went away (ESP reboot, cable pull). pyserial keeps
                # .is_open True on a handle whose device has vanished, so unless
                # we tear it down here the GUI keeps claiming "Disconnect" while
                # every send() silently fails. Drop it so is_open() tells the
                # truth and the reconnect watchdog can act.
                self._running = False
                try:
                    if self.ser is not None:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                s = raw.decode("utf-8", "replace").rstrip()
                if "cam_uart" in s:                 # only the camera channel
                    # trim the "I (12345) cam_uart: " log prefix for readability
                    s = s.split("cam_uart:", 1)[-1].strip()
                    with self._lock:
                        self._lines.append(s)

    def drain(self):
        with self._lock:
            out, self._lines = self._lines, []
        return out


class Viewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Thermal Y16 Viewer")
        self.configure(bg=BG)
        self.geometry("900x720")
        self.minsize(560, 460)

        self.normalize = tk.BooleanVar(value=SETTINGS.get("normalize", True))
        hist_scale = SETTINGS.get("hist_scale", "Full range")
        if hist_scale not in HIST_SCALE_MODES:
            hist_scale = "Full range"
        self.hist_scale = tk.StringVar(value=hist_scale)
        self.invert = tk.BooleanVar(value=SETTINGS.get("invert", False))
        self.palette = tk.StringVar(value=SETTINGS.get("palette", PALETTES[1][0]))
        self.offset = tk.DoubleVar(value=SETTINGS.get("offset", 0))   # ttk.Scale is float-valued
        self.span = tk.DoubleVar(value=SETTINGS.get("span", MAXV))
        self.enhance = tk.StringVar(value=SETTINGS.get("enhance", "Off"))
        self.show_hist = tk.BooleanVar(value=SETTINGS.get("histogram", False))
        hist_style = SETTINGS.get("hist_style", "Bars")
        if hist_style not in HIST_STYLES:
            hist_style = "Bars"
        self.hist_style = tk.StringVar(value=hist_style)
        self._want_serial = False      # True once a connection is wanted, so a
                                       # dropped port auto-reconnects but a
                                       # deliberate Disconnect stays down
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.last_range = (0, 0)
        self._photo = None
        self._proc = None  # latest full-res processed BGR image (for PNG save)
        self.com = tk.StringVar()
        self.serial = SerialConsole()
        self._cmd_win = None

        # Resolution (must match a firmware-advertised frame). Clamp to a name
        # that is actually in RESOLUTIONS: a settings block saved back when the
        # list still had 640x480 would otherwise leave the combobox displaying a
        # size the firmware does not advertise, while _res_dims silently fell
        # back to 384x288 - the label and the decode disagreeing.
        _saved_res = SETTINGS.get("res", RESOLUTIONS[0][0])
        if _saved_res not in [r[0] for r in RESOLUTIONS]:
            _saved_res = RESOLUTIONS[0][0]
        self.res = tk.StringVar(value=_saved_res)
        rw, rh = self._res_dims(self.res.get())

        self._build_style()
        self._build_ui()

        self.cam = CameraThread(DEVICE_INDEX, rw, rh)
        self.cam.start()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(600, self._auto_connect)   # auto-pick the bridge's CDC port
        self.after(33, self._tick)

    # ---- UI ----------------------------------------------------------------
    def _build_style(self):
        s = ttk.Style(self)
        try:
            s.theme_use("clam")
        except tk.TclError:
            pass
        s.configure(".", background=BG, foreground=FG, fieldbackground=PANEL)
        s.configure("TFrame", background=BG)
        s.configure("Panel.TFrame", background=PANEL)
        s.configure("TLabel", background=BG, foreground=FG)
        s.configure("Status.TLabel", background=PANEL, foreground=FG, padding=6)
        s.configure("TButton", background=PANEL, foreground=FG, padding=6, borderwidth=0)
        s.map("TButton", background=[("active", ACCENT)], foreground=[("active", "#101010")])
        s.configure("TCheckbutton", background=BG, foreground=FG)
        s.map("TCheckbutton", background=[("active", BG)])
        s.configure("TCombobox", fieldbackground=PANEL, background=PANEL,
                    foreground=FG, arrowcolor=FG)
        # In readonly state a Combobox draws its text with the SELECTION colors,
        # so without this the value shows white-on-gray and is barely legible.
        s.map("TCombobox",
              fieldbackground=[("readonly", PANEL)],
              foreground=[("readonly", FG)],
              selectbackground=[("readonly", PANEL)],
              selectforeground=[("readonly", FG)],
              background=[("readonly", PANEL)])
        # The drop-down list is a plain Tk Listbox (not ttk) — colour it via the
        # option database so it matches the dark theme.
        self.option_add("*TCombobox*Listbox.background", PANEL)
        self.option_add("*TCombobox*Listbox.foreground", FG)
        self.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        self.option_add("*TCombobox*Listbox.selectForeground", "#101010")
        s.configure("Accent.Horizontal.TScale", background=BG)

    def _build_ui(self):
        # Two toolbar rows rather than one. With everything on a single row, the
        # left-packed controls claim the width first and Tk simply clips whatever
        # is packed to the right off the end - so at the 560 px minsize the three
        # Save buttons vanished entirely with no scroll or overflow to reach them.
        # Splitting view controls (row 1) from actions (row 2) keeps both rows
        # inside the minimum width.
        bar = ttk.Frame(self, padding=(10, 8, 10, 2))
        bar.pack(side="top", fill="x")
        actions = ttk.Frame(self, padding=(10, 0, 10, 6))
        actions.pack(side="top", fill="x")

        ttk.Label(bar, text="Res").pack(side="left", padx=(0, 6))
        res_combo = ttk.Combobox(bar, textvariable=self.res, state="readonly",
                                 width=10, values=[r[0] for r in RESOLUTIONS])
        res_combo.pack(side="left", padx=(0, 16))
        res_combo.bind("<<ComboboxSelected>>", lambda e: self._set_resolution())

        ttk.Label(bar, text="Palette").pack(side="left", padx=(0, 6))
        combo = ttk.Combobox(bar, textvariable=self.palette, state="readonly",
                             width=12, values=[p[0] for p in PALETTES])
        combo.pack(side="left", padx=(0, 16))

        ttk.Label(bar, text="Enhance").pack(side="left", padx=(0, 6))
        enh = ttk.Combobox(bar, textvariable=self.enhance, state="readonly",
                           width=9, values=ENHANCERS)
        enh.pack(side="left", padx=(0, 16))

        ttk.Checkbutton(bar, text="Normalize", variable=self.normalize,
                        command=self._sync_manual).pack(side="left", padx=4)
        ttk.Checkbutton(bar, text="Invert", variable=self.invert).pack(side="left", padx=4)
        ttk.Checkbutton(bar, text="Histogram", variable=self.show_hist).pack(side="left", padx=4)
        hist_combo = ttk.Combobox(bar, textvariable=self.hist_scale, state="readonly",
                                  width=10, values=HIST_SCALE_MODES)
        hist_combo.pack(side="left", padx=(0, 4))
        hist_combo.bind("<<ComboboxSelected>>", lambda e: self._sync_manual())
        ttk.Combobox(bar, textvariable=self.hist_style, state="readonly",
                     width=6, values=HIST_STYLES).pack(side="left", padx=(0, 4))
        ttk.Button(actions, text="Auto-scale", command=self._autoscale).pack(side="left", padx=(0, 4))
        ttk.Button(actions, text="Reload", command=self._reload).pack(side="left", padx=4)

        ttk.Button(actions, text="Save PNG", command=self._save_png).pack(side="right", padx=(4, 0))
        ttk.Button(actions, text="Save TIFF (raw)", command=self._save_tiff).pack(side="right", padx=4)
        ttk.Button(actions, text="Save settings", command=self._save_settings).pack(side="right", padx=4)

        # Manual window controls (only meaningful when Normalize is off).
        self.manual = ttk.Frame(self, padding=(10, 0, 10, 6))
        self.manual.pack(side="top", fill="x")
        ttk.Label(self.manual, text="Offset").pack(side="left")
        self._off_scale = ttk.Scale(self.manual, from_=0, to=MAXV, variable=self.offset,
                                    orient="horizontal")
        self._off_scale.pack(side="left", fill="x", expand=True, padx=(6, 16))
        ttk.Label(self.manual, text="Span").pack(side="left")
        self._span_scale = ttk.Scale(self.manual, from_=1, to=MAXV, variable=self.span,
                                     orient="horizontal")
        self._span_scale.pack(side="left", fill="x", expand=True, padx=6)

        self.status = ttk.Label(self, style="Status.TLabel", anchor="w",
                                text="connecting…")
        self.status.pack(side="bottom", fill="x")

        # Right-hand ESP command panel (COM channel + control cross + returns).
        self._build_command_panel().pack(side="right", fill="y")

        self.canvas = tk.Canvas(self, bg="#000000", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)

        self._sync_manual()

    def _build_command_panel(self):
        panel = ttk.Frame(self, style="Panel.TFrame", padding=10, width=250)
        panel.pack_propagate(False)

        ttk.Label(panel, text="ESP / Camera commands", background=PANEL).pack(anchor="w")

        if not HAVE_SERIAL:
            ttk.Label(panel, text="pip install pyserial\nto enable this panel",
                      background=PANEL).pack(anchor="w", pady=8)
            return panel

        # COM port selector + connect.
        com_row = ttk.Frame(panel, style="Panel.TFrame")
        com_row.pack(fill="x", pady=(8, 4))
        self._com_combo = ttk.Combobox(com_row, textvariable=self.com, state="readonly", width=10)
        self._com_combo.pack(side="left")
        ttk.Button(com_row, text="⟳", width=3, command=self._refresh_ports).pack(side="left", padx=3)
        self._connect_btn = ttk.Button(com_row, text="Connect", command=self._toggle_connect)
        self._connect_btn.pack(side="left")
        self._refresh_ports()

        # Control cross (D-pad): + up, - down, F back, M menu/forward, C FFC.
        cross = ttk.Frame(panel, style="Panel.TFrame")
        cross.pack(pady=10)
        def dbtn(parent, text, key, r, c):
            ttk.Button(parent, text=text, width=5,
                       command=lambda: self._send_key(key)).grid(row=r, column=c, padx=2, pady=2)
        dbtn(cross, "▲ +", "+", 0, 1)
        dbtn(cross, "◀ F", "F", 1, 0)
        dbtn(cross, "C", "C", 1, 1)        # FFC click
        dbtn(cross, "M ▶", "M", 1, 2)      # menu / forward
        dbtn(cross, "▼ -", "-", 2, 1)
        ttk.Button(panel, text="S — system menu",
                   command=lambda: self._send_key("S")).pack(fill="x", pady=(0, 2))
        ttk.Label(panel,
                  text="Passwords (enter on the cross):\n"
                       "  System menu:  + - M C + -\n"
                       "  Other menus:  + - + - + -",
                  background=PANEL, foreground="#8a8d96",
                  justify="left").pack(anchor="w", pady=(0, 8))

        # 8/14-bit phase switch (menu only shows in 8-bit).
        bit_row = ttk.Frame(panel, style="Panel.TFrame")
        bit_row.pack(fill="x")
        ttk.Button(bit_row, text="8-bit (menu)",
                   command=lambda: self._send_cmd("BIT,8")).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(bit_row, text="14-bit",
                   command=lambda: self._send_cmd("BIT,14")).pack(side="left", expand=True, fill="x", padx=(2, 0))

        # Free-form command entry.
        entry_row = ttk.Frame(panel, style="Panel.TFrame")
        entry_row.pack(fill="x", pady=(10, 4))
        self._cmd_entry = ttk.Entry(entry_row)
        self._cmd_entry.pack(side="left", expand=True, fill="x")
        self._cmd_entry.bind("<Return>", lambda e: self._send_entry())
        ttk.Button(entry_row, text="Send", command=self._send_entry).pack(side="left", padx=(4, 0))
        ttk.Label(panel, text="e.g.  GCO   SSM,1   KBD,C", background=PANEL,
                  foreground="#8a8d96").pack(anchor="w")
        ttk.Button(panel, text="All commands…", command=self._open_commands).pack(fill="x", pady=(6, 0))

        # Returns from the camera (cam_uart lines only).
        ttk.Label(panel, text="Camera returns", background=PANEL).pack(anchor="w", pady=(10, 2))
        self._returns = tk.Text(panel, height=10, width=30, bg="#15161a", fg=FG,
                                insertbackground=FG, relief="flat", wrap="word")
        self._returns.pack(fill="both", expand=True)
        self._returns.tag_configure("err", foreground="#e05555")
        self._returns.tag_configure("warn", foreground="#e0c040")
        self._returns.tag_configure("ok", foreground="#60d060")
        self._returns.configure(state="disabled")
        return panel

    def _sync_manual(self):
        # Sliders drive the display window (Normalize off) and/or the histogram
        # axis (hist scale = Manual) — enable them if either consumer is active.
        used = (not self.normalize.get()) or self.hist_scale.get() == "Manual"
        state = "normal" if used else "disabled"
        self._off_scale.configure(state=state)
        self._span_scale.configure(state=state)

    # ---- ESP command channel ----------------------------------------------
    def _refresh_ports(self):
        ports = SerialConsole.ports()
        self._com_combo.configure(values=ports)
        bridge = SerialConsole.find_bridge()
        if bridge:
            self.com.set(bridge)
        elif ports and not self.com.get():
            self.com.set(ports[0])

    def _auto_connect(self):
        """Connect to the bridge's CDC command port automatically at startup."""
        if not HAVE_SERIAL or self.serial.is_open():
            return
        bridge = SerialConsole.find_bridge()
        if not bridge:
            self.after(2000, self._auto_connect)   # keep looking (device may enumerate late)
            return
        self.com.set(bridge)
        try:
            self.serial.connect(bridge)
            self._want_serial = True
            self._connect_btn.configure(text="Disconnect")
            self._append_return(f"— auto-connected {bridge} —")
            # Ask the P4 what resolution it's actually on so we adopt that
            # instead of opening UVC at our own combo default and clobbering
            # the persisted resolution back to it.
            self.after(300, lambda: self.serial.send("RES"))
        except Exception:
            self.after(2000, self._auto_connect)

    def _toggle_connect(self):
        if self.serial.is_open():
            self._want_serial = False       # deliberate: don't auto-reconnect
            self.serial.disconnect()
            self._connect_btn.configure(text="Connect")
            return
        port = self.com.get()
        if not port:
            self._flash("no COM port selected")
            return
        try:
            self.serial.connect(port)
            self._want_serial = True
            self._connect_btn.configure(text="Disconnect")
            self._append_return(f"— connected {port} —")
            self.after(300, lambda: self.serial.send("RES"))
        except Exception as e:
            self._flash(f"open failed: {e}")

    def _send_cmd(self, text):
        if self.serial.send(text):
            self._append_return(f">> {SerialConsole.normalize(text)}")
        else:
            self._flash("not connected")

    def _send_key(self, key):
        """Send a control-cross / menu key. The camera's on-screen menu is only
        drawn into the 8-bit video phase — so when a menu key (S or M) is
        pressed while the 14-bit thermal phase is being delivered, switch to
        8-bit first so the menu is actually visible."""
        if key in ("S", "M") and self.cam.mode == "14-bit thermal":
            self._send_cmd("BIT,8")
        self._send_cmd(f"KBD,{key}")

    def _send_entry(self):
        text = self._cmd_entry.get().strip()
        if text:
            self._send_cmd(text)
            self._cmd_entry.delete(0, "end")

    def _open_commands(self):
        """Pop out a scrollable, filterable list of all camera commands, each
        with its own value field and Send button."""
        if getattr(self, "_cmd_win", None) is not None and self._cmd_win.winfo_exists():
            self._cmd_win.lift()
            return
        win = tk.Toplevel(self)
        self._cmd_win = win
        win.title("Camera commands")
        win.configure(bg=BG)
        win.geometry("600x640")

        top = ttk.Frame(win, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Filter").pack(side="left")
        fvar = tk.StringVar()
        ttk.Entry(top, textvariable=fvar).pack(side="left", fill="x", expand=True, padx=6)

        outer = ttk.Frame(win)
        outer.pack(fill="both", expand=True)
        canvas = tk.Canvas(outer, bg=BG, highlightthickness=0)
        sb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        def on_wheel(e):
            canvas.yview_scroll(int(-e.delta / 120), "units")
        canvas.bind_all("<MouseWheel>", on_wheel)

        def rebuild(*_):
            for w in inner.winfo_children():
                w.destroy()
            flt = fvar.get().lower()
            r = 0
            for cmd, hint, desc in COMMANDS:
                if flt and flt not in cmd.lower() and flt not in desc.lower():
                    continue
                ttk.Label(inner, text=cmd, width=5).grid(row=r, column=0, sticky="w", padx=(8, 4), pady=1)
                ent = ttk.Entry(inner, width=10)
                ent.grid(row=r, column=1, padx=4, pady=1)
                ttk.Button(inner, text="Send", width=5,
                           command=lambda c=cmd, e=ent: self._send_table(c, e)
                           ).grid(row=r, column=2, padx=4)
                txt = (f"[{hint}]  " if hint else "") + desc
                ttk.Label(inner, text=txt).grid(row=r, column=3, sticky="w", padx=4)
                r += 1

        fvar.trace_add("write", rebuild)
        rebuild()

        def on_close():
            canvas.unbind_all("<MouseWheel>")
            win.destroy()
            self._cmd_win = None
        win.protocol("WM_DELETE_WINDOW", on_close)

    def _send_table(self, cmd, entry):
        val = entry.get().strip()
        self._send_cmd(f"{cmd},{val}" if val else cmd)

    def _show_reply(self, s):
        # "RES,W,H" is emitted by the P4 when a resolution switch is requested
        # (from the web UI, or the P4's own console). Match the UVC size to it
        # and restart the capture so the host committed frame stays in sync
        # with what the firmware is delivering.
        m_res = re.match(r"^RES,(\d+),(\d+)$", s)
        if m_res:
            w, h = int(m_res.group(1)), int(m_res.group(2))
            name = next((n for n, rw, rh in RESOLUTIONS if rw == w and rh == h), None)
            if name:
                self.res.set(name)
                self._restart_cam(w, h)
                self._flash(f"resolution -> {w}x{h}")
                self._append_return(f"RES: switched to {w}x{h}", "ok")
            else:
                self._append_return(f"RES: {w}x{h} not in RESOLUTIONS list", "warn")
            return
        m = re.match(r"^([A-Z]{3}),(\d)(?:,(.*))?$", s)
        if not m:
            self._append_return(s)
            return
        cmd, status, rest = m.group(1), m.group(2), m.group(3)
        # Unsolicited notifications (auto-NUC etc.) reply "CMD,0" / "CMD,1"
        # where the digit is the VALUE, not a status flag — show as-is.
        if cmd in ("BPM", "AVS") and rest is None:
            self._append_return(f"{cmd}:  {status}")
        elif status == "2":
            self._append_return(f"{cmd}: unknown command", "err")
        elif status == "0":
            self._append_return(f"{cmd}: wrong param, type or range", "warn")
        elif status == "1":
            self._append_return(f"{cmd}:  {rest}" if rest else f"{cmd}: OK", "ok")
        else:
            self._append_return(s)

    def _append_return(self, line, tag=None):
        self._returns.configure(state="normal")
        self._returns.insert("end", line + "\n", tag or ())
        # keep the box from growing unbounded
        if int(self._returns.index("end-1c").split(".")[0]) > 300:
            self._returns.delete("1.0", "100.0")
        self._returns.see("end")
        self._returns.configure(state="disabled")

    # ---- actions -----------------------------------------------------------
    @staticmethod
    def _res_dims(name):
        for n, w, h in RESOLUTIONS:
            if n == name:
                return w, h
        return RESOLUTIONS[0][1], RESOLUTIONS[0][2]

    def _restart_cam(self, w, h):
        try:
            self.cam.stop()
        except Exception:
            pass
        self.cam = CameraThread(DEVICE_INDEX, w, h)
        self.cam.start()

    def _set_resolution(self):
        w, h = self._res_dims(self.res.get())
        self._restart_cam(w, h)
        self._flash(f"resolution -> {w}x{h}")

    def _reload(self):
        """Fully tear down and recreate the capture. Recovers a hung/garbled
        stream after an ESP reboot, when the handle keeps delivering stale or
        corrupt frames and the auto-watchdog can't tell it's bad.

        Also recycles the serial console: the reboot that broke the video
        handle broke the COM port too, and the old code left that half dead
        while the button still read "Disconnect"."""
        self._restart_cam(self.cam.w, self.cam.h)
        if HAVE_SERIAL:
            self.serial.disconnect()
            self._sync_connect_btn()
            self.after(500, self._auto_connect)   # let the port re-enumerate first
        self._flash("reloading camera…")

    def _sync_connect_btn(self):
        """Make the button label reflect the real port state. Called on a timer
        because the port can die asynchronously (ESP reboot) with no UI event."""
        if not HAVE_SERIAL:
            return
        want = "Disconnect" if self.serial.is_open() else "Connect"
        if self._connect_btn.cget("text") != want:
            self._connect_btn.configure(text=want)
            if want == "Connect":
                self._append_return("— port lost —")
                # Auto-reconnect only if we did not disconnect on purpose.
                if self._want_serial:
                    self.after(1000, self._auto_connect)

    def _save_settings(self):
        """Persist the current settings by rewriting the SETTINGS block in this
        very .py file (no external config file)."""
        data = {
            "palette": self.palette.get(),
            "normalize": bool(self.normalize.get()),
            "hist_scale": self.hist_scale.get(),
            "invert": bool(self.invert.get()),
            "offset": int(self.offset.get()),
            "span": int(self.span.get()),
            "res": self.res.get(),
            "enhance": self.enhance.get(),
            "histogram": bool(self.show_hist.get()),
            "hist_style": self.hist_style.get(),
        }
        block = ("SETTINGS = {\n"
                 + "".join(f"    {k!r}: {data[k]!r},\n" for k in
                           ("palette", "normalize", "hist_scale", "invert", "offset", "span", "res",
                            "enhance", "histogram", "hist_style"))
                 + "}")
        try:
            path = os.path.abspath(__file__)
            with open(path, "r", encoding="utf-8") as fh:
                src = fh.read()
            new, count = re.subn(r"SETTINGS = \{.*?\n\}", block, src, count=1, flags=re.DOTALL)
            if count != 1:
                raise RuntimeError("SETTINGS block not found")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(new)
            self._flash("settings saved to file")
        except Exception as e:
            self._flash(f"save failed: {e}")

    def _autoscale(self):
        f = self.cam.latest()
        if f is None:
            return
        lo, hi = robust_range(f)
        self.normalize.set(False)
        self.offset.set(lo)
        self.span.set(max(1, hi - lo))
        self._sync_manual()

    def _save_tiff(self):
        f = self.cam.latest()
        if f is None:
            self._flash("nothing to save")
            return
        name = datetime.now().strftime("thermal_%Y%m%d_%H%M%S_raw.tiff")
        cv2.imwrite(name, f.astype(np.uint16))   # raw 14-bit counts, lossless
        self._flash(f"saved {name}")

    def _save_png(self):
        if self._proc is None:
            self._flash("nothing to save")
            return
        name = datetime.now().strftime("thermal_%Y%m%d_%H%M%S.png")
        cv2.imwrite(name, self._proc)            # post-processed false-color
        self._flash(f"saved {name}")

    def _flash(self, msg):
        self._flash_msg = (msg, time.time())

    # ---- render ------------------------------------------------------------
    def _process(self, frame14):
        if self.normalize.get():
            lo, hi = robust_range(frame14)
        else:
            lo = int(self.offset.get())
            hi = lo + max(1, int(self.span.get()))
        if hi <= lo:
            hi = lo + 1

        disp = np.clip((frame14.astype(np.int32) - lo) * 255 // (hi - lo), 0, 255).astype(np.uint8)

        # Contrast enhancement on the windowed 8-bit image, before colormap.
        enh = self.enhance.get()
        if enh == "Equalize":
            disp = cv2.equalizeHist(disp)
        elif enh == "CLAHE":
            disp = self._clahe.apply(disp)

        if self.invert.get():
            disp = 255 - disp

        cmap = dict(PALETTES)[self.palette.get()]
        if cmap is None:
            bgr = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
        else:
            bgr = cv2.applyColorMap(disp, cmap)

        if self.show_hist.get():
            self._overlay_histogram(bgr, frame14, lo, hi)
        return bgr

    def _overlay_histogram(self, bgr, frame14, lo, hi):
        """Draw a log-scaled histogram strip along the bottom of the image.
        The X axis span follows the histogram scale mode (Min/Max of the frame,
        the full data range, or the manual offset/span window); the current
        display window [lo, hi] is marked in accent color."""
        h, w = bgr.shape[:2]
        hh = max(40, h // 6)                    # strip height
        maxv = 255 if self.cam.mode == "8-bit video" else MAXV
        mode = self.hist_scale.get()
        if mode == "Min/Max":
            a_lo, a_hi = robust_range(frame14)
        elif mode == "Manual":
            a_lo = int(self.offset.get())
            a_hi = a_lo + max(1, int(self.span.get()))
        else:  # Full range
            a_lo, a_hi = 0, maxv
        a_lo = max(0, min(a_lo, maxv - 1))
        a_hi = max(a_lo + 1, min(a_hi, maxv))
        bins = 128
        hist, _ = np.histogram(frame14, bins=bins, range=(a_lo, a_hi + 1))

        # Fill quantisation gaps.
        #
        # The source is 14-bit but the *populated* codes are often far coarser:
        # in 8-bit video mode, or after the camera's own gain steps, real values
        # land on a lattice (every 4th/16th code) while the codes between them
        # can never occur. Spread over 128 bins that renders as a picket fence
        # of isolated spikes with empty bins between them, which reads as a
        # broken histogram rather than a coarse one. Interpolate across bins
        # that are empty *between* populated ones — leaving the empty tails
        # outside the occupied range alone, since those are genuinely empty.
        occupied = np.nonzero(hist)[0]
        if occupied.size >= 2:
            first, last = occupied[0], occupied[-1]
            interior = np.arange(first, last + 1)
            gaps = hist[interior] == 0
            if gaps.any():
                hist = hist.astype(np.float32)
                hist[interior] = np.interp(interior, occupied, hist[occupied])

        hist = np.log1p(hist.astype(np.float32))
        peak = float(hist.max())
        if peak > 0:
            hist /= peak

        strip = bgr[h - hh:h]
        strip[:] = (strip.astype(np.uint16) * 2 // 5).astype(np.uint8)   # darken
        heights = (hist * (hh - 2)).astype(np.int32)

        if self.hist_style.get() == "Line":
            # Polyline across the strip at full width — no bins-wide upscale, so
            # the trace stays smooth instead of inheriting bar edges.
            xs = np.linspace(0, w - 1, bins).astype(np.int32)
            ys = (hh - 1 - heights).clip(0, hh - 1)
            pts = np.stack([xs, ys + (h - hh)], axis=1).astype(np.int32)
            cv2.polylines(bgr, [pts], False, (230, 230, 230), 1, cv2.LINE_AA)
        else:
            # Render bars into a bins-wide image, then stretch to full width.
            bar_img = np.zeros((hh, bins), dtype=np.uint8)
            for i, bh in enumerate(heights):
                if bh > 0:
                    bar_img[hh - bh:, i] = 230
            bar_img = cv2.resize(bar_img, (w, hh), interpolation=cv2.INTER_NEAREST)
            strip[:] = np.maximum(strip, cv2.cvtColor(bar_img, cv2.COLOR_GRAY2BGR))

        # Mark the display window on the same axis (only where it falls inside).
        span = a_hi - a_lo
        for v in (lo, hi):
            if a_lo <= v <= a_hi:
                x = int((v - a_lo) * (w - 1) / span)
                cv2.line(bgr, (x, h - hh), (x, h - 1), (255, 163, 77), 1)
        cv2.putText(bgr, f"{a_lo} - {a_hi}  win {lo}-{hi}", (4, h - hh + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1, cv2.LINE_AA)

    def _tick(self):
        frame14 = self.cam.latest()
        if frame14 is not None:
            self._proc = self._process(frame14)
            self.last_range = robust_range(frame14)
            self._draw(self._proc)

        if HAVE_SERIAL:
            self._sync_connect_btn()
            for line in self.serial.drain():
                if line.startswith(("—", ">>")):
                    self._append_return(line)
                else:
                    # Decode the framed reply/replies: show CMD,VALUES without
                    # the STX/length/checksum/ETX framing bytes, then colour by
                    # the status code (first value): 1=ok, 0=bad params, 2=bad cmd.
                    for s in SerialConsole.decode_replies(line):
                        self._show_reply(s)

        self._update_status()
        self.after(33, self._tick)

    def _draw(self, bgr):
        cw = max(1, self.canvas.winfo_width())
        ch = max(1, self.canvas.winfo_height())
        ih, iw = bgr.shape[:2]                     # actual frame dimensions
        scale = min(cw / iw, ch / ih)             # keep aspect ratio (letterbox)
        nw, nh = max(1, int(iw * scale)), max(1, int(ih * scale))
        interp = cv2.INTER_NEAREST if scale >= 1 else cv2.INTER_AREA
        resized = cv2.resize(bgr, (nw, nh), interpolation=interp)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        self._photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.canvas.delete("all")
        self.canvas.create_image(cw // 2, ch // 2, image=self._photo, anchor="center")

    def _update_status(self):
        lo, hi = self.last_range
        if getattr(self, "_flash_msg", None) and time.time() - self._flash_msg[1] < 2.5:
            extra = "   " + self._flash_msg[0]
        else:
            extra = ""
        if self.cam.connected:
            dot = f"● connected [{self.cam.mode}]"
        else:
            dot = "○ waiting for camera…"
        self.status.configure(
            text=f"{dot}    range: {lo} – {hi}    fps: {self.cam.fps:4.1f}    "
                 f"palette: {self.palette.get()}{extra}")

    def _on_close(self):
        self.cam.stop()
        try:
            self.serial.disconnect()
        except Exception:
            pass
        try:
            self.destroy()
        except Exception:
            pass
        # Close the owning console window (Windows), then hard-exit so no
        # process or terminal lingers after the GUI is closed.
        if os.name == "nt":
            try:
                import ctypes
                hwnd = ctypes.windll.kernel32.GetConsoleWindow()
                if hwnd:
                    ctypes.windll.user32.PostMessageW(hwnd, 0x0010, 0, 0)  # WM_CLOSE
            except Exception:
                pass
        os._exit(0)


if __name__ == "__main__":
    Viewer().mainloop()
