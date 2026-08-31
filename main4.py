"""
Industrial Defect Inspection & Lighting System
================================================
Changes made in this pass (see chat for full explanation/evidence):

RS-232 / ALT-E4RS light control:
  - Confirmed via the label's Data Matrix code + ALTSYSTEM's own product pages that this
    is an ALT-E4RS (Ethernet Controller series): RS-232C 9600-19200 bps only, 256-step
    (0-255) per-channel brightness, per-channel on/off I/O. Those structural assumptions
    in the original code were correct.
  - Could NOT confirm ALTSYSTEM's literal ASCII command syntax -- their site blocks
    automated fetches and it isn't publicly indexed. The "OPT (With/No Checksum)" and
    "Standard ASCII (SA)" formats already in the code are borrowed from OTHER vendors
    (OPT Machine Vision, and a CCS-style "@00Lx" ASCII format) -- not verified for this
    exact unit. That mismatch is the most likely reason nothing happens on the light,
    regardless of the checksum math.
  - Added a "Serial Monitor / Raw Command Console" panel so you can fire arbitrary
    strings at the controller and see exactly what goes out AND whatever comes back --
    turns this from guessing into something you can empirically test against the real
    hardware.
  - Fixed bugs that are wrong independent of which protocol turns out to be correct:
      * Baud list offered 115200, which this controller doesn't support (9600-19200 only)
      * No response read-back at all -- added a background reader thread + live log
      * dsrdtr not disabled -- can glitch some controllers via DTR toggling on port open
      * No settle delay after opening the port before the initial channel-enable burst
      * turn_all_lights_on/off() double-sent every command: calling slider.set() on a
        ttk.Scale re-invokes the -command callback (verified empirically), which was
        already sending its own enable/brightness commands -- so each click fired every
        command twice back to back

Core detection algorithm (Laplacian V1):
  - Found and fixed a real bug: cv2.convertScaleAbs() clipped the Laplacian response to
    0-255 BEFORE computing the mean/std used for the threshold. On any realistically
    textured/noisy frame, mean + k*std blows past 255 at almost any k >= ~2 -- including
    the app's own default k=3.0 -- so the detector silently found ZERO defects no matter
    how obvious they were. At very low k it swung the other way and flagged tens of
    thousands of noise pixels because there was no pre-blur. Verified both failure modes
    with synthetic test data before fixing.
  - Fix: compute statistics/threshold in float space (no premature 8-bit clipping) and
    added a small Gaussian pre-blur (new "Noise Blur" control) to suppress per-pixel
    sensor noise before differentiating.

UI:
  - Zoom In / Zoom Out / Fit buttons added next to Reset Zoom & Pan
  - Live connection status indicator (colored dot/text) instead of just button text
"""

import os
import sys
import glob
import time
import queue
import importlib.util
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import cv2
import numpy as np
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports

try:
    from pypylon import pylon
    PYLON_AVAILABLE = True
except ImportError:
    PYLON_AVAILABLE = False


# =============================================================================
# 1. THREADED CAMERA STREAM
# =============================================================================
class CameraStream:
    """Threaded camera capture to prevent UI freezing."""
    def __init__(self, src=0):
        if str(src).isdigit():
            src = int(src)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera source '{src}' via OpenCV. If this is meant to be a "
                f"Basler industrial camera, switch Camera Type to 'Basler (pypylon)' instead -- "
                f"plain OpenCV can't see GigE/USB3 Vision cameras."
            )
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame
            time.sleep(0.01)

    def read(self):
        with self.read_lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self.started = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        if self.cap.isOpened():
            self.cap.release()


class BaslerCameraStream:
    """
    Threaded Basler camera capture via pypylon.

    Plain cv2.VideoCapture(index) only enumerates standard OS-level UVC webcams --
    GigE/USB3 Vision industrial cameras like Basler's never show up there, which is
    exactly why pointing this app's old OpenCV-only source at a Basler unit raised
    "camera index out of range" (index 0 simply doesn't exist as a UVC device).
    This class talks to the camera the same way pypylon-based apps normally do
    (InstantCamera + ImageFormatConverter -> BGR8 numpy array), mirroring the
    approach already used for Basler capture elsewhere.
    """
    def __init__(self):
        self.camera = None
        self.converter = None
        self.started = False
        self.read_lock = threading.Lock()
        self.grabbed = False
        self.frame = None
        self.thread = None

    def start(self, serial_filter=None):
        if self.started:
            return self
        if not PYLON_AVAILABLE:
            raise RuntimeError(
                "pypylon not installed. Run: pip install pypylon --break-system-packages\n"
                "(Also needs Basler's pylon runtime installed on this machine.)"
            )

        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        if not devices:
            raise RuntimeError("No Basler camera detected. Check power, cable, and that "
                                "the pylon Viewer can see it.")

        device = devices[0]
        if serial_filter:
            matches = [d for d in devices if serial_filter in d.GetSerialNumber()]
            if matches:
                device = matches[0]
            else:
                raise RuntimeError(f"No connected Basler camera matches serial '{serial_filter}'. "
                                    f"Found: {[d.GetSerialNumber() for d in devices]}")

        self.camera = pylon.InstantCamera(tl_factory.CreateDevice(device))
        self.camera.Open()
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        self.started = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.started and self.camera is not None and self.camera.IsGrabbing():
            try:
                grab_result = self.camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    image = self.converter.Convert(grab_result)
                    frame = image.GetArray()
                    with self.read_lock:
                        self.grabbed = True
                        self.frame = frame
                grab_result.Release()
            except Exception as e:
                print(f"Basler grab error: {e}")
                time.sleep(0.05)

    def read(self):
        with self.read_lock:
            return self.grabbed, (self.frame.copy() if self.frame is not None else None)

    def stop(self):
        self.started = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.camera is not None:
            try:
                if self.camera.IsGrabbing():
                    self.camera.StopGrabbing()
                self.camera.Close()
            except Exception:
                pass


# =============================================================================
# 2. BUILT-IN DETECTOR LOGIC (Laplacian Defect Inspection)
# =============================================================================
def detect_laplacian_v1(img, k_multiplier=3.0, ksize=3, blur_ksize=3):
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    ksize = int(ksize)
    if ksize not in [1, 3, 5, 7]:
        ksize = 3

    # Pre-blur to suppress per-pixel sensor noise before differentiating. Without this,
    # the Laplacian response is dominated by noise rather than real structure, and any
    # threshold loose enough to catch real (low-contrast) defects also floods with
    # thousands of noise-driven false positives.
    blur_ksize = int(blur_ksize)
    if blur_ksize and blur_ksize >= 3:
        if blur_ksize % 2 == 0:
            blur_ksize += 1
        work = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
    else:
        work = gray

    laplacian = cv2.Laplacian(work, cv2.CV_64F, ksize=ksize)

    # Stay in float here -- do NOT clip to 0-255 before computing mean/std/threshold.
    # convertScaleAbs() clipping at this stage was the core bug: it caps every pixel at
    # 255, so mean + k*std routinely exceeds 255 on real, textured frames, and
    # cv2.threshold() can then never fire since no 8-bit pixel can be "above" a
    # threshold that's already off the top of the scale. The detector went completely
    # blind at the app's own default settings.
    lap_abs = np.abs(laplacian)
    mean_val = float(lap_abs.mean())
    std_dev = float(lap_abs.std())
    threshold_value = mean_val + (float(k_multiplier) * std_dev)

    binary_mask = (lap_abs > threshold_value).astype(np.uint8) * 255

    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result_img = img.copy()
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w > 2 or h > 2:
            cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 0, 255), 2)

    return result_img


# =============================================================================
# 3. MAIN INSPECTION APPLICATION
# =============================================================================
class VisionInspectionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Industrial Defect Inspection & Lighting System")
        self.root.geometry("1440x880")
        self.root.minsize(1100, 750)

        self._apply_dark_theme()

        self.ser = None
        self.cam = None

        # Serial reader (background thread -> queue -> UI poll)
        self.rx_queue = queue.Queue()
        self.serial_reader_thread = None
        self.serial_reader_running = False

        # Guard so programmatic slider.set() calls (e.g. from "Turn On Full") don't
        # re-trigger on_slider_move and double-send every command.
        self._suppress_slider_cb = False

        # Algorithm Management
        self.algorithms = {}
        self.load_algorithms()

        # Canvas Zoom, Pan & ROI States
        self.zoom_level = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.last_mouse_x = 0
        self.last_mouse_y = 0

        self.roi = None
        self.is_drawing_roi = False
        self.roi_screen_start = None
        self.roi_screen_current = None
        self.current_raw_frame = None

        self._build_layout()
        self.update_loop()

    def _apply_dark_theme(self):
        style = ttk.Style()
        style.theme_use("clam")

        bg_dark = "#1e1e24"
        panel_bg = "#2b2b36"
        accent_blue = "#007acc"
        fg_white = "#e0e0e0"

        self.root.configure(bg=bg_dark)

        style.configure(".", background=panel_bg, foreground=fg_white, font=("Segoe UI", 9))
        style.configure("TFrame", background=panel_bg)
        style.configure("TLabelframe", background=panel_bg, foreground="#61afef", font=("Segoe UI", 10, "bold"))
        style.configure("TLabelframe.Label", background=panel_bg, foreground="#61afef")
        style.configure("TLabel", background=panel_bg, foreground=fg_white)

        style.configure("TButton", background="#3e4451", foreground="white", borderwidth=0, padding=5)
        style.map("TButton", background=[("active", accent_blue)])

        style.configure("Accent.TButton", background=accent_blue, foreground="white", font=("Segoe UI", 9, "bold"))
        style.map("Accent.TButton", background=[("active", "#005f9e")])

        style.configure("Green.TButton", background="#28a745", foreground="white", font=("Segoe UI", 9, "bold"))
        style.map("Green.TButton", background=[("active", "#1e7e34")])

        style.configure("Red.TButton", background="#dc3545", foreground="white", font=("Segoe UI", 9, "bold"))
        style.map("Red.TButton", background=[("active", "#bd2130")])

        style.configure("TCombobox", fieldbackground="#1e1e24", background="#3e4451", foreground="white")
        style.configure("TEntry", fieldbackground="#1e1e24", foreground="white")

    def _build_layout(self):
        main_paned = ttk.PanedWindow(self.root, orient="horizontal")
        main_paned.pack(fill="both", expand=True)

        # Left Panel (Display Canvas)
        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=4)

        toolbar = ttk.Frame(left_frame, padding=6)
        toolbar.pack(fill="x", side="top")

        ttk.Button(toolbar, text="Zoom In", command=self.zoom_in).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Zoom Out", command=self.zoom_out).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Fit", command=self.zoom_fit).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Reset Zoom & Pan", command=self.reset_zoom).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Clear ROI", command=self.clear_roi).pack(side="left", padx=4)

        ttk.Label(
            toolbar,
            text="[Left Drag]: Draw ROI  |  [Right Drag]: Pan  |  [Scroll]: Zoom",
            foreground="#98c379"
        ).pack(side="right", padx=10)

        self.canvas = tk.Canvas(left_frame, bg="#121214", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)

        # Mouse Event Bindings
        self.canvas.bind("<MouseWheel>", self.on_zoom)
        self.canvas.bind("<Button-4>", self.on_zoom)
        self.canvas.bind("<Button-5>", self.on_zoom)

        self.canvas.bind("<ButtonPress-1>", self.on_roi_start)
        self.canvas.bind("<B1-Motion>", self.on_roi_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_roi_end)

        self.canvas.bind("<ButtonPress-3>", self.on_pan_start)
        self.canvas.bind("<B3-Motion>", self.on_pan_drag)

        # Right Panel (Controls)
        right_container = ttk.Frame(main_paned)
        main_paned.add(right_container, weight=1)

        right_canvas = tk.Canvas(right_container, bg="#2b2b36", borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(right_container, orient="vertical", command=right_canvas.yview)
        self.scroll_frame = ttk.Frame(right_canvas, padding=12)

        self.scroll_frame.bind("<Configure>", lambda e: right_canvas.configure(scrollregion=right_canvas.bbox("all")))
        right_canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")
        right_canvas.configure(yscrollcommand=scrollbar.set)

        right_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self._build_camera_controls()
        self._build_algorithm_controls()
        self._build_light_controls()
        self._build_serial_monitor()

    def _build_camera_controls(self):
        cam_frame = ttk.LabelFrame(self.scroll_frame, text=" Camera Controls ", padding=10)
        cam_frame.pack(fill="x", pady=6)

        ttk.Label(cam_frame, text="Camera Type:").grid(row=0, column=0, sticky="w", pady=4)
        backend_default = "Basler (pypylon)" if PYLON_AVAILABLE else "OpenCV (Index/URL)"
        self.cam_backend_cb = ttk.Combobox(
            cam_frame, values=["Basler (pypylon)", "OpenCV (Index/URL)"],
            state="readonly", width=18
        )
        self.cam_backend_cb.set(backend_default)
        self.cam_backend_cb.grid(row=0, column=1, columnspan=2, sticky="ew", padx=5, pady=4)

        if not PYLON_AVAILABLE:
            ttk.Label(
                cam_frame, text="pypylon not installed -- Basler option will fail until it's added",
                foreground="#e5c07b", font=("Segoe UI", 8)
            ).grid(row=1, column=0, columnspan=3, sticky="w")

        ttk.Label(cam_frame, text="Serial # filter (Basler) /\nIndex or URL (OpenCV):", justify="left").grid(row=2, column=0, sticky="w", pady=4)
        self.ent_cam_src = ttk.Entry(cam_frame, width=10)
        self.ent_cam_src.insert(0, "0")
        self.ent_cam_src.grid(row=2, column=1, padx=5, pady=4)

        self.btn_cam_toggle = ttk.Button(cam_frame, text="Start Stream", style="Accent.TButton", command=self.toggle_camera)
        self.btn_cam_toggle.grid(row=2, column=2, padx=5, pady=4)

    def _build_algorithm_controls(self):
        algo_frame = ttk.LabelFrame(self.scroll_frame, text=" Defect Detection Engine ", padding=10)
        algo_frame.pack(fill="x", pady=6)

        ttk.Label(algo_frame, text="Active Algorithm:").grid(row=0, column=0, sticky="w", pady=4)
        self.algo_cb = ttk.Combobox(algo_frame, values=list(self.algorithms.keys()), state="readonly")
        if self.algorithms:
            self.algo_cb.set("Laplacian V1 (Built-in)")
        self.algo_cb.grid(row=0, column=1, columnspan=2, sticky="ew", pady=4)

        ttk.Button(algo_frame, text="Reload Custom Plugins", command=self.reload_algorithms_list).grid(row=1, column=0, columnspan=3, sticky="ew", pady=4)

        ttk.Separator(algo_frame, orient="horizontal").grid(row=2, column=0, columnspan=3, sticky="ew", pady=8)

        ttk.Label(algo_frame, text="Sensitivity Multiplier:").grid(row=3, column=0, columnspan=2, sticky="w")
        self.lbl_k_val = ttk.Label(algo_frame, text="3.0", foreground="#61afef", font=("Segoe UI", 9, "bold"))
        self.lbl_k_val.grid(row=3, column=2, sticky="e")

        self.scale_k = ttk.Scale(algo_frame, from_=0.5, to=10.0, value=3.0, command=self._on_k_slider_move)
        self.scale_k.grid(row=4, column=0, columnspan=3, sticky="ew", pady=4)

        ttk.Label(algo_frame, text="Filter Kernel Size:").grid(row=5, column=0, sticky="w", pady=4)
        self.ksize_cb = ttk.Combobox(algo_frame, values=["1", "3", "5", "7"], width=5, state="readonly")
        self.ksize_cb.set("3")
        self.ksize_cb.grid(row=5, column=1, sticky="w", padx=5, pady=4)

        ttk.Label(algo_frame, text="Noise Blur (px):").grid(row=6, column=0, sticky="w", pady=4)
        self.blur_cb = ttk.Combobox(algo_frame, values=["0", "3", "5", "7"], width=5, state="readonly")
        self.blur_cb.set("3")
        self.blur_cb.grid(row=6, column=1, sticky="w", padx=5, pady=4)

    def _build_light_controls(self):
        conn_frame = ttk.LabelFrame(self.scroll_frame, text=" RS-232 Light Controller (ALT-E4RS) ", padding=10)
        conn_frame.pack(fill="x", pady=6)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=2, pady=2)
        self.port_cb = ttk.Combobox(conn_frame, values=self.get_ports(), width=8)
        ports = self.get_ports()
        if ports:
            self.port_cb.set(ports[0])
        self.port_cb.grid(row=0, column=1, padx=2, pady=2)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=2, pady=2)
        # ALT-E4RS only supports 9600-19200 bps per ALTSYSTEM's spec -- 115200 (which was
        # previously offered here) simply isn't a valid rate for this controller.
        self.baud_cb = ttk.Combobox(conn_frame, values=["19200", "9600"], width=8, state="readonly")
        self.baud_cb.set("19200")
        self.baud_cb.grid(row=0, column=3, padx=2, pady=2)

        ttk.Label(
            conn_frame, text="ALT-E4RS supports 9600-19200 bps only",
            foreground="#5c6370", font=("Segoe UI", 8)
        ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 4))

        ttk.Label(conn_frame, text="Protocol:").grid(row=2, column=0, padx=2, pady=4, sticky="w")
        self.proto_cb = ttk.Combobox(
            conn_frame,
            values=["OPT (With Checksum)", "OPT (No Checksum)", "Standard ASCII (SA)"],
            state="readonly",
            width=18
        )
        self.proto_cb.set("OPT (With Checksum)")
        self.proto_cb.grid(row=2, column=1, columnspan=3, sticky="ew", pady=4)

        ttk.Label(
            conn_frame,
            text="Unverified for this exact ALT-E4RS unit -- test in the\nSerial Monitor console below before relying on it.",
            foreground="#e5c07b", font=("Segoe UI", 8), justify="left"
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(0, 4))

        self.btn_connect = ttk.Button(conn_frame, text="Connect RS-232", style="Accent.TButton", command=self.toggle_serial_connection)
        self.btn_connect.grid(row=4, column=0, columnspan=4, sticky="ew", pady=4)

        self.lbl_conn_status = ttk.Label(conn_frame, text="\u25cf Disconnected", foreground="#e06c75", font=("Segoe UI", 9, "bold"))
        self.lbl_conn_status.grid(row=5, column=0, columnspan=4, sticky="w", pady=(0, 4))

        # Quick Test Buttons
        btn_box = ttk.Frame(conn_frame)
        btn_box.grid(row=6, column=0, columnspan=4, sticky="ew", pady=4)
        ttk.Button(btn_box, text="TURN ON FULL (255)", style="Green.TButton", command=self.turn_all_lights_on).pack(side="left", fill="x", expand=True, padx=2)
        ttk.Button(btn_box, text="TURN ALL OFF", style="Red.TButton", command=self.turn_all_lights_off).pack(side="right", fill="x", expand=True, padx=2)

        # Sliders
        ctrl_frame = ttk.LabelFrame(self.scroll_frame, text=" Brightness Adjust (0 - 255) ", padding=10)
        ctrl_frame.pack(fill="x", pady=6)

        self.sliders = []
        self.val_labels = []

        for ch in range(1, 5):
            row_frame = ttk.Frame(ctrl_frame)
            row_frame.pack(fill="x", pady=4)

            ttk.Label(row_frame, text=f"CH {ch}:", width=6, font=("Segoe UI", 9, "bold")).pack(side="left")

            slider = ttk.Scale(
                row_frame,
                from_=0,
                to=255,
                orient="horizontal",
                command=lambda val, c=ch: self.on_slider_move(c, val)
            )
            slider.pack(side="left", fill="x", expand=True, padx=6)
            self.sliders.append(slider)

            val_label = ttk.Label(row_frame, text="000", width=4, foreground="#e5c07b")
            val_label.pack(side="right")
            self.val_labels.append(val_label)

    def _build_serial_monitor(self):
        mon_frame = ttk.LabelFrame(self.scroll_frame, text=" Serial Monitor / Raw Command Console ", padding=10)
        mon_frame.pack(fill="x", pady=6)

        info = (
            "ALTSYSTEM's exact ASCII command set isn't public. Type a candidate "
            "string, send it, and watch this log for a reply (or watch the light "
            "itself) -- then wire whatever actually works into the controls above."
        )
        ttk.Label(mon_frame, text=info, foreground="#98c379", wraplength=280, justify="left").pack(anchor="w", pady=(0, 6))

        row = ttk.Frame(mon_frame)
        row.pack(fill="x", pady=2)
        self.ent_raw_cmd = ttk.Entry(row)
        self.ent_raw_cmd.pack(side="left", fill="x", expand=True)
        self.ent_raw_cmd.insert(0, "$1110")

        self.var_append_crlf = tk.BooleanVar(value=True)
        ttk.Checkbutton(mon_frame, text="Append \\r\\n", variable=self.var_append_crlf).pack(anchor="w", pady=(4, 0))

        btn_row = ttk.Frame(mon_frame)
        btn_row.pack(fill="x", pady=4)
        ttk.Button(btn_row, text="Insert XOR Checksum", command=self.insert_checksum).pack(side="left", padx=2)
        ttk.Button(btn_row, text="Send Raw", style="Accent.TButton", command=self.send_raw_console).pack(side="right", padx=2)

        log_frame = ttk.Frame(mon_frame)
        log_frame.pack(fill="both", expand=True, pady=(6, 0))
        self.txt_serial_log = tk.Text(
            log_frame, height=10, bg="#121214", fg="#98c379",
            font=("Consolas", 8), state="disabled", wrap="none"
        )
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.txt_serial_log.yview)
        self.txt_serial_log.configure(yscrollcommand=log_scroll.set)
        self.txt_serial_log.pack(side="left", fill="both", expand=True)
        log_scroll.pack(side="right", fill="y")

        ttk.Button(mon_frame, text="Clear Log", command=self.clear_serial_log).pack(anchor="e", pady=(4, 0))

    # -------------------------------------------------------------------------
    # HARDWARE RS-232 COMMUNICATION PROTOCOL ENGINE
    # -------------------------------------------------------------------------
    def _calculate_xor_checksum(self, cmd_str):
        chk = 0
        for char in cmd_str:
            chk ^= ord(char)
        return f"{chk:02X}"

    def _send_raw(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                data = cmd.encode('ascii')
                self.ser.write(data)
                self.ser.flush()
                self._log_serial(data, 'TX')
            except Exception as e:
                self._log_serial(f"WRITE ERROR: {e}", 'TX')
                print(f"RS-232 Write Error: {e}")
        else:
            self._log_serial("(not connected -- command not sent)", 'TX')

    def enable_channel(self, channel, state=True):
        proto = self.proto_cb.get()
        st_val = "1" if state else "0"

        if "With Checksum" in proto:
            payload = f"$1{channel}{st_val}"
            cmd = f"{payload}{self._calculate_xor_checksum(payload)}\r\n"
        elif "No Checksum" in proto:
            cmd = f"$1{channel}{st_val}\r\n"
        else:
            cmd = f"@00L{channel}{'255' if state else '000'}\r\n"
        self._send_raw(cmd)

    def send_brightness(self, channel, val):
        proto = self.proto_cb.get()
        if "With Checksum" in proto:
            payload = f"$3{channel}{val:03d}"
            cmd = f"{payload}{self._calculate_xor_checksum(payload)}\r\n"
        elif "No Checksum" in proto:
            cmd = f"$3{channel}{val:03d}\r\n"
        else:
            ch_letter = chr(ord('A') + channel - 1)
            cmd = f"S{ch_letter}{val:03d}\r\n"
        self._send_raw(cmd)

    def turn_all_lights_on(self):
        # Guard against slider.set() re-triggering on_slider_move (verified: ttk.Scale
        # invokes its -command on programmatic .set(), not just user drags), which would
        # otherwise send every enable/brightness command twice.
        self._suppress_slider_cb = True
        try:
            for ch in range(1, 5):
                self.enable_channel(ch, state=True)
                time.sleep(0.02)
                self.sliders[ch - 1].set(255)
                self.send_brightness(ch, 255)
                self.val_labels[ch - 1].config(text="255")
        finally:
            self._suppress_slider_cb = False

    def turn_all_lights_off(self):
        self._suppress_slider_cb = True
        try:
            for ch in range(1, 5):
                self.sliders[ch - 1].set(0)
                self.send_brightness(ch, 0)
                self.enable_channel(ch, state=False)
                self.val_labels[ch - 1].config(text="000")
        finally:
            self._suppress_slider_cb = False

    def on_slider_move(self, channel, val):
        val_int = int(float(val))
        self.val_labels[channel - 1].config(text=f"{val_int:03d}")
        if self._suppress_slider_cb:
            return
        if val_int > 0:
            self.enable_channel(channel, state=True)
        self.send_brightness(channel, val_int)

    def get_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["COM1", "COM3", "/dev/ttyUSB0"]

    def toggle_serial_connection(self):
        if self.ser and self.ser.is_open:
            self._stop_serial_reader()
            self.ser.close()
            self.ser = None
            self.btn_connect.config(text="Connect RS-232")
            self.lbl_conn_status.config(text="\u25cf Disconnected", foreground="#e06c75")
            messagebox.showinfo("Status", "Port Closed.")
        else:
            port = self.port_cb.get()
            baud = int(self.baud_cb.get())
            if not port:
                messagebox.showwarning("Warning", "Please select a COM port.")
                return
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2,
                    write_timeout=0.5,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,  # avoid toggling DTR on open, which resets/glitches some controllers
                )
                time.sleep(0.15)  # let the controller settle after the port opens
                self._start_serial_reader()
                self.btn_connect.config(text="Disconnect")
                self.lbl_conn_status.config(text="\u25cf Connected", foreground="#98c379")

                self._suppress_slider_cb = True
                for ch in range(1, 5):
                    self.enable_channel(ch, state=True)
                    time.sleep(0.02)
                self._suppress_slider_cb = False

                messagebox.showinfo("Connected", f"Connected to {port} @ {baud} bps")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    # -------------------------------------------------------------------------
    # SERIAL MONITOR / RAW CONSOLE
    # -------------------------------------------------------------------------
    def _start_serial_reader(self):
        self.serial_reader_running = True
        self.serial_reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.serial_reader_thread.start()
        self._poll_serial_rx()

    def _stop_serial_reader(self):
        self.serial_reader_running = False
        if self.serial_reader_thread:
            self.serial_reader_thread.join(timeout=1.0)
            self.serial_reader_thread = None

    def _serial_reader_loop(self):
        while self.serial_reader_running:
            try:
                if self.ser and self.ser.is_open:
                    data = self.ser.read(256)
                    if data:
                        self.rx_queue.put(data)
                else:
                    time.sleep(0.05)
            except Exception as e:
                self.rx_queue.put(('__ERROR__', str(e)))
                time.sleep(0.2)

    def _poll_serial_rx(self):
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, tuple) and item[0] == '__ERROR__':
                    self._log_serial(f"READ ERROR: {item[1]}", 'RX')
                else:
                    self._log_serial(item, 'RX')
        except queue.Empty:
            pass
        if self.serial_reader_running:
            self.root.after(50, self._poll_serial_rx)

    def _fmt_bytes(self, b):
        hex_part = ' '.join(f'{x:02X}' for x in b)
        ascii_part = ''.join(chr(x) if 32 <= x < 127 else '.' for x in b)
        return hex_part, ascii_part

    def _log_serial(self, raw, tag):
        if not hasattr(self, 'txt_serial_log'):
            return
        ts = time.strftime('%H:%M:%S')
        if isinstance(raw, bytes):
            hex_part, ascii_part = self._fmt_bytes(raw)
            line = f"[{ts}] {tag:<3} {hex_part}   |{ascii_part}|\n"
        else:
            line = f"[{ts}] {tag:<3} {raw}\n"
        self.txt_serial_log.configure(state='normal')
        self.txt_serial_log.insert('end', line)
        num_lines = int(self.txt_serial_log.index('end-1c').split('.')[0])
        if num_lines > 500:
            self.txt_serial_log.delete('1.0', f'{num_lines - 500}.0')
        self.txt_serial_log.see('end')
        self.txt_serial_log.configure(state='disabled')

    def clear_serial_log(self):
        self.txt_serial_log.configure(state='normal')
        self.txt_serial_log.delete('1.0', 'end')
        self.txt_serial_log.configure(state='disabled')

    def send_raw_console(self):
        text = self.ent_raw_cmd.get()
        if not text:
            return
        payload = text
        if self.var_append_crlf.get():
            payload += "\r\n"
        self._send_raw(payload)

    def insert_checksum(self):
        text = self.ent_raw_cmd.get()
        if not text:
            return
        chk = self._calculate_xor_checksum(text)
        self.ent_raw_cmd.delete(0, 'end')
        self.ent_raw_cmd.insert(0, text + chk)

    # -------------------------------------------------------------------------
    # PLUGIN ENGINE & SENSITIVITY
    # -------------------------------------------------------------------------
    def load_algorithms(self):
        self.algorithms = {"Laplacian V1 (Built-in)": detect_laplacian_v1}
        algo_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "algorithms")
        os.makedirs(algo_dir, exist_ok=True)
        for file_path in glob.glob(os.path.join(algo_dir, "*.py")):
            filename = os.path.basename(file_path)
            if filename.startswith("__"):
                continue
            module_name = filename[:-3]
            try:
                spec = importlib.util.spec_from_file_location(module_name, file_path)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                if hasattr(mod, "process_frame"):
                    algo_title = getattr(mod, "NAME", module_name)
                    self.algorithms[algo_title] = mod.process_frame
            except Exception as e:
                print(f"Plugin load error {filename}: {e}")

    def reload_algorithms_list(self):
        self.load_algorithms()
        self.algo_cb['values'] = list(self.algorithms.keys())
        messagebox.showinfo("Plugins", f"Loaded {len(self.algorithms)} algorithms.")

    def _on_k_slider_move(self, val):
        self.lbl_k_val.config(text=f"{float(val):.1f}")

    # -------------------------------------------------------------------------
    # CAMERA CONTROLS & CANVAS INTERACTIVITY
    # -------------------------------------------------------------------------
    def toggle_camera(self):
        if self.cam and self.cam.started:
            self.cam.stop()
            self.cam = None
            self.btn_cam_toggle.config(text="Start Stream")
        else:
            backend = self.cam_backend_cb.get()
            src = self.ent_cam_src.get().strip()
            try:
                if backend.startswith("Basler"):
                    serial_filter = src if src and src != "0" else None
                    self.cam = BaslerCameraStream().start(serial_filter=serial_filter)
                else:
                    self.cam = CameraStream(src).start()
                self.btn_cam_toggle.config(text="Stop Stream")
            except Exception as e:
                messagebox.showerror("Camera Error", f"Cannot open camera:\n{e}")

    def reset_zoom(self):
        self.zoom_level = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.roi = None

    def clear_roi(self):
        self.roi = None

    def zoom_in(self):
        self._zoom_at_canvas_center(1.2)

    def zoom_out(self):
        self._zoom_at_canvas_center(1 / 1.2)

    def _zoom_at_canvas_center(self, factor):
        cx = self.canvas.winfo_width() / 2
        cy = self.canvas.winfo_height() / 2
        new_zoom = self.zoom_level * factor
        if 0.15 <= new_zoom <= 15.0:
            self.pan_x = cx - (cx - self.pan_x) * factor
            self.pan_y = cy - (cy - self.pan_y) * factor
            self.zoom_level = new_zoom

    def zoom_fit(self):
        if self.current_raw_frame is None:
            self.reset_zoom()
            return
        h, w = self.current_raw_frame.shape[:2]
        cw = max(self.canvas.winfo_width(), 1)
        ch = max(self.canvas.winfo_height(), 1)
        fit_zoom = min(cw / w, ch / h)
        fit_zoom = max(0.15, min(fit_zoom, 15.0))
        self.zoom_level = fit_zoom
        self.pan_x = (cw - w * fit_zoom) / 2
        self.pan_y = (ch - h * fit_zoom) / 2

    def screen_to_img_coords(self, sx, sy):
        ix = int((sx - self.pan_x) / self.zoom_level)
        iy = int((sy - self.pan_y) / self.zoom_level)
        return ix, iy

    def on_zoom(self, event):
        zoom_factor = 1.15 if (getattr(event, 'num', 0) == 4 or getattr(event, 'delta', 0) > 0) else 0.85
        new_zoom = self.zoom_level * zoom_factor
        if 0.15 <= new_zoom <= 15.0:
            mx, my = event.x, event.y
            self.pan_x = mx - (mx - self.pan_x) * zoom_factor
            self.pan_y = my - (my - self.pan_y) * zoom_factor
            self.zoom_level = new_zoom

    def on_pan_start(self, event):
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def on_pan_drag(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.pan_x += dx
        self.pan_y += dy
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def on_roi_start(self, event):
        self.is_drawing_roi = True
        self.roi_screen_start = (event.x, event.y)
        self.roi_screen_current = (event.x, event.y)

    def on_roi_drag(self, event):
        if self.is_drawing_roi:
            self.roi_screen_current = (event.x, event.y)

    def on_roi_end(self, event):
        if not self.is_drawing_roi or not self.roi_screen_start:
            return
        self.is_drawing_roi = False
        x1, y1 = self.screen_to_img_coords(self.roi_screen_start[0], self.roi_screen_start[1])
        x2, y2 = self.screen_to_img_coords(event.x, event.y)
        if abs(x2 - x1) > 8 and abs(y2 - y1) > 8:
            self.roi = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
        self.roi_screen_start = None
        self.roi_screen_current = None

    def update_loop(self):
        if self.cam and self.cam.started:
            grabbed, frame = self.cam.read()
            if grabbed and frame is not None:
                self.current_raw_frame = frame

        if self.current_raw_frame is None:
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Camera Inactive", (220, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 120), 2)
            display_frame = placeholder
        else:
            display_frame = self.process_frame(self.current_raw_frame)

        self._render_canvas(display_frame)
        self.root.after(30, self.update_loop)

    def process_frame(self, frame):
        out_frame = frame.copy()
        h, w = out_frame.shape[:2]

        selected_algo = self.algo_cb.get()
        algo_fn = self.algorithms.get(selected_algo, detect_laplacian_v1)
        params = {
            'k_multiplier': float(self.scale_k.get()),
            'ksize': int(self.ksize_cb.get()),
            'blur_ksize': int(self.blur_cb.get()),
        }

        if self.roi:
            x1, y1, x2, y2 = self.roi
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            if x2 > x1 and y2 > y1:
                roi_crop = out_frame[y1:y2, x1:x2]
                try:
                    processed_roi = algo_fn(roi_crop, **params)
                except TypeError:
                    processed_roi = algo_fn(roi_crop, params)

                out_frame[y1:y2, x1:x2] = processed_roi
                cv2.rectangle(out_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        else:
            try:
                out_frame = algo_fn(out_frame, **params)
            except TypeError:
                out_frame = algo_fn(out_frame, params)

        return out_frame

    def _render_canvas(self, frame):
        h, w = frame.shape[:2]
        new_w = max(1, int(w * self.zoom_level))
        new_h = max(1, int(h * self.zoom_level))

        rgb_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized_img = cv2.resize(rgb_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pil_img = Image.fromarray(resized_img)
        self.tk_img = ImageTk.PhotoImage(image=pil_img)

        self.canvas.delete("all")
        self.canvas.create_image(self.pan_x, self.pan_y, anchor="nw", image=self.tk_img)

        # Draw ROI selection preview box dynamically on canvas
        if self.is_drawing_roi and self.roi_screen_start and self.roi_screen_current:
            x0, y0 = self.roi_screen_start
            x1, y1 = self.roi_screen_current
            self.canvas.create_rectangle(x0, y0, x1, y1, outline="#00e676", width=2, dash=(4, 2))


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = VisionInspectionApp(root)
    root.mainloop()
