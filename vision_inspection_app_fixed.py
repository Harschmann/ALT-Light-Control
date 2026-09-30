import os
import sys
import glob
import time
import importlib.util
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

import serial
import serial.tools.list_ports


# =============================================================================
# BASLER SUPPORT
# =============================================================================
try:
    from pypylon import pylon
    BASLER_AVAILABLE = True
except ImportError:
    pylon = None
    BASLER_AVAILABLE = False


# =============================================================================
# ALT-E4RS LIGHT PROTOCOL (verified)
# =============================================================================
def build_light_frame(levels):
    """
    Build the confirmed ALT-E4RS-12V frame:

        [0]     0x4C
        [1]     0x15
        [2..5]  channel 1-4 level, 0-255
        [6]     XOR checksum of bytes 1..5
        [7]     0x0D  (CR)
        [8]     0x0A  (LF)

    Reverse-engineered from Viskit.Components.Light.AltLightERS
    .ChangeLightValue4Ch and confirmed working over RS-232 at 9600 8N1.

    BUG THIS REPLACES: this file used to send `f"$3{channel}{val:03d}\\r\\n"`
    -- a guessed ASCII command, one channel at a time. That format was never
    verified against the real hardware, and more importantly the device has
    NO way to address a single channel independently -- every command frame
    must carry all 4 channel levels together, or the write simply does
    nothing (which is exactly the "code you gave me doesn't work" symptom
    from earlier in this conversation, now regressed back into this file).
    """
    levels = list(levels)[:4] + [0] * (4 - len(levels))
    msg = bytearray(9)
    msg[0] = 0x4C
    msg[1] = 0x15
    for i in range(4):
        msg[2 + i] = max(0, min(255, int(levels[i])))
    msg[6] = msg[1] ^ msg[2] ^ msg[3] ^ msg[4] ^ msg[5]
    msg[7] = 0x0D
    msg[8] = 0x0A
    return bytes(msg)


# =============================================================================
# 1. CAMERA THREAD WORKER
# =============================================================================
class CameraStream:
    """Threaded camera capture supporting OpenCV and Basler cameras."""

    def __init__(self, src=0):
        self.started = False
        self.read_lock = threading.Lock()

        self.grabbed = False
        self.frame = None

        self.cap = None
        self.camera = None
        self.is_basler = False

        # ---------------------------------------------------------------------
        # BASLER CAMERA
        # ---------------------------------------------------------------------
        if isinstance(src, str) and src.startswith("Basler:"):

            if not BASLER_AVAILABLE:
                raise RuntimeError(
                    "pypylon is not installed.\n\n"
                    "Install it using:\n"
                    "pip install pypylon"
                )

            self.is_basler = True

            full_name = src[len("Basler:"):].strip()

            factory = pylon.TlFactory.GetInstance()
            devices = factory.EnumerateDevices()

            selected_device = None

            for device in devices:
                if device.GetFullName() == full_name:
                    selected_device = device
                    break

            if selected_device is None:
                raise RuntimeError(
                    f"Basler camera not found:\n{full_name}"
                )

            self.camera = pylon.InstantCamera(
                factory.CreateDevice(selected_device)
            )

            self.camera.Open()

            # Latest image only -> prevents frame lag
            self.camera.StartGrabbing(
                pylon.GrabStrategy_LatestImageOnly
            )

            self.converter = pylon.ImageFormatConverter()

            self.converter.OutputPixelFormat = (
                pylon.PixelType_BGR8packed
            )

            self.converter.OutputBitAlignment = (
                pylon.OutputBitAlignment_MsbAligned
            )

            # Grab initial frame
            result = self.camera.RetrieveResult(
                5000,
                pylon.TimeoutHandling_ThrowException
            )

            if result.GrabSucceeded():
                image = self.converter.Convert(result)
                self.frame = image.GetArray().copy()
                self.grabbed = True

            result.Release()

        # ---------------------------------------------------------------------
        # NORMAL OPENCV CAMERA / VIDEO SOURCE
        # ---------------------------------------------------------------------
        else:

            if str(src).isdigit():
                src = int(src)

            self.cap = cv2.VideoCapture(src)

            if not self.cap.isOpened():
                raise RuntimeError(
                    f"Unable to open camera/source: {src}"
                )

            self.grabbed, self.frame = self.cap.read()

    def start(self):

        if self.started:
            return self

        self.started = True

        self.thread = threading.Thread(
            target=self.update,
            daemon=True
        )

        self.thread.start()

        return self

    def update(self):

        while self.started:

            # ================================================================
            # BASLER
            # ================================================================
            if self.is_basler:

                result = None
                try:

                    if not self.camera.IsGrabbing():
                        break

                    result = self.camera.RetrieveResult(
                        1000,
                        pylon.TimeoutHandling_Return
                    )

                    if result.GrabSucceeded():

                        image = self.converter.Convert(result)

                        frame = image.GetArray()

                        with self.read_lock:
                            self.grabbed = True
                            self.frame = frame.copy()

                except Exception as e:
                    print(f"Basler grab error: {e}")
                finally:
                    # BUG FIX: result.Release() used to sit only after the
                    # GrabSucceeded block, outside any try/finally. If
                    # self.converter.Convert(result) raised (a real
                    # possibility -- format conversion errors happen), the
                    # exception jumped straight to `except`, and
                    # result.Release() was never called. Each skipped
                    # release leaks one of the camera's internal grab
                    # buffers; over a long inspection shift this exhausts
                    # Basler's buffer pool and the camera silently stops
                    # delivering frames. Releasing in `finally` guarantees
                    # it happens whether or not conversion succeeded.
                    if result is not None:
                        try:
                            result.Release()
                        except Exception:
                            pass

            # ================================================================
            # OPENCV
            # ================================================================
            else:

                grabbed, frame = self.cap.read()

                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame

                time.sleep(0.01)

    def read(self):

        with self.read_lock:

            return (
                self.grabbed,
                self.frame.copy()
                if self.frame is not None
                else None
            )

    def stop(self):

        self.started = False

        if hasattr(self, "thread"):
            self.thread.join(timeout=1.0)

        # ---------------------------------------------------------------------
        # BASLER CLEANUP
        # ---------------------------------------------------------------------
        if self.is_basler:

            try:

                if self.camera and self.camera.IsGrabbing():
                    self.camera.StopGrabbing()

                if self.camera and self.camera.IsOpen():
                    self.camera.Close()

            except Exception as e:
                print(f"Basler close error: {e}")

        # ---------------------------------------------------------------------
        # OPENCV CLEANUP
        # ---------------------------------------------------------------------
        else:

            if self.cap is not None and self.cap.isOpened():
                self.cap.release()


from collections import deque


# =============================================================================
# 2. BUILT-IN DETECTOR LOGIC (Laplacian V1)
# =============================================================================
class LaplacianDetectorV1:
    """
    Kept as a callable CLASS (not a plain function) instead of a plain function
    for one reason only: it needs to remember the last few frames' detection
    masks. That's what fixes the random few-millisecond flicker -- everything
    else about it is called exactly the same way (algo_fn(img, **params)) as
    before, so it's a drop-in replacement.

    Two separate problems were causing the flicker, fixed together here:

    1) GLOBAL threshold: the old version compared every pixel's Laplacian
       response to ONE mean+k*std computed over the WHOLE frame/ROI. Sensor
       noise shifts that whole-frame average slightly from frame to frame, so
       pixels sitting right near the line jump back and forth across it just
       from noise -- a different random pixel each frame. Fixed by using a
       LOCAL sliding-window mean/std (cv2.boxFilter, same technique as the
       dust_inspector app's proven local Z-score) computed directly on the
       Laplacian response, so a pixel is only flagged if it stands out from
       its own neighborhood, not the whole image.
    2) NO temporal check: even a perfect single-frame threshold will
       occasionally flag one stray pixel purely from random noise, and a
       single frame's stray box is exactly what reads as a "few-millisecond
       blink". A REAL defect is flagged in basically every frame, not just
       one. So: keep a short rolling buffer of the last few frames' raw masks
       and only draw a box where a pixel was flagged in at least min_hits of
       them. A real defect trivially passes (it's in all of them); an
       isolated noise blip in 1 frame out of 3 does not.
    """

    def __init__(self):
        self._buffer = deque(maxlen=3)
        self._buffer_len = 3
        self._last_shape = None

    def __call__(self, img, k_multiplier=3.0, ksize=3, window=51,
                 persist_frames=3, persist_hits=2):
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        ksize = int(ksize)
        if ksize not in [1, 3, 5, 7]:
            ksize = 3

        window = int(window)
        if window < 3:
            window = 3
        if window % 2 == 0:
            window += 1

        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=ksize)
        lap_abs = np.abs(laplacian)  # float -- no premature 8-bit clipping

        mean_local = cv2.boxFilter(lap_abs, ddepth=-1, ksize=(window, window),
                                    normalize=True, borderType=cv2.BORDER_REFLECT)
        mean_sq_local = cv2.boxFilter(lap_abs * lap_abs, ddepth=-1, ksize=(window, window),
                                       normalize=True, borderType=cv2.BORDER_REFLECT)
        var_local = np.clip(mean_sq_local - mean_local * mean_local, 0, None)
        std_local = np.sqrt(var_local)

        threshold_local = mean_local + (float(k_multiplier) * std_local)
        raw_mask = (lap_abs > threshold_local).astype(np.uint8)

        # --- temporal persistence filter ---
        persist_frames = max(1, int(persist_frames))
        persist_hits = max(1, int(persist_hits))

        if self._buffer_len != persist_frames:
            self._buffer_len = persist_frames
            self._buffer = deque(maxlen=persist_frames)

        if self._last_shape != raw_mask.shape:
            # ROI/size changed (ROI drawn/cleared, zoom re-crop, etc.) -- old
            # frames aren't comparable anymore, so start the buffer over.
            self._buffer.clear()
            self._last_shape = raw_mask.shape

        self._buffer.append(raw_mask)

        if persist_hits <= 1 or len(self._buffer) < 2:
            final_mask = raw_mask * 255
        else:
            hit_count = np.sum(self._buffer, axis=0)
            effective_hits = min(persist_hits, len(self._buffer))
            final_mask = (hit_count >= effective_hits).astype(np.uint8) * 255

        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result_img = img.copy()
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w > 2 or h > 2:
                cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 0, 255), 2)

        return result_img


# =============================================================================
# 3. MAIN APPLICATION GUI
# =============================================================================
class VisionInspectionApp:

    def __init__(self, root):

        self.root = root

        self.root.title(
            "Industrial Defect Inspection & Lighting Control System"
        )

        self.root.geometry("1400x850")
        self.root.minsize(1000, 700)

        # ---------------------------------------------------------------------
        # DARK INDUSTRIAL THEME
        # ---------------------------------------------------------------------
        self.bg_color = "#111418"
        self.panel_color = "#191d22"
        self.panel2_color = "#20252b"
        self.accent_color = "#00a8ff"
        self.text_color = "#e6e6e6"

        self.root.configure(bg=self.bg_color)

        style = ttk.Style()

        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure(
            ".",
            background=self.panel_color,
            foreground=self.text_color,
            font=("Segoe UI", 10)
        )

        style.configure(
            "TFrame",
            background=self.panel_color
        )

        style.configure(
            "TLabel",
            background=self.panel_color,
            foreground=self.text_color
        )

        style.configure(
            "TLabelframe",
            background=self.panel_color,
            foreground=self.accent_color,
            bordercolor="#343a40"
        )

        style.configure(
            "TLabelframe.Label",
            background=self.panel_color,
            foreground=self.accent_color,
            font=("Segoe UI", 10, "bold")
        )

        style.configure(
            "TButton",
            background=self.panel2_color,
            foreground=self.text_color,
            padding=6
        )

        style.map(
            "TButton",
            background=[
                ("active", "#29313a")
            ]
        )

        style.configure(
            "TCombobox",
            fieldbackground=self.panel2_color,
            background=self.panel2_color,
            foreground=self.text_color
        )

        style.configure(
            "TScale",
            background=self.panel_color
        )

        style.configure(
            "TPanedwindow",
            background=self.bg_color
        )

        # ---------------------------------------------------------------------
        # Serial Connection Instance
        # ---------------------------------------------------------------------
        self.ser = None

        # All 4 channel levels, sent together in every frame (the device
        # can't be addressed one channel at a time -- see build_light_frame).
        self.channel_values = [0, 0, 0, 0]

        # Non-blocking send throttle: the ALT-E4RS needs a minimum gap
        # between commands, and dragging a slider fires far faster than
        # that. A literal time.sleep() here would freeze the UI, so instead
        # a send is either issued immediately (if enough time has passed)
        # or deferred once via root.after() -- any further slider moves
        # before it fires just get folded into that one pending send.
        self._last_light_send_time = 0.0
        self._pending_light_send_id = None

        # ---------------------------------------------------------------------
        # Camera Instance
        # ---------------------------------------------------------------------
        self.cam = None

        # Camera discovery map
        self.camera_map = {}

        # ---------------------------------------------------------------------
        # Algorithm Plugin Engine
        # ---------------------------------------------------------------------
        self.algorithms = {}
        self.load_algorithms()

        # ---------------------------------------------------------------------
        # Transformation & ROI State
        # ---------------------------------------------------------------------
        self.zoom_level = 1.0
        self.pan_x = 0
        self.pan_y = 0

        self.last_mouse_x = 0
        self.last_mouse_y = 0

        # ROI in Image Pixel Space
        self.roi = None
        self.is_drawing_roi = False
        self.roi_start = None

        # ---------------------------------------------------------------------
        # Frame Processing
        # ---------------------------------------------------------------------
        self.current_raw_frame = None

        # Detection now runs on a background thread (see _dispatch_detection)
        # instead of blocking update_loop's 30ms Tk-main-thread tick, which
        # was the actual cause of the reported live-feed lag.
        self._detection_busy = False
        self._display_lock = threading.Lock()
        self._latest_processed_frame = None

        self._build_layout()

        # BUG FIX: there was no WM_DELETE_WINDOW handler at all. Closing the
        # window used to just tear down the Tk root directly -- the camera
        # thread (daemon=True) would die with the process, but the serial
        # port was never closed cleanly and, more importantly, WHATEVER
        # BRIGHTNESS THE CHANNELS WERE LAST SET TO STAYS ON THE LIGHT after
        # the app exits, since nothing ever sent an all-off frame. Now
        # closing the app explicitly turns the lights off first.
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.update_loop()

    def on_close(self):

        try:
            if self.ser and self.ser.is_open:
                self.channel_values = [0, 0, 0, 0]
                try:
                    self.ser.write(build_light_frame(self.channel_values))
                except Exception as e:
                    print(f"All-off on close failed: {e}")
                self.ser.close()
        except Exception as e:
            print(f"Serial cleanup on close failed: {e}")

        try:
            if self.cam and self.cam.started:
                self.cam.stop()
        except Exception as e:
            print(f"Camera cleanup on close failed: {e}")

        self.root.destroy()

    # =========================================================================
    # GUI LAYOUT
    # =========================================================================
    def _build_layout(self):

        # Main Split Frame
        main_paned = ttk.PanedWindow(
            self.root,
            orient="horizontal"
        )

        main_paned.pack(
            fill="both",
            expand=True
        )

        # ---------------------------------------------------------------------
        # LEFT PANEL
        # ---------------------------------------------------------------------
        left_frame = ttk.Frame(main_paned)

        main_paned.add(
            left_frame,
            weight=3
        )

        # ---------------------------------------------------------------------
        # TOOLBAR
        # ---------------------------------------------------------------------
        toolbar = ttk.Frame(
            left_frame,
            padding=5
        )

        toolbar.pack(
            fill="x",
            side="top"
        )

        ttk.Button(
            toolbar,
            text="Reset View / Zoom",
            command=self.reset_zoom
        ).pack(
            side="left",
            padx=5
        )

        ttk.Button(
            toolbar,
            text="Clear ROI",
            command=self.clear_roi
        ).pack(
            side="left",
            padx=5
        )

        self.lbl_info = ttk.Label(
            toolbar,
            text=(
                "Left-Click + Drag: Draw ROI   |   "
                "Right-Click + Drag: Pan   |   "
                "Scroll: Zoom"
            )
        )

        self.lbl_info.pack(
            side="right",
            padx=10
        )

        # ---------------------------------------------------------------------
        # CANVAS
        # ---------------------------------------------------------------------
        self.canvas = tk.Canvas(
            left_frame,
            bg="#090b0d",
            highlightthickness=0
        )

        self.canvas.pack(
            fill="both",
            expand=True
        )

        # Mouse bindings
        self.canvas.bind(
            "<MouseWheel>",
            self.on_zoom
        )

        self.canvas.bind(
            "<Button-4>",
            self.on_zoom
        )

        self.canvas.bind(
            "<Button-5>",
            self.on_zoom
        )

        self.canvas.bind(
            "<ButtonPress-1>",
            self.on_roi_start
        )

        self.canvas.bind(
            "<B1-Motion>",
            self.on_roi_drag
        )

        self.canvas.bind(
            "<ButtonRelease-1>",
            self.on_roi_end
        )

        self.canvas.bind(
            "<ButtonPress-3>",
            self.on_pan_start
        )

        self.canvas.bind(
            "<B3-Motion>",
            self.on_pan_drag
        )

        # ---------------------------------------------------------------------
        # RIGHT PANEL
        # ---------------------------------------------------------------------
        right_container = ttk.Frame(main_paned)

        main_paned.add(
            right_container,
            weight=1
        )

        right_canvas = tk.Canvas(
            right_container,
            borderwidth=0,
            highlightthickness=0,
            bg=self.panel_color
        )

        scrollbar = ttk.Scrollbar(
            right_container,
            orient="vertical",
            command=right_canvas.yview
        )

        self.scroll_frame = ttk.Frame(
            right_canvas,
            padding=10
        )

        self.scroll_frame.bind(
            "<Configure>",
            lambda e: right_canvas.configure(
                scrollregion=right_canvas.bbox("all")
            )
        )

        right_canvas.create_window(
            (0, 0),
            window=self.scroll_frame,
            anchor="nw"
        )

        right_canvas.configure(
            yscrollcommand=scrollbar.set
        )

        right_canvas.pack(
            side="left",
            fill="both",
            expand=True
        )

        scrollbar.pack(
            side="right",
            fill="y"
        )

        # Sections
        self._build_camera_controls()
        self._build_algorithm_controls()
        self._build_light_controls()

    # =========================================================================
    # CAMERA CONTROLS
    # =========================================================================
    def _build_camera_controls(self):

        cam_frame = ttk.LabelFrame(
            self.scroll_frame,
            text=" Camera Controls ",
            padding=10
        )

        cam_frame.pack(
            fill="x",
            pady=5
        )

        ttk.Label(
            cam_frame,
            text="Camera:"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=5
        )

        self.ent_cam_src = ttk.Combobox(
            cam_frame,
            width=32,
            state="readonly"
        )

        self.ent_cam_src.grid(
            row=0,
            column=1,
            padx=5,
            pady=5
        )

        self.btn_cam_toggle = ttk.Button(
            cam_frame,
            text="Start Camera",
            command=self.toggle_camera
        )

        self.btn_cam_toggle.grid(
            row=0,
            column=2,
            padx=5,
            pady=5
        )

        ttk.Button(
            cam_frame,
            text="\u21bb",
            width=3,
            command=self.refresh_cameras
        ).grid(
            row=0,
            column=3,
            padx=2
        )

        self.refresh_cameras()

    # =========================================================================
    # CAMERA DISCOVERY
    # =========================================================================
    def refresh_cameras(self):
        """
        BUG FIX: this used to open+close cv2.VideoCapture(index) for
        indices 0-9 SYNCHRONOUSLY on the Tk main thread, every time this
        ran (including once automatically on startup). Each failed open
        can take well over a second on some backends, so a refresh could
        freeze the entire app -- window doesn't repaint, camera preview
        stalls, sliders stop responding -- for several seconds at a time.
        The actual probing now runs on a background thread; only the final
        combobox update is marshaled back onto the main thread.
        """

        if getattr(self, "_refreshing_cameras", False):
            return
        self._refreshing_cameras = True

        self.ent_cam_src["values"] = ["Scanning for cameras..."]
        self.ent_cam_src.current(0)

        threading.Thread(
            target=self._refresh_cameras_worker,
            daemon=True
        ).start()

    def _refresh_cameras_worker(self):

        cameras = []

        # ---------------------------------------------------------------------
        # BASLER CAMERAS
        # ---------------------------------------------------------------------
        if BASLER_AVAILABLE:

            try:

                factory = pylon.TlFactory.GetInstance()

                devices = factory.EnumerateDevices()

                for device in devices:

                    model = device.GetModelName()
                    serial_no = device.GetSerialNumber()
                    full_name = device.GetFullName()

                    display_name = (
                        f"Basler: {model} | SN: {serial_no}"
                    )

                    cameras.append(
                        (
                            display_name,
                            f"Basler:{full_name}"
                        )
                    )

            except Exception as e:

                print(
                    f"Basler camera discovery error: {e}"
                )

        # ---------------------------------------------------------------------
        # NORMAL OPENCV CAMERAS
        # ---------------------------------------------------------------------
        for index in range(10):

            cap = None

            try:

                cap = cv2.VideoCapture(index)

                if cap.isOpened():

                    cameras.append(
                        (
                            f"OpenCV Camera {index}",
                            str(index)
                        )
                    )

            except Exception as e:

                print(
                    f"OpenCV camera {index}: {e}"
                )

            finally:

                if cap is not None:
                    cap.release()

        self.root.after(0, lambda: self._apply_camera_list(cameras))

    def _apply_camera_list(self, cameras):

        self._refreshing_cameras = False

        # ---------------------------------------------------------------------
        # UPDATE CAMERA MAP
        # ---------------------------------------------------------------------
        self.camera_map = {
            display: source
            for display, source in cameras
        }

        display_values = list(
            self.camera_map.keys()
        )

        self.ent_cam_src["values"] = display_values

        if display_values:

            self.ent_cam_src.current(0)

        else:

            self.ent_cam_src["values"] = [
                "No camera found"
            ]

            self.ent_cam_src.current(0)

        print(
            f"Detected {len(display_values)} camera(s)"
        )

    # =========================================================================
    # ALGORITHM CONTROLS
    # =========================================================================
    def _build_algorithm_controls(self):

        algo_frame = ttk.LabelFrame(
            self.scroll_frame,
            text=" Detection Algorithm & Controls ",
            padding=10
        )

        algo_frame.pack(
            fill="x",
            pady=5
        )

        ttk.Label(
            algo_frame,
            text="Select Algorithm:"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=5
        )

        self.algo_cb = ttk.Combobox(
            algo_frame,
            values=list(self.algorithms.keys()),
            state="readonly"
        )

        if self.algorithms:
            self.algo_cb.set(
                "Laplacian V1 (Built-in)"
            )

        self.algo_cb.grid(
            row=0,
            column=1,
            columnspan=2,
            sticky="ew",
            pady=5
        )

        ttk.Button(
            algo_frame,
            text="Reload Algorithms",
            command=self.reload_algorithms_list
        ).grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=2
        )

        ttk.Separator(
            algo_frame,
            orient="horizontal"
        ).grid(
            row=2,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=10
        )

        ttk.Label(
            algo_frame,
            text="k-Multiplier (Threshold):"
        ).grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="w"
        )

        self.lbl_k_val = ttk.Label(
            algo_frame,
            text="3.0"
        )

        self.lbl_k_val.grid(
            row=3,
            column=2,
            sticky="e"
        )

        self.scale_k = ttk.Scale(
            algo_frame,
            from_=0.5,
            to=10.0,
            value=3.0,
            command=self._on_k_slider_move
        )

        self.scale_k.grid(
            row=4,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=2
        )

        ttk.Label(
            algo_frame,
            text="Filter Kernel Size (ksize):"
        ).grid(
            row=5,
            column=0,
            sticky="w",
            pady=5
        )

        self.ksize_cb = ttk.Combobox(
            algo_frame,
            values=["1", "3", "5", "7"],
            width=5,
            state="readonly"
        )

        self.ksize_cb.set("3")

        self.ksize_cb.grid(
            row=5,
            column=1,
            sticky="w",
            padx=5,
            pady=5
        )

        ttk.Label(
            algo_frame,
            text="Local Window (px):"
        ).grid(
            row=6,
            column=0,
            sticky="w",
            pady=5
        )

        self.window_cb = ttk.Combobox(
            algo_frame,
            values=["21", "31", "51", "75", "101"],
            width=5,
            state="readonly"
        )

        self.window_cb.set("51")

        self.window_cb.grid(
            row=6,
            column=1,
            sticky="w",
            padx=5,
            pady=5
        )

        ttk.Label(
            algo_frame,
            text="Flicker Filter:"
        ).grid(
            row=7,
            column=0,
            sticky="w",
            pady=5
        )

        # (persist_frames, persist_hits): a pixel must be flagged in at least
        # persist_hits of the last persist_frames frames to survive. "Off" is
        # the old immediate single-frame behaviour.
        self.FLICKER_PRESETS = {
            "Off": (1, 1),
            "Light (2/3 frames)": (3, 2),
            "Strong (3/5 frames)": (5, 3),
        }

        self.flicker_cb = ttk.Combobox(
            algo_frame,
            values=list(self.FLICKER_PRESETS.keys()),
            width=18,
            state="readonly"
        )

        self.flicker_cb.set("Light (2/3 frames)")

        self.flicker_cb.grid(
            row=7,
            column=1,
            columnspan=2,
            sticky="ew",
            padx=5,
            pady=5
        )

    # =========================================================================
    # LIGHT CONTROLS
    # =========================================================================
    def _build_light_controls(self):

        conn_frame = ttk.LabelFrame(
            self.scroll_frame,
            text=" OPT Light Controller ",
            padding=10
        )

        conn_frame.pack(
            fill="x",
            pady=5
        )

        ttk.Label(
            conn_frame,
            text="Port:"
        ).grid(
            row=0,
            column=0,
            padx=2
        )

        self.port_cb = ttk.Combobox(
            conn_frame,
            values=self.get_ports(),
            width=10
        )

        self.port_cb.grid(
            row=0,
            column=1,
            padx=2
        )

        ttk.Label(
            conn_frame,
            text="Baud:"
        ).grid(
            row=0,
            column=2,
            padx=2
        )

        self.baud_cb = ttk.Combobox(
            conn_frame,
            values=[
                "19200",
                "9600",
                "115200"
            ],
            width=8
        )

        self.baud_cb.set("19200")

        self.baud_cb.grid(
            row=0,
            column=3,
            padx=2
        )

        self.btn_connect = ttk.Button(
            conn_frame,
            text="Connect",
            command=self.toggle_serial_connection
        )

        self.btn_connect.grid(
            row=1,
            column=0,
            columnspan=4,
            sticky="ew",
            pady=5
        )

        # ---------------------------------------------------------------------
        # CHANNEL BRIGHTNESS
        # ---------------------------------------------------------------------
        ctrl_frame = ttk.LabelFrame(
            self.scroll_frame,
            text=" Channel Brightness (0 - 255) ",
            padding=10
        )

        ctrl_frame.pack(
            fill="x",
            pady=5
        )

        self.sliders = []
        self.val_labels = []

        for ch in range(1, 5):

            row_frame = ttk.Frame(
                ctrl_frame
            )

            row_frame.pack(
                fill="x",
                pady=4
            )

            ttk.Label(
                row_frame,
                text=f"Ch {ch}:",
                width=6
            ).pack(
                side="left"
            )

            slider = ttk.Scale(
                row_frame,
                from_=0,
                to=255,
                orient="horizontal",
                command=lambda val, c=ch:
                self.on_slider_move(c, val)
            )

            slider.pack(
                side="left",
                fill="x",
                expand=True,
                padx=5
            )

            self.sliders.append(slider)

            val_label = ttk.Label(
                row_frame,
                text="0",
                width=4
            )

            val_label.pack(
                side="right"
            )

            self.val_labels.append(
                val_label
            )

    # =========================================================================
    # DYNAMIC ALGORITHM PLUGIN ENGINE
    # =========================================================================
    def load_algorithms(self):

        self.algorithms = {
            "Laplacian V1 (Built-in)": LaplacianDetectorV1()
        }

        algo_dir = os.path.join(
            os.path.dirname(__file__),
            "algorithms"
        )

        if not os.path.exists(algo_dir):
            os.makedirs(
                algo_dir,
                exist_ok=True
            )

        for file_path in glob.glob(
            os.path.join(algo_dir, "*.py")
        ):

            filename = os.path.basename(
                file_path
            )

            if filename.startswith("__"):
                continue

            module_name = filename[:-3]

            try:

                spec = (
                    importlib.util
                    .spec_from_file_location(
                        module_name,
                        file_path
                    )
                )

                mod = (
                    importlib.util
                    .module_from_spec(spec)
                )

                spec.loader.exec_module(mod)

                if hasattr(
                    mod,
                    "process_frame"
                ):

                    algo_title = getattr(
                        mod,
                        "NAME",
                        module_name
                    )

                    self.algorithms[
                        algo_title
                    ] = mod.process_frame

            except Exception as e:

                print(
                    f"Failed to load algorithm "
                    f"plugin {filename}: {e}"
                )

    def reload_algorithms_list(self):

        self.load_algorithms()

        self.algo_cb["values"] = list(
            self.algorithms.keys()
        )

        messagebox.showinfo(
            "Algorithm Loader",
            f"Loaded {len(self.algorithms)} algorithm(s)."
        )

    # =========================================================================
    # CAMERA CONTROLS
    # =========================================================================
    def toggle_camera(self):

        if self.cam and self.cam.started:

            self.cam.stop()

            self.cam = None

            self.btn_cam_toggle.config(
                text="Start Camera"
            )

        else:

            selected = self.ent_cam_src.get().strip()

            if selected not in self.camera_map:

                messagebox.showwarning(
                    "Camera",
                    "Please select a valid camera."
                )

                return

            src = self.camera_map[selected]

            try:

                self.cam = CameraStream(
                    src
                ).start()

                self.btn_cam_toggle.config(
                    text="Stop Camera"
                )

            except Exception as e:

                self.cam = None

                messagebox.showerror(
                    "Camera Error",
                    f"Unable to open video source:\n\n{e}"
                )

    def _on_k_slider_move(self, val):

        self.lbl_k_val.config(
            text=f"{float(val):.1f}"
        )

    # =========================================================================
    # SERIAL LIGHT CONTROLLER
    # =========================================================================
    def get_ports(self):

        ports = [
            port.device
            for port in serial.tools.list_ports.comports()
        ]

        return ports if ports else [
            "COM1",
            "COM3",
            "/dev/ttyUSB0"
        ]

    def toggle_serial_connection(self):

        if self.ser and self.ser.is_open:

            self.ser.close()

            self.btn_connect.config(
                text="Connect"
            )

            messagebox.showinfo(
                "Status",
                "Disconnected from device."
            )

        else:

            port = self.port_cb.get()
            baud = self.baud_cb.get()

            if not port:

                messagebox.showwarning(
                    "Error",
                    "Please select a COM port."
                )

                return

            try:

                self.ser = serial.Serial(
                    port,
                    int(baud),
                    timeout=1,
                    # BUG FIX: no write_timeout meant self.ser.write() below
                    # (called straight from the slider callback, on the Tk
                    # main thread) could block forever if the device ever
                    # stopped acknowledging -- freezing the entire GUI,
                    # including the camera preview, with no way to recover
                    # short of killing the process.
                    write_timeout=2
                )

                self.btn_connect.config(
                    text="Disconnect"
                )

                messagebox.showinfo(
                    "Status",
                    f"Connected to {port}"
                )

            except Exception as e:

                messagebox.showerror(
                    "Connection Error",
                    str(e)
                )

    def send_all_channels(self):
        """
        Throttled, non-blocking send of ALL 4 current channel levels
        together (see build_light_frame -- the device cannot be addressed
        one channel at a time, unlike the old send_command()).

        Rapid slider drags call this far more often than the controller's
        minimum command spacing allows. Rather than sending every single
        call (frames can be dropped) or blocking with time.sleep() (freezes
        the UI), at most one send goes out immediately and any calls that
        arrive too soon after it schedule exactly one deferred follow-up
        via root.after(); further rapid calls before that follow-up fires
        just get folded into it, since it always reads self.channel_values
        fresh at send time.
        """

        if not (self.ser and self.ser.is_open):
            return

        now = time.monotonic()
        elapsed_ms = (now - self._last_light_send_time) * 1000.0
        min_gap_ms = 100

        if elapsed_ms < min_gap_ms:
            if self._pending_light_send_id is None:
                delay_ms = int(min_gap_ms - elapsed_ms) + 1
                self._pending_light_send_id = self.root.after(
                    delay_ms, self._do_send_channels
                )
            return

        self._do_send_channels()

    def _do_send_channels(self):

        self._pending_light_send_id = None

        if not (self.ser and self.ser.is_open):
            return

        self._last_light_send_time = time.monotonic()

        frame = build_light_frame(self.channel_values)

        try:

            self.ser.write(frame)

        except Exception as e:

            print(
                f"Write error: {e}"
            )

    def on_slider_move(self, channel, val):

        val_int = int(float(val))

        self.val_labels[
            channel - 1
        ].config(
            text=str(val_int)
        )

        self.channel_values[channel - 1] = val_int

        self.send_all_channels()

    # =========================================================================
    # CANVAS INTERACTIVITY
    # =========================================================================
    def reset_zoom(self):

        self.zoom_level = 1.0
        self.pan_x = 0
        self.pan_y = 0

    def clear_roi(self):

        self.roi = None

    def screen_to_img_coords(
        self,
        sx,
        sy
    ):

        ix = int(
            (sx - self.pan_x)
            / self.zoom_level
        )

        iy = int(
            (sy - self.pan_y)
            / self.zoom_level
        )

        return ix, iy

    def on_zoom(self, event):

        if event.num == 4 or event.delta > 0:
            zoom_factor = 1.1
        else:
            zoom_factor = 0.9

        new_zoom = (
            self.zoom_level
            * zoom_factor
        )

        if (
            new_zoom < 0.2
            or new_zoom > 10.0
        ):
            return

        mx, my = event.x, event.y

        self.pan_x = (
            mx
            - (mx - self.pan_x)
            * zoom_factor
        )

        self.pan_y = (
            my
            - (my - self.pan_y)
            * zoom_factor
        )

        self.zoom_level = new_zoom

    def on_pan_start(self, event):

        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def on_pan_drag(self, event):

        dx = (
            event.x
            - self.last_mouse_x
        )

        dy = (
            event.y
            - self.last_mouse_y
        )

        self.pan_x += dx
        self.pan_y += dy

        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def on_roi_start(self, event):

        self.is_drawing_roi = True

        self.roi_start = (
            event.x,
            event.y
        )

    def on_roi_drag(self, event):

        if self.is_drawing_roi:

            x1, y1 = self.screen_to_img_coords(
                self.roi_start[0],
                self.roi_start[1]
            )

            x2, y2 = self.screen_to_img_coords(
                event.x,
                event.y
            )

            self.roi = (
                min(x1, x2),
                min(y1, y2),
                max(x1, x2),
                max(y1, y2)
            )

    def on_roi_end(self, event):

        self.is_drawing_roi = False

        if self.roi_start:

            x1, y1 = self.screen_to_img_coords(
                self.roi_start[0],
                self.roi_start[1]
            )

            x2, y2 = self.screen_to_img_coords(
                event.x,
                event.y
            )

            if (
                abs(x2 - x1) > 5
                and abs(y2 - y1) > 5
            ):

                self.roi = (
                    min(x1, x2),
                    min(y1, y2),
                    max(x1, x2),
                    max(y1, y2)
                )

            else:

                self.roi = None

    # =========================================================================
    # VIDEO PROCESSING & UI REFRESH LOOP
    # =========================================================================
    def update_loop(self):

        if self.cam and self.cam.started:

            grabbed, frame = self.cam.read()

            if grabbed and frame is not None:

                self.current_raw_frame = frame

        if self.current_raw_frame is None:

            placeholder = np.zeros(
                (480, 640, 3),
                dtype=np.uint8
            )

            cv2.putText(
                placeholder,
                "Camera Stopped / No Feed",
                (140, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            display_frame = placeholder

        else:

            # See _dispatch_detection()/process_frame() for why this no
            # longer calls process_frame() directly here.
            self._dispatch_detection(self.current_raw_frame)

            with self._display_lock:
                display_frame = (
                    self._latest_processed_frame
                    if self._latest_processed_frame is not None
                    else self.current_raw_frame
                )

        self._render_canvas(
            display_frame
        )

        self.root.after(
            30,
            self.update_loop
        )

    def _snapshot_detection_params(self):
        """
        Reads every ttk widget process_frame() needs, ON THE MAIN THREAD.
        ttk widget .get() calls are not safe to make from a background
        thread, so this is called from update_loop() and the plain
        dict/values it returns are what actually get handed to the worker
        thread in _dispatch_detection() -- the worker never touches a
        widget directly.
        """

        selected_algo_name = self.algo_cb.get()

        algo_fn = self.algorithms.get(
            selected_algo_name,
            self.algorithms.get("Laplacian V1 (Built-in)")
        )

        persist_frames, persist_hits = self.FLICKER_PRESETS.get(
            self.flicker_cb.get(), (3, 2)
        )

        params = {
            "k_multiplier": float(self.scale_k.get()),
            "ksize": int(self.ksize_cb.get()),
            "window": int(self.window_cb.get()),
            "persist_frames": persist_frames,
            "persist_hits": persist_hits
        }

        return self.roi, algo_fn, params

    def process_frame(self, frame, roi, algo_fn, params):
        """
        Pure pixel-crunching, safe to run on a background thread: touches
        only the frame/roi/algo_fn/params arguments it was given, never a
        Tk widget. (Renamed nothing -- this is the exact same logic that
        used to live directly in update_loop's call chain; only the widget
        reads were pulled out into _snapshot_detection_params, above.)
        """

        out_frame = frame.copy()

        h, w = out_frame.shape[:2]

        # ---------------------------------------------------------------------
        # ROI PROCESSING
        # ---------------------------------------------------------------------
        if roi:

            x1, y1, x2, y2 = roi

            x1, y1 = max(0, x1), max(0, y1)

            x2, y2 = min(w, x2), min(h, y2)

            if x2 > x1 and y2 > y1:

                roi_crop = out_frame[
                    y1:y2,
                    x1:x2
                ]

                try:

                    processed_roi = algo_fn(
                        roi_crop,
                        **params
                    )

                except TypeError:

                    processed_roi = algo_fn(
                        roi_crop,
                        params
                    )

                out_frame[
                    y1:y2,
                    x1:x2
                ] = processed_roi

                cv2.rectangle(
                    out_frame,
                    (x1, y1),
                    (x2, y2),
                    (255, 255, 0),
                    2
                )

        # ---------------------------------------------------------------------
        # FULL FRAME PROCESSING
        # ---------------------------------------------------------------------
        else:

            try:

                out_frame = algo_fn(
                    out_frame,
                    **params
                )

            except TypeError:

                out_frame = algo_fn(
                    out_frame,
                    params
                )

        return out_frame

    def _dispatch_detection(self, frame):
        """
        Kicks off one background detection pass, if one isn't already
        running. See update_loop() for why this exists: process_frame()
        used to run directly on the Tk main thread inside the 30ms-repeating
        update_loop callback, so a slow algorithm pass directly delayed
        that tick AND pushed the next scheduled tick back too -- this is
        the actual cause of the reported "feed lag". Now it runs here, on
        its own thread, and update_loop always just displays whatever the
        most recently finished result is instead of waiting for a new one.
        """

        if self._detection_busy:
            return

        roi, algo_fn, params = self._snapshot_detection_params()

        self._detection_busy = True

        threading.Thread(
            target=self._detection_worker,
            args=(frame, roi, algo_fn, params),
            daemon=True
        ).start()

    def _detection_worker(self, frame, roi, algo_fn, params):

        try:
            result = self.process_frame(frame, roi, algo_fn, params)
            with self._display_lock:
                self._latest_processed_frame = result
        except Exception as e:
            print(f"Detection worker error: {e}")
        finally:
            self._detection_busy = False

    # =========================================================================
    # CANVAS RENDERING
    # =========================================================================
    def _render_canvas(self, frame):

        h, w = frame.shape[:2]

        new_w = max(
            1,
            int(w * self.zoom_level)
        )

        new_h = max(
            1,
            int(h * self.zoom_level)
        )

        rgb_img = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        resized_img = cv2.resize(
            rgb_img,
            (new_w, new_h),
            interpolation=cv2.INTER_LINEAR
        )

        pil_img = Image.fromarray(
            resized_img
        )

        self.tk_img = ImageTk.PhotoImage(
            image=pil_img
        )

        self.canvas.delete("all")

        self.canvas.create_image(
            self.pan_x,
            self.pan_y,
            anchor="nw",
            image=self.tk_img
        )


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================
if __name__ == "__main__":

    root = tk.Tk()

    app = VisionInspectionApp(
        root
    )

    root.mainloop()