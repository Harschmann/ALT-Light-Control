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

                    result.Release()

                except Exception as e:
                    print(f"Basler grab error: {e}")

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


# =============================================================================
# 2. BUILT-IN DETECTOR LOGIC (Laplacian V1)
# =============================================================================
def detect_laplacian_v1(img, k_multiplier=3.0, ksize=3):

    """
    Laplacian V1 detection logic.
    Accepts BGR image input, processes gray/laplacian, returns output BGR image.
    """

    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    # Apply Laplacian Filter
    laplacian = cv2.Laplacian(
        gray,
        cv2.CV_64F,
        ksize=ksize
    )

    laplacian_abs = cv2.convertScaleAbs(laplacian)

    # Dynamic Thresholding
    mean_val, std_dev = cv2.meanStdDev(laplacian_abs)

    threshold_value = (
        mean_val[0][0]
        + (k_multiplier * std_dev[0][0])
    )

    _, binary_mask = cv2.threshold(
        laplacian_abs,
        threshold_value,
        255,
        cv2.THRESH_BINARY
    )

    # Find contours
    contours, _ = cv2.findContours(
        binary_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    result_img = img.copy()

    for contour in contours:

        x, y, w, h = cv2.boundingRect(contour)

        if w > 1 or h > 1:

            cv2.rectangle(
                result_img,
                (x, y),
                (x + w, y + h),
                (0, 0, 255),
                2
            )

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

        self._build_layout()

        self.update_loop()

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
            text="↻",
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
            "Laplacian V1 (Built-in)": detect_laplacian_v1
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
                    timeout=1
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

    def send_command(self, channel, val):

        if self.ser and self.ser.is_open:

            cmd = (
                f"$3{channel}{val:03d}\r\n"
            )

            try:

                self.ser.write(
                    cmd.encode("ascii")
                )

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

        self.send_command(
            channel,
            val_int
        )

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

            display_frame = self.process_frame(
                self.current_raw_frame
            )

        self._render_canvas(
            display_frame
        )

        self.root.after(
            30,
            self.update_loop
        )

    def process_frame(self, frame):

        out_frame = frame.copy()

        h, w = out_frame.shape[:2]

        selected_algo_name = (
            self.algo_cb.get()
        )

        algo_fn = self.algorithms.get(
            selected_algo_name,
            detect_laplacian_v1
        )

        k_mult = float(
            self.scale_k.get()
        )

        ksize = int(
            self.ksize_cb.get()
        )

        params = {
            "k_multiplier": k_mult,
            "ksize": ksize
        }

        # ---------------------------------------------------------------------
        # ROI PROCESSING
        # ---------------------------------------------------------------------
        if self.roi:

            x1, y1, x2, y2 = self.roi

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
