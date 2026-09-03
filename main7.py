import tkinter as tk
from tkinter import ttk, messagebox
import time
import threading
import queue
import socket
import serial
import serial.tools.list_ports


class TcpTransport:
    """
    Thin wrapper that mimics a pyserial Serial object's interface
    (is_open / write / read / close) over a plain TCP socket, so all the
    existing send/receive/monitor code (which only ever calls
    self.ser.write/.read/.is_open/.close) works unchanged whether the
    connection is RS-232 or the controller's Ethernet interface.
    """
    def __init__(self, ip, port, timeout=0.2):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)  # connect timeout
        self.sock.connect((ip, port))
        self.sock.settimeout(timeout)  # read timeout once connected
        self.is_open = True

    def write(self, data):
        self.sock.sendall(data)

    def read(self, n):
        try:
            data = self.sock.recv(n)
            return data if data else b""
        except socket.timeout:
            return b""
        except OSError:
            return b""

    def close(self):
        self.is_open = False
        try:
            self.sock.close()
        except Exception:
            pass


class UdpTransport:
    """
    Same interface as TcpTransport, but over UDP. ALTSYSTEM's spec mentions
    the Ethernet interface supporting BOTH TCP/IP and UDP -- a successful TCP
    connect only proves that port accepts TCP connections, not that it's the
    actual light-control command channel. This lets that be tested too,
    without any code, in case the real command path is UDP datagrams.
    """
    def __init__(self, ip, port, timeout=0.2):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.is_open = True

    def write(self, data):
        self.sock.sendto(data, (self.ip, self.port))

    def read(self, n):
        try:
            data, _ = self.sock.recvfrom(n)
            return data if data else b""
        except socket.timeout:
            return b""
        except OSError:
            return b""

    def close(self):
        self.is_open = False
        try:
            self.sock.close()
        except Exception:
            pass



class AltLightControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ALT Light Controller (4/8 Channels)")
        self.root.geometry("480x520")
        self.root.minsize(450, 400)
        
        self.ser = None
        self.num_channels = tk.IntVar(value=4) # Default 4 channels
        self.channel_values = [0] * 16 # Store max possible channel values

        # The real ALT-side implementation enforces a 100ms minimum gap between
        # commands (Thread.Sleep in its TurnOn()) -- rapid slider drags were
        # firing far faster than that, which the controller can drop/ignore.
        # These track that throttling on the Python side (non-blocking, so the
        # UI doesn't freeze the way a literal time.sleep() in the callback would).
        self._last_send_time = 0.0
        self._pending_after_id = None

        self.rx_queue = queue.Queue()
        self.reader_thread = None
        self.reader_running = False

        self.setup_ui()

    def setup_ui(self):
        # --- Connection Setup ---
        conn_frame = ttk.LabelFrame(self.root, text=" Connection ", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        type_row = ttk.Frame(conn_frame)
        type_row.pack(fill="x", pady=(0, 5))
        ttk.Label(type_row, text="Type:").pack(side="left")
        self.conn_type = tk.StringVar(value="Ethernet (TCP)")
        self.conn_type_cb = ttk.Combobox(
            type_row, textvariable=self.conn_type,
            values=["Ethernet (TCP)", "Ethernet (UDP)", "Serial (RS-232)"], state="readonly", width=16
        )
        self.conn_type_cb.pack(side="left", padx=5)
        self.conn_type_cb.bind("<<ComboboxSelected>>", lambda e: self.on_conn_type_change())

        # Serial fields
        self.serial_frame = ttk.Frame(conn_frame)
        ttk.Label(self.serial_frame, text="Port:").grid(row=0, column=0, padx=2)
        self.port_cb = ttk.Combobox(self.serial_frame, values=self.get_ports(), width=10)
        self.port_cb.grid(row=0, column=1, padx=2)
        if self.port_cb['values']:
            self.port_cb.current(0)
        ttk.Label(self.serial_frame, text="Baud:").grid(row=0, column=2, padx=2)
        self.baud_cb = ttk.Combobox(self.serial_frame, values=["9600", "19200"], width=8, state="readonly")
        self.baud_cb.set("19200") # ALT default is usually 19200
        self.baud_cb.grid(row=0, column=3, padx=2)

        # Ethernet fields
        self.tcp_frame = ttk.Frame(conn_frame)
        ttk.Label(self.tcp_frame, text="IP:").grid(row=0, column=0, padx=2)
        self.ip_entry = ttk.Entry(self.tcp_frame, width=15)
        self.ip_entry.insert(0, "192.168.10.10")
        self.ip_entry.grid(row=0, column=1, padx=2)
        ttk.Label(self.tcp_frame, text="Port:").grid(row=0, column=2, padx=2)
        self.tcp_port_entry = ttk.Entry(self.tcp_frame, width=8)
        self.tcp_port_entry.insert(0, "1000")
        self.tcp_port_entry.grid(row=0, column=3, padx=2)

        self.on_conn_type_change()

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(fill="x", pady=5)

        # --- Mode Selection ---
        mode_frame = ttk.Frame(self.root)
        mode_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(mode_frame, text="Select Mode:", font=("Arial", 9, "bold")).pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="4 Channels", variable=self.num_channels, value=4, command=self.build_sliders).pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="8 Channels", variable=self.num_channels, value=8, command=self.build_sliders).pack(side="left", padx=5)

        ttk.Label(
            self.root,
            text="Your unit's label reads ALT-E4RS -- \"E4\" is a fixed 4-channel model. "
                 "8-Channel mode sends a longer packet that a 4-channel unit isn't built "
                 "to parse -- only use it if you actually have the 8-channel (E8) unit.",
            foreground="#b45309", font=("Arial", 8), wraplength=440, justify="left"
        ).pack(fill="x", padx=10, pady=(0, 5))

        # --- Channel Controls ---
        self.ctrl_frame = ttk.LabelFrame(self.root, text=" Channel Brightness (0 - 255) ", padding=10)
        self.ctrl_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.slider_widgets = []
        self.val_labels = []
        self.build_sliders()

        # --- Byte Monitor (see exactly what's really going out / coming back) ---
        mon_frame = ttk.LabelFrame(self.root, text=" Byte Monitor ", padding=8)
        mon_frame.pack(fill="both", padx=10, pady=(0, 5))

        raw_row = ttk.Frame(mon_frame)
        raw_row.pack(fill="x", pady=(0, 4))
        ttk.Label(raw_row, text="Send Raw Hex:").pack(side="left")
        self.ent_raw_hex = ttk.Entry(raw_row)
        self.ent_raw_hex.pack(side="left", fill="x", expand=True, padx=5)
        self.ent_raw_hex.insert(0, "EF EF 00 FF FF FF FF FF EE EE")
        ttk.Button(raw_row, text="Send", command=self.send_raw_hex).pack(side="left")

        self.txt_monitor = tk.Text(mon_frame, height=6, font=("Consolas", 8), wrap="none")
        self.txt_monitor.pack(fill="both", expand=True)
        self.txt_monitor.configure(state="disabled")

        ttk.Button(mon_frame, text="Clear", command=self.clear_monitor).pack(anchor="e", pady=(4, 0))

    def clear_monitor(self):
        self.txt_monitor.configure(state="normal")
        self.txt_monitor.delete("1.0", "end")
        self.txt_monitor.configure(state="disabled")

    def send_raw_hex(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Not Connected", "Connect to the port first.")
            return
        text = self.ent_raw_hex.get().strip()
        try:
            hex_bytes = bytes(int(b, 16) for b in text.split())
        except ValueError:
            messagebox.showerror("Bad Input", "Enter space-separated hex bytes, e.g.\nEF EF 00 FF FF FF FF FF EE EE")
            return
        try:
            self.ser.write(hex_bytes)
            self.log_monitor(hex_bytes, "TX")
        except Exception as e:
            messagebox.showerror("Write Error", str(e))

    def log_monitor(self, raw_bytes, tag):
        ts = time.strftime("%H:%M:%S")
        hex_part = " ".join(f"{b:02X}" for b in raw_bytes)
        line = f"[{ts}] {tag:<3} {hex_part}\n"
        self.txt_monitor.configure(state="normal")
        self.txt_monitor.insert("end", line)
        num_lines = int(self.txt_monitor.index("end-1c").split(".")[0])
        if num_lines > 300:
            self.txt_monitor.delete("1.0", f"{num_lines - 300}.0")
        self.txt_monitor.see("end")
        self.txt_monitor.configure(state="disabled")

    def _start_reader(self):
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self._poll_rx()

    def _stop_reader(self):
        self.reader_running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
            self.reader_thread = None

    def _reader_loop(self):
        while self.reader_running:
            try:
                if self.ser and self.ser.is_open:
                    data = self.ser.read(64)
                    if data:
                        self.rx_queue.put(data)
                else:
                    time.sleep(0.05)
            except Exception as e:
                self.rx_queue.put(("__ERROR__", str(e)))
                time.sleep(0.2)

    def _poll_rx(self):
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, tuple) and item[0] == "__ERROR__":
                    print(f"Read error: {item[1]}")
                else:
                    self.log_monitor(item, "RX")
        except queue.Empty:
            pass
        if self.reader_running:
            self.root.after(50, self._poll_rx)

    def get_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["COM1", "COM3", "/dev/ttyUSB0"]

    def on_conn_type_change(self):
        if self.conn_type.get().startswith("Serial"):
            self.tcp_frame.pack_forget()
            self.serial_frame.pack(fill="x")
        else:
            self.serial_frame.pack_forget()
            self.tcp_frame.pack(fill="x")

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            if self._pending_after_id is not None:
                self.root.after_cancel(self._pending_after_id)
                self._pending_after_id = None
            self._stop_reader()
            self.ser.close()
            self.btn_connect.config(text="Connect")
            messagebox.showinfo("Status", "Disconnected from device.")
        else:
            try:
                if self.conn_type.get() == "Ethernet (TCP)":
                    ip = self.ip_entry.get().strip()
                    tcp_port = int(self.tcp_port_entry.get().strip())
                    if not ip:
                        messagebox.showwarning("Error", "Please enter the controller's IP address.")
                        return
                    self.ser = TcpTransport(ip, tcp_port, timeout=0.2)
                elif self.conn_type.get() == "Ethernet (UDP)":
                    ip = self.ip_entry.get().strip()
                    udp_port = int(self.tcp_port_entry.get().strip())
                    if not ip:
                        messagebox.showwarning("Error", "Please enter the controller's IP address.")
                        return
                    self.ser = UdpTransport(ip, udp_port, timeout=0.2)
                else:
                    port = self.port_cb.get()
                    baud = self.baud_cb.get()
                    if not port:
                        messagebox.showwarning("Error", "Please select a COM port.")
                        return
                    self.ser = serial.Serial(port, int(baud), timeout=0.2)

                time.sleep(0.15)  # let the controller settle after the connection opens
                self._start_reader()
                self.btn_connect.config(text="Disconnect")
                self.send_all_channels() # Send initial zeroes on connect
            except Exception as e:
                self.ser = None
                messagebox.showerror("Connection Error", str(e))

    def build_sliders(self):
        # Clear existing widgets
        for widget in self.ctrl_frame.winfo_children():
            widget.destroy()
            
        self.slider_widgets = []
        self.val_labels = []
        
        count = self.num_channels.get()
        
        for ch in range(count):
            row_frame = ttk.Frame(self.ctrl_frame)
            row_frame.pack(fill="x", pady=4)

            ttk.Label(row_frame, text=f"Channel {ch+1}:", width=10, font=("Arial", 9)).pack(side="left")

            current_val = self.channel_values[ch]
            slider = ttk.Scale(
                row_frame, 
                from_=0, 
                to=255, 
                orient="horizontal", 
                value=current_val,
                command=lambda val, c=ch: self.on_slider_move(c, val)
            )
            slider.pack(side="left", fill="x", expand=True, padx=10)
            self.slider_widgets.append(slider)

            val_label = ttk.Label(row_frame, text=str(current_val), width=4, font=("Arial", 10, "bold"))
            val_label.pack(side="right")
            self.val_labels.append(val_label)

        # Update root geometry based on channel count to keep it looking clean
        if count == 4:
            self.root.geometry("480x320")
        else:
            self.root.geometry("480x480")

    def on_slider_move(self, channel_index, val):
        val_int = int(float(val))
        self.val_labels[channel_index].config(text=str(val_int))
        self.channel_values[channel_index] = val_int
        self.send_all_channels()

    def send_all_channels(self):
        if not (self.ser and self.ser.is_open):
            return

        now = time.monotonic()
        elapsed_ms = (now - self._last_send_time) * 1000.0

        if elapsed_ms < 100:
            # Too soon since the last real send. Rather than sending anyway
            # (which the real 100ms-gap requirement says the controller may
            # drop) or blocking the UI with time.sleep(), schedule exactly one
            # follow-up send for whenever the gap will be satisfied. Any
            # further slider moves before then just get folded into that one
            # pending send (it always reads self.channel_values fresh when it
            # fires), so a fast drag ends with one final, accurate update
            # instead of a queue of stale ones.
            if self._pending_after_id is None:
                delay_ms = int(100 - elapsed_ms) + 1
                self._pending_after_id = self.root.after(delay_ms, self._do_send)
            return

        self._do_send()

    def _do_send(self):
        self._pending_after_id = None

        if not (self.ser and self.ser.is_open):
            return

        self._last_send_time = time.monotonic()

        count = self.num_channels.get()
        current_vals = self.channel_values[:count]
        
        # Creating Binary Packet matching the C# protocol
        msg = bytearray()
        msg.append(0xEF) # Header
        msg.append(0xEF) # Header
        msg.append(0x00) # msg[2]
        
        for val in current_vals:
            msg.append(val)
            
        # Checksum calculation: msg[2] ^ msg[3] ... ^ (last_channel + 1)
        checksum = 0x00
        for val in current_vals[:-1]:
            checksum ^= val
            
        # Add 1 to the very last value, ensure it stays within 1 byte (0xFF)
        last_val_plus_one = (current_vals[-1] + 1) & 0xFF
        checksum ^= last_val_plus_one
        
        msg.append(checksum)
        msg.append(0xEE) # Footer
        msg.append(0xEE) # Footer
        
        try:
            self.ser.write(msg)
            self.log_monitor(msg, "TX")
        except Exception as e:
            print(f"Write error: {e}")
            self.log_monitor(f"WRITE ERROR: {e}".encode(), "TX")

if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    # Adding a cleaner visual theme
    if 'clam' in style.theme_names():
        style.theme_use('clam')
    app = AltLightControllerGUI(root)
    root.mainloop()
