import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class LightControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("OPT ALT-E4RS-12V Light Controller")
        self.root.geometry("450x420")
        
        self.ser = None

        # --- Connection Setup ---
        conn_frame = ttk.LabelFrame(root, text=" Serial Connection ", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_cb = ttk.Combobox(conn_frame, values=self.get_ports(), width=15)
        self.port_cb.grid(row=0, column=1, padx=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=5)
        self.baud_cb = ttk.Combobox(conn_frame, values=["19200", "9600", "115200"], width=8)
        self.baud_cb.set("19200")  # Standard default for OPT controllers
        self.baud_cb.grid(row=0, column=3, padx=5)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=4, padx=5)

        # --- Channel Controls (1 to 4) ---
        ctrl_frame = ttk.LabelFrame(root, text=" Channel Brightness (0 - 255) ", padding=10)
        ctrl_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.sliders = []
        self.val_labels = []

        for ch in range(1, 5):
            row_frame = ttk.Frame(ctrl_frame)
            row_frame.pack(fill="x", pady=8)

            ttk.Label(row_frame, text=f"Channel {ch}:", width=10).pack(side="left")

            slider = ttk.Scale(
                row_frame, 
                from_=0, 
                to=255, 
                orient="horizontal", 
                command=lambda val, c=ch: self.on_slider_move(c, val)
            )
            slider.pack(side="left", fill="x", expand=True, padx=10)
            self.sliders.append(slider)

            val_label = ttk.Label(row_frame, text="0", width=5)
            val_label.pack(side="right")
            self.val_labels.append(val_label)

    def get_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["COM1", "COM3", "/dev/ttyUSB0"]

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.btn_connect.config(text="Connect")
            messagebox.showinfo("Status", "Disconnected from device.")
        else:
            port = self.port_cb.get()
            baud = self.baud_cb.get()
            if not port:
                messagebox.showwarning("Error", "Please select a COM port.")
                return
            try:
                self.ser = serial.Serial(port, int(baud), timeout=1)
                self.btn_connect.config(text="Disconnect")
                messagebox.showinfo("Status", f"Connected to {port}")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def send_command(self, channel, val):
        if self.ser and self.ser.is_open:
            # OPT standard command syntax: $3<Channel><Value_in_3_digits>
            # Example: Channel 1 set to 255 -> "$31255\r\n"
            cmd = f"$3{channel}{val:03d}\r\n"
            try:
                self.ser.write(cmd.encode('ascii'))
            except Exception as e:
                print(f"Write error: {e}")

    def on_slider_move(self, channel, val):
        val_int = int(float(val))
        self.val_labels[channel - 1].config(text=str(val_int))
        self.send_command(channel, val_int)

if __name__ == "__main__":
    root = tk.Tk()
    app = LightControllerGUI(root)
    root.mainloop()
