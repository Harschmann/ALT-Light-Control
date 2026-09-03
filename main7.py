import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class AltLightControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ALT Light Controller (4/8 Channels)")
        self.root.geometry("480x520")
        self.root.minsize(450, 400)
        
        self.ser = None
        self.num_channels = tk.IntVar(value=4) # Default 4 channels
        self.channel_values = [0] * 16 # Store max possible channel values
        
        self.setup_ui()

    def setup_ui(self):
        # --- Connection Setup ---
        conn_frame = ttk.LabelFrame(self.root, text=" Serial Connection ", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=2)
        self.port_cb = ttk.Combobox(conn_frame, values=self.get_ports(), width=10)
        self.port_cb.grid(row=0, column=1, padx=2)
        if self.port_cb['values']:
            self.port_cb.current(0)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=2)
        self.baud_cb = ttk.Combobox(conn_frame, values=["9600", "19200", "38400", "115200"], width=8)
        self.baud_cb.set("19200") # ALT default is usually 19200
        self.baud_cb.grid(row=0, column=3, padx=2)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=4, padx=5)

        # --- Mode Selection ---
        mode_frame = ttk.Frame(self.root)
        mode_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(mode_frame, text="Select Mode:", font=("Arial", 9, "bold")).pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="4 Channels", variable=self.num_channels, value=4, command=self.build_sliders).pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="8 Channels", variable=self.num_channels, value=8, command=self.build_sliders).pack(side="left", padx=5)

        # --- Channel Controls ---
        self.ctrl_frame = ttk.LabelFrame(self.root, text=" Channel Brightness (0 - 255) ", padding=10)
        self.ctrl_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.slider_widgets = []
        self.val_labels = []
        self.build_sliders()

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
                self.send_all_channels() # Send initial zeroes on connect
            except Exception as e:
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
        except Exception as e:
            print(f"Write error: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    # Adding a cleaner visual theme
    if 'clam' in style.theme_names():
        style.theme_use('clam')
    app = AltLightControllerGUI(root)
    root.mainloop()
