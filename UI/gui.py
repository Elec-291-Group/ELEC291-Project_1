import sys
import time
import csv
import threading
import json
import queue
from collections import deque

import serial
import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ===================== CONFIGURATION =====================
DEFAULT_PORT = "COM7"
BAUD = 57600
LOG_FILENAME = "temperature_log.csv"
PROFILE_FILE = "reflow_profiles.json"

# Graph Settings
X_SIZE = 300

# If DE10 already averages, set FILTER_WINDOW = 1 (no extra smoothing)
FILTER_WINDOW = 1   # 1 disables filtering; 5 gives a small average


DEFAULT_PROFILES = {
    "Lead-Free SAC305": {"soak_temp": 150, "soak_time": 90, "reflow_temp": 245, "reflow_time": 60},
    "Leaded Sn63Pb37":  {"soak_temp": 150, "soak_time": 60, "reflow_temp": 183, "reflow_time": 45},
    "Test":             {"soak_temp": 50,  "soak_time": 10, "reflow_temp": 70,  "reflow_time": 10},
}


# ===================== FORMAT HELPERS (MATCH DE10 BUFFERS) =====================
def clamp_int(v, lo, hi):
    try:
        v = int(v)
    except Exception:
        v = lo
    return max(lo, min(hi, v))


def fmt_temp_3dig(temp_c: int) -> str:
    """
    Buf_*_Temp is 3 digits + null terminator (ds 4).
    Enforce 0..999 and return exactly 3 ASCII digits.
    """
    t = clamp_int(temp_c, 0, 999)
    return f"{t:03d}"


def sec_to_mmss_str(total_sec: int) -> str:
    """
    Buf_*_Time is 4 digits MMSS + null terminator (ds 5).
    Convert seconds to MMSS, clamp to 99:59.
    Return exactly 4 ASCII digits.
    """
    total_sec = clamp_int(total_sec, 0, 99 * 60 + 59)
    m = total_sec // 60
    s = total_sec % 60
    return f"{m:02d}{s:02d}"


# ===================== SERIAL MANAGER =====================
class SerialManager:
    """
    One writer thread + one reader thread.
    - Prevents interleaved writes / bursts that scramble MCU parser/LCD FSM.
    """
    def __init__(self):
        self.ser = None
        self.rx_thread = None
        self.tx_thread = None
        self.stop_event = threading.Event()

        self.rx_queue = queue.Queue()
        self.tx_queue = queue.Queue()

        self.connected = False

        # pacing: CV-8052 UART parsers often need a little spacing
        self.min_tx_gap_s = 0.08

    def list_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(p.device)
        return ports

    def connect(self, port, baud):
        self.disconnect()

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.connected = True
        except serial.SerialException as e:
            self.connected = False
            self.ser = None
            return False, str(e)

        self.stop_event.clear()

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()

        # Flush any boot spam so our first command doesn't get mixed in
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        return True, None

    def disconnect(self):
        self.stop_event.set()
        self.connected = False

        try:
            while not self.tx_queue.empty():
                self.tx_queue.get_nowait()
        except Exception:
            pass

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.ser = None

    def send_line(self, line: str):
        """
        Enqueue a single line to transmit.
        We always send CRLF to be friendly to common embedded parsers.
        """
        if not line.endswith("\n"):
            line = line + "\n"
        self.tx_queue.put(line)

    def send_lines(self, lines):
        for ln in lines:
            self.send_line(ln)

    def _tx_loop(self):
        last_tx = 0.0
        while not self.stop_event.is_set():
            try:
                line = self.tx_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            if not self.connected or not self.ser or not self.ser.is_open:
                continue

            now = time.time()
            dt = now - last_tx
            if dt < self.min_tx_gap_s:
                time.sleep(self.min_tx_gap_s - dt)

            try:
                payload = line.replace("\n", "\r\n").encode(errors="ignore")
                self.ser.write(payload)
                last_tx = time.time()
                print(f"TX -> {line.strip()}")
            except Exception as e:
                self.rx_queue.put(("SERIAL_ERROR", str(e)))
                self.disconnect()
                return

    def _rx_loop(self):
        while not self.stop_event.is_set():
            if not self.connected or not self.ser or not self.ser.is_open:
                time.sleep(0.05)
                continue

            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue

                self.rx_queue.put(("LINE", line))

            except Exception as e:
                self.rx_queue.put(("SERIAL_ERROR", str(e)))
                self.disconnect()
                return


# ===================== GUI APPLICATION =====================
class ReflowGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Reflow Oven Controller")
        self.root.geometry("1150x720")

        self.sm = SerialManager()

        # Data Buffers
        self.xdata = deque(maxlen=X_SIZE)
        self.yraw = deque(maxlen=X_SIZE)
        self.yfilt = deque(maxlen=X_SIZE)

        self.is_running = False
        self.start_time = 0.0

        # MCU state / debug
        self.last_temp = None
        self.last_status = "IDLE"
        self.last_line_time = 0.0

        # Load Profiles
        self.profiles = self.load_profiles()
        self.current_profile_key = list(self.profiles.keys())[0]

        # Tk variables
        self.var_port = tk.StringVar(value=DEFAULT_PORT)
        self.var_baud = tk.IntVar(value=BAUD)

        # manual port entry var
        self.var_port_manual = tk.StringVar(value="")

        self.var_soak_temp = tk.DoubleVar()
        self.var_soak_time = tk.IntVar()
        self.var_reflow_temp = tk.DoubleVar()
        self.var_reflow_time = tk.IntVar()

        # Layout
        style = ttk.Style()
        style.theme_use("clam")

        self.paned = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.left_frame = ttk.Frame(self.paned, width=330)
        self.right_frame = ttk.Frame(self.paned)

        self.paned.add(self.left_frame, weight=1)
        self.paned.add(self.right_frame, weight=4)

        self.build_right_panel()
        self.build_left_panel()

        # Start animation + RX polling
        self.start_animation()
        self.root.after(50, self.poll_serial_rx)

    # ---------- UI BUILD ----------
    def build_left_panel(self):
        # --- CONNECTION ---
        conn = ttk.LabelFrame(self.left_frame, text="Connection", padding=10)
        conn.pack(fill=tk.X, padx=10, pady=(10, 8))

        # Row 1: dropdown selection + refresh + connect
        row1 = ttk.Frame(conn)
        row1.pack(fill=tk.X)

        ttk.Label(row1, text="Port:").pack(side=tk.LEFT)
        self.cmb_ports = ttk.Combobox(row1, textvariable=self.var_port, width=10, values=self.sm.list_ports())
        self.cmb_ports.pack(side=tk.LEFT, padx=(6, 10))

        ttk.Label(row1, text="Baud:").pack(side=tk.LEFT)
        self.ent_baud = ttk.Entry(row1, textvariable=self.var_baud, width=8)
        self.ent_baud.pack(side=tk.LEFT, padx=(6, 10))

        ttk.Button(row1, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT, padx=(0, 6))
        self.btn_connect = ttk.Button(row1, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT)

        # Row 2: manual type-in + Find & Connect
        row2 = ttk.Frame(conn)
        row2.pack(fill=tk.X, pady=(8, 0))

        ttk.Label(row2, text="Type port:").pack(side=tk.LEFT)
        self.ent_manual_port = ttk.Entry(row2, textvariable=self.var_port_manual, width=12)
        self.ent_manual_port.pack(side=tk.LEFT, padx=(6, 10))
        self.ent_manual_port.bind("<Return>", lambda _e: self.find_and_connect())

        ttk.Button(row2, text="Find & Connect", command=self.find_and_connect).pack(side=tk.LEFT)

        self.lbl_conn = ttk.Label(conn, text="Disconnected", foreground="gray")
        self.lbl_conn.pack(anchor="w", pady=(8, 0))

        # --- TITLE ---
        lbl_title = ttk.Label(self.left_frame, text="Reflow Profiles", font=("Arial", 14, "bold"))
        lbl_title.pack(pady=(5, 5), anchor="w", padx=10)

        # --- PROFILE LIST ---
        self.listbox = tk.Listbox(self.left_frame, height=8, font=("Arial", 10), exportselection=False)
        self.listbox.pack(fill=tk.X, padx=10, pady=5)
        self.listbox.bind("<<ListboxSelect>>", self.on_profile_select)
        self.refresh_profile_list()

        # --- PARAMETERS ---
        param_frame = ttk.LabelFrame(self.left_frame, text="Parameters", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=10)

        self.create_input(param_frame, "Soak Temp (°C):", self.var_soak_temp, 0)
        self.create_input(param_frame, "Soak Time (s):", self.var_soak_time, 1)
        self.create_input(param_frame, "Reflow Temp (°C):", self.var_reflow_temp, 2)
        self.create_input(param_frame, "Reflow Time (s):", self.var_reflow_time, 3)

        for var in [self.var_soak_temp, self.var_soak_time, self.var_reflow_temp, self.var_reflow_time]:
            var.trace_add("write", self.update_preview_trace)

        # --- ACTION BUTTONS ---
        btn_frame = ttk.Frame(self.left_frame)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Button(btn_frame, text="Save Profile (PC)", command=self.save_current_profile).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(btn_frame, text="New", command=self.create_new_profile).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )
        ttk.Button(btn_frame, text="Delete", command=self.delete_profile).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=2
        )

        # --- MCU PROFILE PUSH ---
        mcu_frame = ttk.LabelFrame(self.left_frame, text="DE10 / CV-8052", padding=10)
        mcu_frame.pack(fill=tk.X, padx=10, pady=(12, 10))

        ttk.Button(mcu_frame, text="Upload Profile to DE10 (SAVE)", command=self.upload_profile_save).pack(
            fill=tk.X, pady=(0, 6)
        )
        ttk.Button(mcu_frame, text="Upload Profile to DE10 (RAM only)", command=self.upload_profile_ram).pack(
            fill=tk.X, pady=(0, 6)
        )

        # --- OVEN CONTROL ---
        ctrl_frame = ttk.LabelFrame(self.left_frame, text="Oven Control", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=(10, 10))

        self.btn_start = ttk.Button(ctrl_frame, text="START REFLOW", command=self.start_process)
        self.btn_start.pack(fill=tk.X, pady=5)

        self.btn_stop = ttk.Button(ctrl_frame, text="EMERGENCY STOP", command=self.stop_process)
        self.btn_stop.pack(fill=tk.X, pady=5)

        self.btn_clear = ttk.Button(ctrl_frame, text="Clear Graph", command=self.clear_graph)
        self.btn_clear.pack(fill=tk.X, pady=5)

        # --- STATUS ---
        self.lbl_status = ttk.Label(self.left_frame, text="Status: IDLE", font=("Consolas", 12), foreground="gray")
        self.lbl_status.pack(side=tk.BOTTOM, pady=(0, 12))

        self.lbl_temp = ttk.Label(self.left_frame, text="Temp: ---", font=("Consolas", 12))
        self.lbl_temp.pack(side=tk.BOTTOM, pady=(0, 6))

    def build_right_panel(self):
        self.fig, self.ax = plt.subplots(figsize=(6.3, 5.5), dpi=100)
        self.ax.set_title("Temperature Curve")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.grid(True, linestyle="--", alpha=0.6)

        self.line_target, = self.ax.plot([], [], "g--", label="Target")
        self.line_filt, = self.ax.plot([], [], "b-", linewidth=2, label="Displayed")
        self.line_raw, = self.ax.plot([], [], "r-", linewidth=0.7, alpha=0.5, label="Raw")
        self.ax.legend(loc="upper left")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def create_input(self, parent, label, var, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=5)
        ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=1, sticky="e", pady=5)

    # ---------- Profiles ----------
    def load_profiles(self):
        try:
            with open(PROFILE_FILE, "r") as f:
                data = json.load(f)
                profiles = data.get("profiles", DEFAULT_PROFILES)
                if not profiles:
                    return DEFAULT_PROFILES
                return profiles
        except (FileNotFoundError, json.JSONDecodeError):
            return DEFAULT_PROFILES

    def refresh_profile_list(self):
        self.listbox.delete(0, tk.END)
        for name in self.profiles:
            self.listbox.insert(tk.END, name)

        if self.listbox.size() > 0:
            self.listbox.selection_set(0)
            self.load_fields_from_selection()

    def on_profile_select(self, event):
        self.load_fields_from_selection()

    def load_fields_from_selection(self):
        try:
            idx = self.listbox.curselection()[0]
            name = self.listbox.get(idx)
            self.current_profile_key = name
            data = self.profiles[name]

            self.var_soak_temp.set(data.get("soak_temp", 0))
            self.var_soak_time.set(data.get("soak_time", 0))
            self.var_reflow_temp.set(data.get("reflow_temp", 0))
            self.var_reflow_time.set(data.get("reflow_time", 0))

            self.update_preview()
        except IndexError:
            pass

    def update_preview_trace(self, *args):
        self.update_preview()

    def update_preview(self):
        try:
            st = float(self.var_soak_temp.get())
            s_time = int(self.var_soak_time.get())
            rt = float(self.var_reflow_temp.get())
            r_time = int(self.var_reflow_time.get())

            ramp_up_time = int(max(1, (st - 25.0) / 1.5))
            ramp_reflow_time = int(max(1, (rt - st) / 1.5))

            xs = [0]
            ys = [25]

            xs.append(ramp_up_time); ys.append(st)
            time_end_soak = ramp_up_time + s_time
            xs.append(time_end_soak); ys.append(st)

            time_start_reflow = time_end_soak + ramp_reflow_time
            xs.append(time_start_reflow); ys.append(rt)

            time_end_reflow = time_start_reflow + r_time
            xs.append(time_end_reflow); ys.append(rt)

            xs.append(time_end_reflow + 60); ys.append(50)

            self.line_target.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()
        except Exception:
            pass

    def save_current_profile(self):
        name = self.current_profile_key
        try:
            self.profiles[name] = {
                "soak_temp": float(self.var_soak_temp.get()),
                "soak_time": int(self.var_soak_time.get()),
                "reflow_temp": float(self.var_reflow_temp.get()),
                "reflow_time": int(self.var_reflow_time.get()),
            }
            with open(PROFILE_FILE, "w") as f:
                json.dump({"selected": name, "profiles": self.profiles}, f, indent=2)
            messagebox.showinfo("Saved", f"Profile '{name}' saved locally (PC).")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def create_new_profile(self):
        def save_new(name_win, entry):
            new_name = entry.get().strip()
            if new_name:
                self.profiles[new_name] = self.profiles[self.current_profile_key].copy()
                self.refresh_profile_list()
                idx = list(self.profiles.keys()).index(new_name)
                self.listbox.selection_clear(0, tk.END)
                self.listbox.selection_set(idx)
                self.listbox.see(idx)
                self.load_fields_from_selection()
                name_win.destroy()

        top = tk.Toplevel(self.root)
        top.title("New Profile")
        ttk.Label(top, text="Profile Name:").pack(padx=10, pady=5)
        ent = ttk.Entry(top)
        ent.pack(padx=10, pady=5)
        ttk.Button(top, text="Create", command=lambda: save_new(top, ent)).pack(pady=5)

    def delete_profile(self):
        if len(self.profiles) <= 1:
            messagebox.showwarning("Stop", "Cannot delete the last profile.")
            return
        if messagebox.askyesno("Confirm", f"Delete '{self.current_profile_key}'?"):
            del self.profiles[self.current_profile_key]
            self.refresh_profile_list()

    # ---------- Connection ----------
    def refresh_ports(self):
        ports = self.sm.list_ports()
        self.cmb_ports["values"] = ports
        if ports and self.var_port.get() not in ports:
            self.var_port.set(ports[0])

    def toggle_connection(self):
        if self.sm.connected:
            self.sm.disconnect()
            self.btn_connect.config(text="Connect")
            self.lbl_conn.config(text="Disconnected", foreground="gray")
            return

        port = self.var_port.get().strip()
        baud = int(self.var_baud.get())

        ok, err = self.sm.connect(port, baud)
        if not ok:
            messagebox.showerror("Serial", f"Failed to connect:\n{err}")
            return

        self.btn_connect.config(text="Disconnect")
        self.lbl_conn.config(text=f"Connected: {port} @ {baud}", foreground="green")
        self.sm.send_line("UI:REMOTE")

    def _normalize_port_name(self, s: str) -> str:
        s = s.strip()
        if not s:
            return ""
        if s.isdigit():
            return f"COM{s}"
        if s.lower().startswith("com") and s[3:].isdigit():
            return "COM" + s[3:]
        return s

    def find_and_connect(self):
        typed = self._normalize_port_name(self.var_port_manual.get())
        desired = typed if typed else self._normalize_port_name(self.var_port.get())

        available = self.sm.list_ports()
        self.cmb_ports["values"] = available

        if desired not in available:
            messagebox.showerror(
                "Port not found",
                f"'{desired}' is not currently available.\n\nAvailable ports:\n" + "\n".join(available or ["(none)"])
            )
            return

        baud = int(self.var_baud.get())
        ok, err = self.sm.connect(desired, baud)
        if not ok:
            messagebox.showerror("Serial", f"Failed to connect:\n{err}")
            return

        self.var_port.set(desired)
        self.btn_connect.config(text="Disconnect")
        self.lbl_conn.config(text=f"Connected: {desired} @ {baud}", foreground="green")
        self.sm.send_line("UI:REMOTE")

    # ---------- MCU Protocol Helpers ----------
    def get_current_params(self):
        st = int(float(self.var_soak_temp.get()))
        stime = int(self.var_soak_time.get())
        rt = int(float(self.var_reflow_temp.get()))
        rtime = int(float(self.var_reflow_time.get()))
        return st, stime, rt, rtime

    def upload_profile_common(self, save_to_nvm: bool):
        """
        We keep CFG JSON as an optional "future" path,
        but the important part here is S/K/R/L are now buffer-formatted:
          S:TTT   (3 digits)
          K:MMSS  (4 digits)
          R:TTT
          L:MMSS
        """
        if not self.sm.connected:
            messagebox.showwarning("Serial", "Not connected to DE10.")
            return

        try:
            st, stime, rt, rtime = self.get_current_params()
        except Exception:
            messagebox.showerror("Params", "Invalid parameters. Check the fields.")
            return

        # Pre-format exactly how the DE10 buffers want them
        s_temp = fmt_temp_3dig(st)
        r_temp = fmt_temp_3dig(rt)
        s_time = sec_to_mmss_str(stime)
        r_time = sec_to_mmss_str(rtime)

        # Optional JSON (harmless if ignored by firmware)
        cfg = {
            "soak_temp": int(s_temp),
            "soak_time": int(stime),
            "reflow_temp": int(r_temp),
            "reflow_time": int(rtime),
            "save": 1 if save_to_nvm else 0,
            "name": self.current_profile_key,
        }
        cfg_line = "CFG " + json.dumps(cfg, separators=(",", ":"))

        self.sm.send_lines([
    "RUN:0",
    "LCD:HOME",
    "MENU:EXIT",

    # send buffer text first
    f"S:{s_temp}",
    f"K:{s_time}",
    f"R:{r_temp}",
    f"L:{r_time}",

    # now commit/apply on the MCU
    "CFG:APPLY",

    # optional / harmless if ignored
    cfg_line,
])


        if save_to_nvm:
            self.sm.send_line("SAVE:1")

        messagebox.showinfo(
            "Upload",
            "Sent profile to DE10 (buffer-formatted).\n\n"
            f"Soak Temp:  S:{s_temp}\n"
            f"Soak Time:  K:{s_time}  (MMSS)\n"
            f"Reflow Temp:R:{r_temp}\n"
            f"Reflow Time:L:{r_time}  (MMSS)\n\n"
            "Next step: firmware must read UART and copy into Buf_*."
        )

    def upload_profile_save(self):
        self.upload_profile_common(save_to_nvm=True)

    def upload_profile_ram(self):
        self.upload_profile_common(save_to_nvm=False)

    # ---------- Oven Control ----------
    def start_process(self):
        if not self.sm.connected:
            messagebox.showwarning("Serial", "Not connected to DE10.")
            return
        if self.is_running:
            return

        self.upload_profile_common(save_to_nvm=False)
        self.sm.send_line("RUN:1")

        self.is_running = True
        self.start_time = time.time()
        self.clear_graph()
        self.lbl_status.config(text="Status: RUNNING", foreground="green")

    def stop_process(self):
        if self.sm.connected:
            self.sm.send_line("RUN:0")
        self.is_running = False
        self.lbl_status.config(text="Status: STOPPED", foreground="red")

    def clear_graph(self):
        self.xdata.clear()
        self.yraw.clear()
        self.yfilt.clear()
        self.line_filt.set_data([], [])
        self.line_raw.set_data([], [])
        self.canvas.draw_idle()

    # ---------- RX Parsing ----------
    def poll_serial_rx(self):
        try:
            while True:
                kind, payload = self.sm.rx_queue.get_nowait()

                if kind == "SERIAL_ERROR":
                    self.lbl_conn.config(text=f"Serial error: {payload}", foreground="red")
                    self.btn_connect.config(text="Connect")
                    continue

                if kind != "LINE":
                    continue

                line = payload
                self.last_line_time = time.time()

                if line.startswith("TEMP"):
                    try:
                        s = line.replace("TEMP", "").replace(":", " ").strip()
                        val = float(s.split()[0])
                        self.last_temp = val
                        self.lbl_temp.config(text=f"Temp: {val:.2f} °C")
                        self._push_temp(val)
                    except Exception:
                        pass

                elif line.startswith("STATUS"):
                    self.last_status = line
                    self.lbl_status.config(text=f"Status: {line.replace('STATUS', '').strip()}", foreground="blue")

                else:
                    print(f"RX <- {line}")

        except queue.Empty:
            pass

        self.root.after(50, self.poll_serial_rx)

    def _push_temp(self, val: float):
        if self.is_running:
            t = time.time() - self.start_time
        else:
            t = (self.xdata[-1] + 0.1) if len(self.xdata) else 0.0

        self.xdata.append(t)
        self.yraw.append(val)

        if FILTER_WINDOW <= 1:
            self.yfilt.append(val)
        else:
            n = min(FILTER_WINDOW, len(self.yraw))
            window = list(self.yraw)[-n:]
            self.yfilt.append(sum(window) / float(n))

    # ---------- Animation ----------
    def start_animation(self):
        def animate(_i):
            self.line_raw.set_data(self.xdata, self.yraw)
            self.line_filt.set_data(self.xdata, self.yfilt)

            if len(self.xdata) > 0:
                self.ax.set_xlim(0, max(self.xdata[-1] + 10, 60))
                ymax = max(max(self.yraw, default=0.0) + 20.0, 260.0)
                self.ax.set_ylim(0, ymax)

            return self.line_raw, self.line_filt

        self.anim = animation.FuncAnimation(self.fig, animate, interval=100, blit=False)


# ===================== MAIN =====================
if __name__ == "__main__":
    root = tk.Tk()
    app = ReflowGUI(root)

    def on_close():
        try:
            app.sm.disconnect()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
