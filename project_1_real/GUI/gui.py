import sys, time, csv, threading, json
from collections import deque
import queue

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, RadioButtons, TextBox
import serial

# Popup editor (normal cursor, multiline)
import tkinter as tk
from tkinter import messagebox

# ===================== SERIAL =====================
PORT = "COM8"
BAUD = 115200

# ===================== PLOT / LOG =====================
xsize = 200
PLOT_INTERVAL_MS = 50
LOG_FILENAME = "temperature_log.csv"

# ===================== FILTER / ALERT =====================
SMOOTH_WINDOW = 30
ALERT_HIGH_C = 260.0
ALERT_LOW_C  = 0.0

AUTOSCALE_EVERY_N_FRAMES = 10
FLASH_DURATION = 0.30
TEMP_OFFSET_C = 0.0

# ===================== REFLOW PROFILES =====================
PROFILE_FILE = "reflow_profiles.json"

DEFAULT_PROFILES = {
    "Leaded Sn63Pb37": [
        (0,   25),
        (60,  120),
        (180, 150),
        (240, 183),
        (270, 183),
        (360,  50),
    ],
    "Lead-Free SAC305": [
        (0,   25),
        (90,  150),
        (210, 180),
        (270, 245),
        (300, 245),
        (420,  60),
    ],
    "Quick Test Ramp": [
        (0,   25),
        (60,  120),
        (120, 200),
        (180,  25),
    ],
}

PROFILES = dict(DEFAULT_PROFILES)
selected_profile_name = "Lead-Free SAC305"

# How often to send setpoint updates to MCU
SETPOINT_SEND_MIN_DT = 0.25
SETPOINT_SEND_DEADBAND = 0.5

# ===================== LAYOUT =====================
LEFT_PANEL_W = 0.27
BOTTOM_BAR_H = 0.14
PAD = 0.02

# ===================== THREADING / STATE =====================
event_q = queue.Queue(maxsize=2000)
stop_event = threading.Event()
csv_lock = threading.Lock()

ser = None
csv_file = None
csv_writer = None

xdata = deque(maxlen=xsize)
tdata = deque(maxlen=xsize)
yraw  = deque(maxlen=xsize)
yfilt = deque(maxlen=xsize)

xtarget = deque(maxlen=xsize)
ytarget = deque(maxlen=xsize)
target_temp = None

def make_smooth_buf():
    return deque(maxlen=max(1, int(SMOOTH_WINDOW)))
smooth_buf = make_smooth_buf()

tmin = None
tmax = None
last_alert_state = None
frame_count = 0

base_sample = None
paused = False

profile_running = False
profile_t0 = None
last_sp_sent = None
last_sp_sent_t = 0.0

# ===================== UI refs =====================
fig = None
ax = None
line_filt = None
line_raw = None
line_target = None
status_text = None

profile_ax = None
profile_radio = None
profile_names = None
name_box = None

preview_ax = None
preview_text = None

pause_button = None


# ===================== CONFIG IO =====================
def load_profiles():
    global PROFILES, selected_profile_name
    try:
        with open(PROFILE_FILE, "r") as f:
            data = json.load(f)
        profs = data.get("profiles", {})
        if isinstance(profs, dict) and len(profs) > 0:
            PROFILES = {k: [tuple(pt) for pt in v] for k, v in profs.items()}
        sel = data.get("selected", None)
        if sel in PROFILES:
            selected_profile_name = sel
        else:
            selected_profile_name = list(PROFILES.keys())[0]
        print("[CFG] Loaded profiles from", PROFILE_FILE)
    except FileNotFoundError:
        print("[CFG] No profile file yet, using defaults.")
    except Exception as e:
        print("[CFG] Failed to load profiles:", e)

def save_profiles():
    try:
        with open(PROFILE_FILE, "w") as f:
            json.dump({"selected": selected_profile_name, "profiles": PROFILES}, f, indent=2)
        print("[CFG] Saved profiles to", PROFILE_FILE)
    except Exception as e:
        print("[CFG] Failed to save profiles:", e)


# ===================== UTIL =====================
def open_serial():
    return serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)

def send_line(msg: str):
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.write((msg.strip() + "\n").encode())
    except Exception as e:
        print("[SER] write failed:", e)

def c_to_cx100(temp_c: float) -> int:
    return int(round(float(temp_c) * 100.0))

def sec_to_ms(t_sec: float) -> int:
    return int(round(float(t_sec) * 1000.0))

def send_profile_points(points):
    """
    Send all profile points to MCU as:
      PROF:BEGIN
      P:<t_ms>,<temp_cx100>
      ...
      PROF:END
    """
    send_line("PROF:BEGIN")
    time.sleep(0.05)

    for t_sec, temp_c in points:
        t_ms = sec_to_ms(t_sec)
        temp_cx100 = c_to_cx100(temp_c)
        send_line(f"P:{t_ms},{temp_cx100}")
        time.sleep(0.01)  # pacing

    send_line("PROF:END")

def open_csv(clear=False):
    global csv_file, csv_writer
    mode = "w" if clear else "a"
    csv_file = open(LOG_FILENAME, mode, newline="")
    csv_writer = csv.writer(csv_file)
    if clear or csv_file.tell() == 0:
        csv_writer.writerow([
            "unix_time",
            "sample",
            "temp_raw_C",
            "temp_filt_C",
            "target_C",
            "run_state",
            "profile"
        ])
        csv_file.flush()

def close_csv():
    global csv_file, csv_writer
    try:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
    except Exception:
        pass
    csv_file = None
    csv_writer = None

def close_resources():
    global ser
    try:
        if ser and ser.is_open:
            ser.close()
    except Exception:
        pass
    with csv_lock:
        close_csv()

def parse_line(s):
    s = s.strip()
    if not s:
        return None
    try:
        return ("TEMP", float(s))
    except ValueError:
        return None

def update_alert(temp_c):
    global last_alert_state
    if temp_c >= ALERT_HIGH_C:
        state = "HIGH"
    elif temp_c <= ALERT_LOW_C:
        state = "LOW"
    else:
        state = "OK"

    if state != last_alert_state:
        last_alert_state = state
        if state == "HIGH":
            print(f"[ALERT] HIGH temperature: {temp_c:.2f} °C")
        elif state == "LOW":
            print(f"[ALERT] LOW temperature: {temp_c:.2f} °C")
        else:
            print(f"[OK] Temperature back in range: {temp_c:.2f} °C")

def profile_target_at(t_sec: float, pts):
    if t_sec <= pts[0][0]:
        return float(pts[0][1])
    if t_sec >= pts[-1][0]:
        return float(pts[-1][1])
    for (t0, y0), (t1, y1) in zip(pts[:-1], pts[1:]):
        if t0 <= t_sec <= t1:
            if t1 == t0:
                return float(y1)
            a = (t_sec - t0) / (t1 - t0)
            return float(y0 + a * (y1 - y0))
    return float(pts[-1][1])

def current_profile_points():
    return PROFILES[selected_profile_name]

def build_profile_preview(points, n=250):
    t_end = points[-1][0]
    if t_end <= 0:
        return [], []
    ts = np.linspace(0, t_end, n)
    ys = [profile_target_at(t, points) for t in ts]
    xs = list(range(len(ts)))
    return xs, ys

def profile_points_to_text(points):
    return "\n".join([f"{int(t)},{float(temp):.1f}" for t, temp in points])

def parse_profile_text(text):
    pts = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        line = line.replace(";", ",")
        if "," in line:
            a, b = line.split(",", 1)
        else:
            parts = line.split()
            if len(parts) < 2:
                raise ValueError(f"Bad line: '{raw}'")
            a, b = parts[0], parts[1]
        t = int(float(a.strip()))
        temp = float(b.strip())
        pts.append((t, temp))

    if len(pts) < 2:
        raise ValueError("Need at least 2 points.")

    pts.sort(key=lambda x: x[0])
    for i in range(1, len(pts)):
        if pts[i][0] <= pts[i-1][0]:
            raise ValueError("Times must be strictly increasing.")
    return pts

def drain_queue():
    while True:
        try:
            event_q.get_nowait()
        except queue.Empty:
            break


# ===================== UI helpers =====================
def refresh_preview_text():
    if preview_text is None:
        return
    preview_text.set_text(profile_points_to_text(current_profile_points()))
    fig.canvas.draw_idle()

def preview_selected_profile():
    xs, ys = build_profile_preview(current_profile_points(), n=250)
    line_target.set_data(xs, ys)
    if xs:
        ax.set_xlim(0, max(xs))
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()

def set_selected_profile(name):
    global selected_profile_name
    selected_profile_name = name
    if name_box is not None:
        name_box.set_val(selected_profile_name)
    refresh_preview_text()
    preview_selected_profile()

def refresh_profile_radio():
    global profile_radio, profile_names
    profile_ax.cla()
    profile_ax.set_title("Profiles", fontsize=10)

    profile_names = list(PROFILES.keys())
    if selected_profile_name not in PROFILES:
        globals()["selected_profile_name"] = profile_names[0]

    active_idx = profile_names.index(selected_profile_name)
    profile_radio = RadioButtons(profile_ax, profile_names, active=active_idx)
    for txt in profile_radio.labels:
        txt.set_fontsize(9)
    profile_radio.on_clicked(on_profile_selected)
    fig.canvas.draw_idle()


# ===================== Tk editor popup =====================
def open_profile_editor_popup(_event=None):
    if profile_running:
        print("[UI] Can't edit while running. Stop first.")
        return

    root = tk.Tk()
    root.withdraw()

    win = tk.Toplevel(root)
    win.title(f"Edit Profile Points — {selected_profile_name}")
    win.geometry("520x420")

    lab = tk.Label(
        win,
        text="One point per line:  time_seconds,temperature_C   (example: 60,150)\n"
             "Times must be strictly increasing."
    )
    lab.pack(padx=10, pady=(10, 4), anchor="w")

    text = tk.Text(win, wrap="none", font=("Consolas", 10), undo=True)
    text.pack(padx=10, pady=6, fill="both", expand=True)
    text.insert("1.0", profile_points_to_text(current_profile_points()))

    btn_frame = tk.Frame(win)
    btn_frame.pack(padx=10, pady=10, fill="x")

    def do_validate():
        try:
            _ = parse_profile_text(text.get("1.0", "end-1c"))
            messagebox.showinfo("OK", "Points look valid.")
        except Exception as e:
            messagebox.showerror("Invalid", str(e))

    def do_apply():
        try:
            pts = parse_profile_text(text.get("1.0", "end-1c"))
        except Exception as e:
            messagebox.showerror("Invalid", str(e))
            return

        PROFILES[selected_profile_name] = pts
        refresh_preview_text()
        preview_selected_profile()
        messagebox.showinfo("Applied", "Applied changes. Click Save to persist to JSON.")
        win.destroy()
        root.destroy()

    def do_cancel():
        win.destroy()
        root.destroy()

    tk.Button(btn_frame, text="Validate", command=do_validate).pack(side="left")
    tk.Button(btn_frame, text="Apply", command=do_apply).pack(side="left", padx=8)
    tk.Button(btn_frame, text="Cancel", command=do_cancel).pack(side="right")

    win.protocol("WM_DELETE_WINDOW", do_cancel)
    win.mainloop()


# ===================== UI callbacks =====================
def on_profile_selected(label):
    if profile_running:
        print("[UI] Can't change profile while running. Stop first.")
        refresh_profile_radio()
        return
    set_selected_profile(label)

def on_new_clicked(_event):
    global PROFILES
    if profile_running:
        print("[UI] Can't create while running. Stop first.")
        return
    name = name_box.text.strip()
    if not name:
        print("[UI] Enter a profile name first.")
        return
    if name in PROFILES:
        print("[UI] That name already exists.")
        return
    PROFILES[name] = list(current_profile_points())
    set_selected_profile(name)
    refresh_profile_radio()
    save_profiles()

def on_save_clicked(_event):
    global PROFILES
    if profile_running:
        print("[UI] Can't save while running. Stop first.")
        return

    desired_name = name_box.text.strip() or selected_profile_name
    if desired_name != selected_profile_name and desired_name in PROFILES:
        print("[UI] That name already exists. Pick a different name.")
        return

    PROFILES[desired_name] = list(current_profile_points())
    set_selected_profile(desired_name)
    refresh_profile_radio()
    save_profiles()
    print(f"[UI] Saved profile '{selected_profile_name}'")

def on_delete_clicked(_event):
    global selected_profile_name
    if profile_running:
        print("[UI] Can't delete while running. Stop first.")
        return
    if len(PROFILES) <= 1:
        print("[UI] Can't delete the last profile.")
        return

    print(f"[UI] Deleting profile '{selected_profile_name}'")
    del PROFILES[selected_profile_name]

    selected_profile_name = list(PROFILES.keys())[0]
    set_selected_profile(selected_profile_name)
    refresh_profile_radio()
    save_profiles()

def on_pause_clicked(_event):
    global paused
    paused = not paused
    pause_button.label.set_text("Resume" if paused else "Pause")
    drain_queue()
    fig.canvas.draw_idle()

def on_clear_clicked(_event):
    global tmin, tmax, frame_count, base_sample, last_alert_state
    global profile_running, profile_t0, last_sp_sent, last_sp_sent_t, target_temp

    print("[UI] Clear / Restart")
    drain_queue()

    xdata.clear(); tdata.clear(); yraw.clear(); yfilt.clear()
    xtarget.clear(); ytarget.clear()
    smooth_buf.clear()

    tmin = None; tmax = None
    frame_count = 0
    last_alert_state = None
    base_sample = None

    profile_running = False
    profile_t0 = None
    last_sp_sent = None
    last_sp_sent_t = 0.0
    target_temp = None
    send_line("RUN:0")

    with csv_lock:
        close_csv()
        open_csv(clear=True)

    line_raw.set_data([], [])
    line_filt.set_data([], [])
    preview_selected_profile()
    status_text.set_text("Cleared. Waiting for data...")
    fig.canvas.draw_idle()

def on_start_clicked(_event):
    global profile_running, profile_t0, last_sp_sent, last_sp_sent_t
    print("[UI] Start Profile")
    drain_queue()

    xtarget.clear(); ytarget.clear()
    line_target.set_data([], [])

    # NEW: send full profile points as P:<t_ms>,<temp_cx100>
    send_profile_points(current_profile_points())

    profile_running = True
    profile_t0 = time.time()
    last_sp_sent = None
    last_sp_sent_t = 0.0
    send_line("RUN:1")
    fig.canvas.draw_idle()

def on_stop_clicked(_event):
    global profile_running, profile_t0, target_temp
    print("[UI] Stop Profile")
    profile_running = False
    profile_t0 = None
    target_temp = None
    send_line("RUN:0")
    preview_selected_profile()
    fig.canvas.draw_idle()

def on_close(_):
    stop_event.set()
    try:
        send_line("RUN:0")
    except Exception:
        pass
    save_profiles()
    close_resources()
    sys.exit(0)


# ===================== SERIAL THREAD =====================
def serial_reader_thread():
    global ser
    sample = -1
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if not line:
                continue
            s = line.decode(errors="ignore").strip()
            parsed = parse_line(s)
            if parsed is None:
                continue
            now = time.time()

            sample += 1
            item = ("TEMP", sample, parsed[1], now)

            try:
                event_q.put_nowait(item)
            except queue.Full:
                try:
                    event_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    event_q.put_nowait(item)
                except queue.Full:
                    pass
        except serial.SerialException as e:
            print("Serial error:", e)
            stop_event.set()
        except Exception:
            pass


# ===================== PLOT / ANIMATION =====================
def init_plot():
    ax.set_title("Reflow Profile Controller (°C)")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Temperature (°C)")
    ax.grid(True)
    ax.legend(loc="lower left", fontsize=9)
    return line_filt, line_raw, line_target, status_text

def animate(_):
    global tmin, tmax, frame_count, base_sample
    global target_temp, last_sp_sent, last_sp_sent_t

    if paused:
        drain_queue()
        return line_filt, line_raw, line_target, status_text

    drained = 0
    got_any_temp = False

    if profile_running and profile_t0 is not None:
        t_sec = time.time() - profile_t0
        target_temp = profile_target_at(t_sec, current_profile_points())

        now_t = time.time()
        if (now_t - last_sp_sent_t) >= SETPOINT_SEND_MIN_DT:
            if (last_sp_sent is None) or (abs(target_temp - last_sp_sent) >= SETPOINT_SEND_DEADBAND):
                # NEW: send integer cx100 for easy MCU parsing
                send_line(f"SP:{c_to_cx100(target_temp)}")
                last_sp_sent = target_temp
                last_sp_sent_t = now_t
    else:
        target_temp = None

    while drained < 400:
        try:
            item = event_q.get_nowait()
        except queue.Empty:
            break
        drained += 1

        _, mcu_sample, temp_raw, now = item
        got_any_temp = True
        temp_raw = temp_raw - TEMP_OFFSET_C

        if base_sample is None:
            base_sample = mcu_sample
        plot_sample = mcu_sample - base_sample

        smooth_buf.append(temp_raw)
        temp_f = sum(smooth_buf) / len(smooth_buf)

        tmin = temp_f if tmin is None else min(tmin, temp_f)
        tmax = temp_f if tmax is None else max(tmax, temp_f)

        xdata.append(plot_sample)
        yraw.append(temp_raw)
        yfilt.append(temp_f)

        if target_temp is not None:
            xtarget.append(plot_sample)
            ytarget.append(target_temp)

        with csv_lock:
            if csv_writer is not None:
                csv_writer.writerow([
                    now,
                    plot_sample,
                    f"{temp_raw:.3f}",
                    f"{temp_f:.3f}",
                    "" if target_temp is None else f"{target_temp:.3f}",
                    "RUN" if profile_running else "IDLE",
                    selected_profile_name
                ])

    if not got_any_temp and drained == 0:
        return line_filt, line_raw, line_target, status_text

    frame_count += 1
    if frame_count % 50 == 0:
        with csv_lock:
            if csv_file is not None:
                csv_file.flush()

    if len(xdata):
        line_raw.set_data(xdata, yraw)
        line_filt.set_data(xdata, yfilt)
        if len(xdata) > 1:
            ax.set_xlim(xdata[0], xdata[-1])

    if profile_running and len(xtarget):
        line_target.set_data(xtarget, ytarget)

    if len(yfilt) > 3 and frame_count % AUTOSCALE_EVERY_N_FRAMES == 0:
        ymin, ymax = min(yfilt), max(yfilt)
        if ymin == ymax:
            ymin -= 0.5
            ymax += 0.5
        pad = 0.1 * (ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)

    if len(yfilt) > 0:
        update_alert(yfilt[-1])

    run_state = "PAUSED" if paused else ("PROFILE RUN" if profile_running else "IDLE")
    if len(yfilt) > 0:
        status_text.set_text(
            "{}\n"
            "Profile: {}\n"
            "Target: {}\n"
            "Now: {:.2f} °C (raw {:.2f})\n"
            "Min: {:.2f}  Max: {:.2f}".format(
                run_state,
                selected_profile_name,
                "--" if target_temp is None else f"{target_temp:.1f} °C",
                yfilt[-1], yraw[-1],
                tmin, tmax
            )
        )
    else:
        status_text.set_text(f"{run_state}\nWaiting for data...")

    return line_filt, line_raw, line_target, status_text


# ===================== MAIN =====================
load_profiles()

try:
    ser = open_serial()
except Exception as e:
    print("Could not open serial port:", e)
    sys.exit(1)

with csv_lock:
    open_csv(clear=False)

t = threading.Thread(target=serial_reader_thread, daemon=True)
t.start()

fig = plt.figure(figsize=(14, 7))
fig.canvas.mpl_connect("close_event", on_close)

# Main plot axes (right side, above bottom bar)
ax = fig.add_axes([
    LEFT_PANEL_W + PAD,
    BOTTOM_BAR_H + PAD,
    1.0 - (LEFT_PANEL_W + 2*PAD),
    1.0 - (BOTTOM_BAR_H + 2*PAD)
])

line_filt, = ax.plot([], [], lw=2, label=f"Filtered (avg {SMOOTH_WINDOW})")
line_raw,  = ax.plot([], [], lw=1, alpha=0.5, label="Raw")
line_target, = ax.plot([], [], lw=2, linestyle="--", label="Target (profile)")
status_text = ax.text(0.02, 0.98, "Waiting for data...", transform=ax.transAxes, va="top")

# ---------- LEFT PANEL ----------
profile_ax = fig.add_axes([PAD, 0.54, LEFT_PANEL_W - 2*PAD, 0.40])
profile_ax.set_facecolor((0.98, 0.98, 0.98))

name_ax = fig.add_axes([PAD, 0.47, LEFT_PANEL_W - 2*PAD, 0.05])
name_box = TextBox(name_ax, "Name", initial=selected_profile_name)
name_box.text_disp.set_fontsize(9)
name_box.text_disp.set_family("monospace")

preview_ax = fig.add_axes([PAD, 0.18, LEFT_PANEL_W - 2*PAD, 0.26])
preview_ax.set_facecolor((0.97, 0.97, 0.97))
preview_ax.set_xticks([])
preview_ax.set_yticks([])
preview_ax.set_frame_on(True)
preview_text = preview_ax.text(
    0.02, 0.98, "",
    va="top", ha="left",
    family="monospace", fontsize=8
)

fig.text(PAD, 0.145, "Edit with popup: One per line -> 60,150", fontsize=8)

edit_btn_ax = fig.add_axes([PAD, 0.12, LEFT_PANEL_W - 2*PAD, 0.05])
edit_btn = Button(edit_btn_ax, "Edit Points…")
edit_btn.on_clicked(open_profile_editor_popup)

if selected_profile_name not in PROFILES:
    selected_profile_name = list(PROFILES.keys())[0]
refresh_profile_radio()
refresh_preview_text()
preview_selected_profile()

# ---------- BOTTOM BAR BUTTONS ----------
btn_y = PAD
btn_h = BOTTOM_BAR_H - 2*PAD
gap = 0.01

def add_btn(x, w, label, handler):
    bax = fig.add_axes([x, btn_y, w, btn_h])
    b = Button(bax, label)
    b.on_clicked(handler)
    return b

btn_left = LEFT_PANEL_W + PAD
x = btn_left

pause_button = add_btn(x, 0.08, "Pause", on_pause_clicked); x += 0.08 + gap
start_button = add_btn(x, 0.08, "Start", on_start_clicked); x += 0.08 + gap
stop_button  = add_btn(x, 0.08, "Stop",  on_stop_clicked);  x += 0.08 + gap
clear_button = add_btn(x, 0.08, "Clear", on_clear_clicked); x += 0.08 + gap
save_button  = add_btn(x, 0.10, "Save",  on_save_clicked);  x += 0.10 + gap
new_button   = add_btn(x, 0.07, "New",   on_new_clicked);   x += 0.07 + gap
del_button   = add_btn(x, 0.07, "Del",   on_delete_clicked)

ani = animation.FuncAnimation(
    fig,
    animate,
    init_func=init_plot,
    interval=PLOT_INTERVAL_MS,
    blit=False,
    cache_frame_data=False
)

plt.show()
