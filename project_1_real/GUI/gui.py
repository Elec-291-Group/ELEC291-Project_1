import sys, time, csv, threading
from collections import deque
import queue

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import serial

# ---------------- SERIAL ----------------
PORT = "COM8"
BAUD = 115200

# ---------------- PLOT / LOG ----------------
xsize = 200
PLOT_INTERVAL_MS = 50
LOG_FILENAME = "temperature_log.csv"

# ---------------- FILTER / ALERT ----------------
SMOOTH_WINDOW = 30
ALERT_HIGH_C = 260.0   # reflow safety-ish default, change as needed
ALERT_LOW_C  = 0.0

AUTOSCALE_EVERY_N_FRAMES = 10
MAX_MARKERS = 200

FLASH_DURATION = 0.30
TEMP_OFFSET_C = 0.0     # you were subtracting 20 earlier; set to 0 unless you need calibration offset

# ---------------- REFLOW PROFILE ----------------
# seconds, °C  (edit this)
PROFILE_POINTS = [
    (0,   25),
    (60,  150),
    (180, 150),
    (240, 235),
    (270, 235),
    (360, 50),
]

# How often to send setpoint updates to MCU
SETPOINT_SEND_MIN_DT = 0.25   # seconds
SETPOINT_SEND_DEADBAND = 0.5  # °C change before re-sending

# ---------------- THREADING / STATE ----------------
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

# Target (profile) plot
xtarget = deque(maxlen=xsize)
ytarget = deque(maxlen=xsize)
target_temp = None

def make_smooth_buf():
    return deque(maxlen=max(1, int(SMOOTH_WINDOW)))

smooth_buf = make_smooth_buf()

alarm_on_x  = deque(maxlen=MAX_MARKERS)
alarm_on_y  = deque(maxlen=MAX_MARKERS)
alarm_off_x = deque(maxlen=MAX_MARKERS)
alarm_off_y = deque(maxlen=MAX_MARKERS)

tmin = None
tmax = None
last_alert_state = None
frame_count = 0
alarm_state = None

base_sample = None
paused = False
flash_until = 0.0

# Profile running state
profile_running = False
profile_t0 = None
last_sp_sent = None
last_sp_sent_t = 0.0


# ---------------- UTIL ----------------
def open_serial():
    return serial.Serial(port=PORT, baudrate=BAUD, timeout=0.1)

def send_line(msg: str):
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.write((msg.strip() + "\n").encode())
    except Exception as e:
        print("[SER] write failed:", e)

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
            "run_state"
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
    """
    Accept:
      - plain float: "47.23"
    Optional future expansions (won't break):
      - "T:47.23"
      - "A:1" / "A:0"
    """
    s = s.strip()
    if not s:
        return None
    su = s.upper()

    if su.startswith("T:"):
        try:
            return ("TEMP", float(s.split(":", 1)[1].strip()))
        except:
            return None

    if su.startswith("A:"):
        rhs = s.split(":", 1)[1].strip()
        if rhs in ("0", "1"):
            return ("ALARM", "ON" if rhs == "1" else "OFF")
        return None

    # current format: just a float
    try:
        return ("TEMP", float(s))
    except ValueError:
        return None

def offsets2d(xs, ys):
    n = min(len(xs), len(ys))
    if n == 0:
        return np.empty((0, 2))
    return np.column_stack((list(xs)[:n], list(ys)[:n]))

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


# ---------------- SERIAL READER THREAD ----------------
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

            if parsed[0] == "TEMP":
                sample += 1
                item = ("TEMP", sample, parsed[1], now)
            else:
                item = ("ALARM", parsed[1], now)

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


# ---------------- PLOT ----------------
def init_plot():
    ax.set_title("Reflow Profile Controller (°C)")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Temperature (°C)")
    ax.grid(True)
    ax.legend(loc="lower left")
    return line_filt, line_raw, line_target, sc_on, sc_off, status_text

def drain_queue():
    while True:
        try:
            event_q.get_nowait()
        except queue.Empty:
            break

def animate(_):
    global tmin, tmax, frame_count, alarm_state, base_sample, flash_until
    global target_temp, last_sp_sent, last_sp_sent_t

    if paused:
        drain_queue()
        ax.set_facecolor((1.0, 1.0, 1.0))
        fig.patch.set_facecolor((1.0, 1.0, 1.0))
        return line_filt, line_raw, line_target, sc_on, sc_off, status_text

    drained = 0
    got_any_temp = False

    # --- handle profile: compute target & send setpoint (rate-limited) ---
    if profile_running and profile_t0 is not None:
        t_sec = time.time() - profile_t0
        target_temp = profile_target_at(t_sec, PROFILE_POINTS)

        now_t = time.time()
        if (now_t - last_sp_sent_t) >= SETPOINT_SEND_MIN_DT:
            if (last_sp_sent is None) or (abs(target_temp - last_sp_sent) >= SETPOINT_SEND_DEADBAND):
                send_line(f"SP:{target_temp:.1f}")
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

        if item[0] == "ALARM":
            new_state, alarm_time = item[1], item[2]
            if new_state != alarm_state:
                alarm_state = new_state
                print(f"[MCU] Alarm state -> {alarm_state}")

                if alarm_state == "ON":
                    flash_until = time.time() + FLASH_DURATION

                if len(tdata) > 0 and len(xdata) > 0 and len(yraw) > 0:
                    td = np.array(tdata, dtype=float)
                    idx = int(np.argmin(np.abs(td - alarm_time)))
                    if 0 <= idx < len(xdata) and 0 <= idx < len(yraw):
                        xs = list(xdata)[idx]
                        ys = list(yraw)[idx]
                        if alarm_state == "ON":
                            alarm_on_x.append(xs)
                            alarm_on_y.append(ys)
                        else:
                            alarm_off_x.append(xs)
                            alarm_off_y.append(ys)
            continue

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
        tdata.append(now)
        yraw.append(temp_raw)
        yfilt.append(temp_f)

        # target line uses same x axis if available
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
                    "RUN" if profile_running else "IDLE"
                ])

    if not got_any_temp and drained == 0:
        now_t = time.time()
        if now_t < flash_until:
            ax.set_facecolor((1.0, 0.85, 0.85))
            fig.patch.set_facecolor((1.0, 0.92, 0.92))
        else:
            ax.set_facecolor((1.0, 1.0, 1.0))
            fig.patch.set_facecolor((1.0, 1.0, 1.0))
        return line_filt, line_raw, line_target, sc_on, sc_off, status_text

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

    if len(xtarget):
        line_target.set_data(xtarget, ytarget)

    sc_on.set_offsets(offsets2d(alarm_on_x, alarm_on_y))
    sc_off.set_offsets(offsets2d(alarm_off_x, alarm_off_y))

    if len(yfilt) > 3 and frame_count % AUTOSCALE_EVERY_N_FRAMES == 0:
        ymin, ymax = min(yfilt), max(yfilt)
        if ymin == ymax:
            ymin -= 0.5
            ymax += 0.5
        pad = 0.1 * (ymax - ymin)
        ax.set_ylim(ymin - pad, ymax + pad)

    if len(yfilt) > 0:
        update_alert(yfilt[-1])

    now_t = time.time()
    if now_t < flash_until:
        ax.set_facecolor((1.0, 0.85, 0.85))
        fig.patch.set_facecolor((1.0, 0.92, 0.92))
    else:
        ax.set_facecolor((1.0, 1.0, 1.0))
        fig.patch.set_facecolor((1.0, 1.0, 1.0))

    run_state = "PAUSED" if paused else ("PROFILE RUN" if profile_running else "IDLE")

    if len(yfilt) > 0:
        status_text.set_text(
            "{}\n"
            "Target: {}\n"
            "Now: {:.2f} °C (raw {:.2f})\n"
            "Min: {:.2f}  Max: {:.2f}\n"
            "Markers: ON={} OFF={}".format(
                run_state,
                "--" if target_temp is None else f"{target_temp:.1f} °C",
                yfilt[-1], yraw[-1],
                tmin, tmax,
                len(alarm_on_x), len(alarm_off_x)
            )
        )
    else:
        status_text.set_text("{}\nWaiting for data...".format(run_state))

    return line_filt, line_raw, line_target, sc_on, sc_off, status_text


# ---------------- UI HANDLERS ----------------
def on_clear_clicked(_event):
    global tmin, tmax, frame_count, alarm_state, base_sample, last_alert_state, flash_until, smooth_buf
    global profile_running, profile_t0, last_sp_sent, last_sp_sent_t, target_temp

    print("[UI] Clear / Restart")

    drain_queue()

    xdata.clear()
    tdata.clear()
    yraw.clear()
    yfilt.clear()
    xtarget.clear()
    ytarget.clear()

    smooth_buf = make_smooth_buf()
    smooth_buf.clear()

    alarm_on_x.clear()
    alarm_on_y.clear()
    alarm_off_x.clear()
    alarm_off_y.clear()

    tmin = None
    tmax = None
    frame_count = 0
    alarm_state = None
    last_alert_state = None
    base_sample = None
    flash_until = 0.0

    # stop profile too
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
    line_target.set_data([], [])
    sc_on.set_offsets(np.empty((0, 2)))
    sc_off.set_offsets(np.empty((0, 2)))
    status_text.set_text("Cleared. Waiting for data...")

    ax.set_facecolor((1.0, 1.0, 1.0))
    fig.patch.set_facecolor((1.0, 1.0, 1.0))
    ax.relim()
    ax.autoscale_view()
    plt.draw()

def on_pause_clicked(_event):
    global paused, flash_until
    paused = not paused
    if paused:
        print("[UI] Paused (plot+CSV)")
        pause_button.label.set_text("Resume")
        flash_until = 0.0
        status_text.set_text("PAUSED\nWaiting...")
    else:
        print("[UI] Resumed (plot+CSV)")
        pause_button.label.set_text("Pause")
        drain_queue()
    plt.draw()

def on_start_profile(_event):
    global profile_running, profile_t0, last_sp_sent, last_sp_sent_t
    print("[UI] Start Profile")
    drain_queue()
    profile_running = True
    profile_t0 = time.time()
    last_sp_sent = None
    last_sp_sent_t = 0.0
    send_line("RUN:1")
    plt.draw()

def on_stop_profile(_event):
    global profile_running, profile_t0
    print("[UI] Stop Profile")
    profile_running = False
    profile_t0 = None
    send_line("RUN:0")
    plt.draw()

def on_close(_):
    stop_event.set()
    try:
        send_line("RUN:0")
    except Exception:
        pass
    close_resources()
    sys.exit(0)


# ---------------- MAIN ----------------
try:
    ser = open_serial()
except Exception as e:
    print("Could not open serial port:", e)
    sys.exit(1)

with csv_lock:
    open_csv(clear=False)

t = threading.Thread(target=serial_reader_thread, daemon=True)
t.start()

fig = plt.figure()
fig.canvas.mpl_connect("close_event", on_close)

ax = fig.add_subplot(111)

line_filt, = ax.plot([], [], lw=2, label=f"Filtered (avg {SMOOTH_WINDOW})")
line_raw,  = ax.plot([], [], lw=1, alpha=0.5, label="Raw")
line_target, = ax.plot([], [], lw=2, linestyle="--", label="Target (profile)")

sc_on = ax.scatter([], [], marker="^", s=60, label="Alarm ON")
sc_off = ax.scatter([], [], marker="v", s=60, label="Alarm OFF")

status_text = ax.text(0.02, 0.98, "Waiting for data...", transform=ax.transAxes, va="top")

pause_ax = plt.axes([0.10, 0.02, 0.18, 0.06])
pause_button = Button(pause_ax, "Pause")
pause_button.on_clicked(on_pause_clicked)

clear_ax = plt.axes([0.78, 0.02, 0.20, 0.06])
clear_button = Button(clear_ax, "Clear")
clear_button.on_clicked(on_clear_clicked)

start_ax = plt.axes([0.32, 0.02, 0.20, 0.06])
start_button = Button(start_ax, "Start Profile")
start_button.on_clicked(on_start_profile)

stop_ax = plt.axes([0.54, 0.02, 0.20, 0.06])
stop_button = Button(stop_ax, "Stop Profile")
stop_button.on_clicked(on_stop_profile)

ani = animation.FuncAnimation(
    fig,
    animate,
    init_func=init_plot,
    interval=PLOT_INTERVAL_MS,
    blit=False,
    cache_frame_data=False
)

plt.show()
