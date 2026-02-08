import time
import threading
from datetime import datetime

import serial
import serial.tools.list_ports

import openpyxl
from openpyxl import Workbook

import kconvert  # uses mV_to_C() conversion

# ===================== USER SETTINGS =====================
MCU_PORT = "COM11"
MCU_BAUD = 115200

DMM_BAUD = 9600
DMM_TIMEOUT_S = 3.0

# Cold junction temp (same idea as your GUI entry)
COLD_JUNCTION_C = 22.0

# Output file
OUT_XLSX = "dmm_vs_mcu_log.xlsx"

# How often to sample (your GUI did ~2 Hz)
SAMPLE_PERIOD_S = 0.5

# Save every N rows (protects data if you stop the script)
SAVE_EVERY_N_ROWS = 10
# =========================================================


def find_dmm_port():
    """
    Mimics your existing approach: probe ports, send Ctrl+C, look for '=>'
    """
    ports = list(serial.tools.list_ports.comports())
    for p in reversed(ports):
        port_name = p.device
        try:
            ser = serial.Serial(port_name, DMM_BAUD, timeout=0.5)
            time.sleep(0.2)
            ser.write(b"\x03")           # request prompt
            prompt = ser.readline().decode(errors="ignore")
            if len(prompt) > 1 and prompt[1] == ">":
                # Found it. Configure like your GUI script.
                ser.timeout = DMM_TIMEOUT_S
                ser.write(b"VDC; RATE S; *IDN?\r\n")
                dev = ser.readline().decode(errors="ignore").strip()
                ser.readline()  # discard prompt "=>"
                ser.write(b"MEAS1?\r\n")  # request first value
                return ser, port_name, dev
            ser.close()
        except Exception:
            try:
                ser.close()
            except Exception:
                pass
    return None, None, None


class LatestValue:
    """Thread-safe holder for latest MCU temperature reading."""
    def __init__(self):
        self.lock = threading.Lock()
        self.value = None
        self.timestamp = None

    def set(self, v):
        with self.lock:
            self.value = v
            self.timestamp = time.time()

    def get(self):
        with self.lock:
            return self.value, self.timestamp


def mcu_reader_thread(mcu_ser, latest: LatestValue, stop_event: threading.Event):
    """
    Continuously read MCU lines (expects plain float like your .pyw does).
    """
    while not stop_event.is_set():
        try:
            line = mcu_ser.readline()
            if not line:
                continue
            s = line.decode(errors="ignore").strip()
            if not s:
                continue
            val = float(s)
            latest.set(val)
        except Exception:
            # Don’t kill the whole program if MCU sends junk once
            continue


def parse_dmm_reading(line: str):
    """
    Example line from DMM: "+0.234E-3 VDC"
    Returns (raw_str, volts_float, mV_float) or (raw_str, None, None)
    """
    raw = line.strip()
    cleaned = raw.replace("VDC", "").strip()
    try:
        v = float(cleaned)
        mv = v * 1000.0
        return raw, v, mv
    except Exception:
        return raw, None, None


def setup_workbook(path: str):
    try:
        wb = openpyxl.load_workbook(path)
        ws = wb.active
        return wb, ws
    except Exception:
        wb = Workbook()
        ws = wb.active
        ws.title = "Log"
        ws.append([
            "timestamp_iso",
            "t_seconds",
            "dmm_raw",
            "dmm_volts",
            "dmm_millivolts",
            "k_type_temp_c_from_dmm",
            "mcu_temp_c",
            "abs_diff_c",
            "mcu_age_s"
        ])
        wb.save(path)
        return wb, ws


def main():
    # --- Connect MCU ---
    mcu_ser = serial.Serial(MCU_PORT, MCU_BAUD, timeout=0.1)
    print(f"MCU connected on {MCU_PORT} @ {MCU_BAUD}")

    # --- Find + connect DMM ---
    dmm_ser, dmm_port, dmm_idn = find_dmm_port()
    if dmm_ser is None:
        raise RuntimeError("Could not find multimeter (no port responded with '=>').")
    print(f"DMM connected on {dmm_port} @ {DMM_BAUD}")
    print(f"DMM ID: {dmm_idn}")

    # --- Start MCU reader thread ---
    latest_mcu = LatestValue()
    stop_event = threading.Event()
    th = threading.Thread(target=mcu_reader_thread, args=(mcu_ser, latest_mcu, stop_event), daemon=True)
    th.start()

    # --- Excel workbook ---
    wb, ws = setup_workbook(OUT_XLSX)
    rows_since_save = 0
    t0 = time.time()

    print(f"Logging to: {OUT_XLSX}")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            # Read DMM line (your GUI expects: reading line, then prompt line)
            dmm_line = dmm_ser.readline().decode(errors="ignore")
            _ = dmm_ser.readline()  # discard prompt "=>"

            # If out-of-sync (your GUI checks if strin[1]=='>')
            if len(dmm_line) > 1 and dmm_line[1] == ">":
                dmm_line = dmm_ser.readline().decode(errors="ignore")
                _ = dmm_ser.readline()

            # Immediately request next value
            dmm_ser.write(b"MEAS1?\r\n")

            # Parse DMM
            dmm_raw, dmm_v, dmm_mv = parse_dmm_reading(dmm_line)

            # Latest MCU temp
            mcu_temp, mcu_ts = latest_mcu.get()
            mcu_age = None if mcu_ts is None else (time.time() - mcu_ts)

            # Convert DMM mV -> K-type temp
            ktemp = None
            if dmm_mv is not None:
                try:
                    ktemp = float(round(kconvert.mV_to_C(dmm_mv, COLD_JUNCTION_C), 2))
                except Exception:
                    ktemp = None

            # Compare
            abs_diff = None
            if (ktemp is not None) and (mcu_temp is not None):
                abs_diff = abs(ktemp - mcu_temp)

            now = datetime.now().isoformat(timespec="milliseconds")
            t = time.time() - t0

            ws.append([
                now,
                t,
                dmm_raw,
                dmm_v,
                dmm_mv,
                ktemp,
                mcu_temp,
                abs_diff,
                mcu_age
            ])
            rows_since_save += 1

            if rows_since_save >= SAVE_EVERY_N_ROWS:
                wb.save(OUT_XLSX)
                rows_since_save = 0

            time.sleep(SAMPLE_PERIOD_S)

    except KeyboardInterrupt:
        print("\nStopping... saving workbook.")
        wb.save(OUT_XLSX)

    finally:
        stop_event.set()
        try:
            dmm_ser.close()
        except Exception:
            pass
        try:
            mcu_ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
