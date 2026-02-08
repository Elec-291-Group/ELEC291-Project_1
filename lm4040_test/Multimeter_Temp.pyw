#!/usr/bin/python
from tkinter import *
import time
import serial
import sys
import kconvert
import csv
import os

top = Tk()
top.resizable(0, 0)
top.title("DMM (COM13) vs DE10 (COM12) Thermocouple Compare")

# ===================== USER SETTINGS =====================
DMM_PORT  = "COM13"      # Fluke/Tek DMM4020
DMM_BAUD  = 9600

DE10_PORT = "COM12"      # DE10 USB-Serial adapter
DE10_BAUD = 115200

UPDATE_MS = 1000         # 1 Hz to match DE10 loop
# =========================================================

CJTemp = StringVar()
Temp = StringVar()
DMMout = StringVar()
portstatus = StringVar()
DMM_Name = StringVar()

ser = None    # DMM
ser2 = None   # DE10

last_de10_raw = None    # raw value from DE10 (scaled by 1/1000)

# ---------------- CSV SETTINGS ----------------
CSV_FILENAME = os.path.join(
    os.path.dirname(os.path.abspath(sys.argv[0])),
    "thermocouple_log.csv"
)

with open(CSV_FILENAME, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow([
        "python_temp_C",
        "de10_temp_C",
        "python_minus_de10_C"
    ])
# ---------------------------------------------

def Just_Exit():
    top.destroy()
    try:
        if ser:
            ser.close()
    except:
        pass
    try:
        if ser2:
            ser2.close()
    except:
        pass

def log_to_csv(py_c, de10_c):
    with open(CSV_FILENAME, "a", newline="") as f:
        w = csv.writer(f)
        if de10_c is None:
            w.writerow([round(py_c, 1), "", ""])
        else:
            diff = py_c - de10_c
            w.writerow([
                round(py_c, 1),
                round(de10_c, 1),
                round(diff, 1)
            ])
        f.flush()

def try_parse_number(text):
    """
    Extract first valid float from a DE10 serial line.
    Example: 'T=0.022 C' -> 0.022
    """
    s = text.strip()
    s = s.replace("T=", " ")
    s = s.replace("V=", " ")
    s = s.replace("=", " ")
    s = s.replace("C", " ")
    s = s.replace("°", " ")
    for tok in s.replace(",", " ").split():
        try:
            return float(tok)
        except:
            continue
    return None

def drain_de10():
    """Non-blocking read of DE10; keep most recent valid value"""
    global last_de10_raw
    if ser2 is None:
        return
    try:
        n = ser2.in_waiting
        if n <= 0:
            return
        data = ser2.read(n).decode(errors="ignore")
        lines = data.replace("\r", "\n").split("\n")
        for s in reversed(lines):
            if s.strip():
                v = try_parse_number(s)
                if v is not None:
                    last_de10_raw = v
                break
    except:
        return

def update_temp():
    global last_de10_raw

    # --- Update DE10 first ---
    drain_de10()

    # --- Read DMM ---
    try:
        line = ser.readline().rstrip().decode(errors="ignore")
        ser.readline()  # discard "=>"
        if len(line) > 1 and line[1] == '>':
            line = ser.readline().decode(errors="ignore")
        ser.write(b"MEAS1?\r\n")
    except Exception as e:
        DMMout.set("----")
        Temp.set("----")
        portstatus.set("DMM read error | " + str(e))
        top.after(2000, update_temp)
        return

    DMMout.set(line.replace("\r", "").replace("\n", ""))

    try:
        mv = float(line.replace("VDC", "").strip()) * 1000.0
        valid = True
    except:
        valid = False

    try:
        cj = float(CJTemp.get())
    except:
        cj = 0.0

    # --- Apply DE10 scaling ---
    # DE10 sends temperature scaled by 1/1000
    if last_de10_raw is not None:
        de10_c = last_de10_raw * 1000.0
    else:
        de10_c = None

    if valid:
        ktemp = round(kconvert.mV_to_C(mv, cj), 1)

        if ktemp < -200:
            Temp.set("UNDER")
        elif ktemp > 1372:
            Temp.set("OVER")
        else:
            Temp.set(ktemp)
            log_to_csv(ktemp, de10_c)

            if de10_c is None:
                portstatus.set("Logging (waiting for DE10)")
            else:
                portstatus.set(
                    "Logging OK | Δ = " +
                    str(round(ktemp - de10_c, 1)) + " C"
                )
    else:
        Temp.set("----")
        portstatus.set("Bad DMM parse")

    top.after(UPDATE_MS, update_temp)

def init_ports():
    global ser, ser2

    portstatus.set("Opening ports...")
    top.update()

    # --- DE10 ---
    try:
        ser2 = serial.Serial(DE10_PORT, DE10_BAUD, timeout=0)
        try:
            ser2.setDTR(True)
            ser2.setRTS(True)
        except:
            pass
    except Exception as e:
        portstatus.set("DE10 open failed | " + str(e))
        return

    # --- DMM ---
    try:
        ser = serial.Serial(DMM_PORT, DMM_BAUD, timeout=0.5)
        time.sleep(0.2)
        ser.write(b"\x03")
        _ = ser.readline()
        ser.timeout = 3
        ser.write(b"VDC; RATE S; *IDN?\r\n")
        devicename = ser.readline().rstrip().decode(errors="ignore")
        DMM_Name.set(devicename)
        ser.readline()
        ser.write(b"MEAS1?\r\n")
    except Exception as e:
        portstatus.set("DMM open failed | " + str(e))
        return

    portstatus.set(
        "Ports open | DE10=" + DE10_PORT +
        " | DMM=" + DMM_PORT
    )
    top.after(1000, update_temp)

# ---------------- UI ----------------
Label(top, text="Cold Junction Temperature:").grid(row=1, column=0)
Entry(top, bd=1, width=7, textvariable=CJTemp).grid(row=2, column=0)

Label(top, text="Multimeter reading:").grid(row=3, column=0)
Label(top, textvariable=DMMout, width=20,
      font=("Helvetica", 20), fg="red").grid(row=4, column=0)

Label(top, text="Thermocouple Temperature (C)").grid(row=5, column=0)
Label(top, textvariable=Temp, width=5,
      font=("Helvetica", 100), fg="blue").grid(row=6, column=0)

Label(top, textvariable=portstatus, width=80,
      font=("Helvetica", 12)).grid(row=7, column=0)
Label(top, textvariable=DMM_Name, width=80,
      font=("Helvetica", 12)).grid(row=8, column=0)

Button(top, width=11, text="Exit", command=Just_Exit).grid(row=9, column=0)

CJTemp.set("22")
DMMout.set("NO DATA")
DMM_Name.set("--------")
portstatus.set("CSV: " + CSV_FILENAME)

init_ports()
top.mainloop()
