#!/usr/bin/python
from tkinter import *
import time
import serial
import serial.tools.list_ports
import sys
import kconvert

import csv

top = Tk()
top.resizable(0,0)
top.title("Fluke_45/Tek_DMM4020 K-type Thermocouple")

#ATTENTION: Make sure the multimeter is configured at 9600 baud, 8-bits, parity none, 1 stop bit, echo Off

CJTemp = StringVar()
Temp = StringVar()
DMMout = StringVar()
portstatus = StringVar()
DMM_Name = StringVar()
connected=0
global ser

# ---------------- CSV SETTINGS ----------------
CSV_FILENAME = "thermocouple_log.csv"
# Clear CSV on every program run + write header (3 columns as requested)
with open(CSV_FILENAME, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["thermocouple_C", "setpoint_C", "temp_difference_C"])
# ---------------------------------------------

# ---------------- REFLOW PROFILES ----------------
# Format: list of (time_seconds, setpoint_temp_C)
# Setpoint is linearly interpolated between points.
REFLOW_PROFILES = {
    "Lead-Free (Example)": [
        (0,   25),
        (60,  150),
        (120, 180),
        (180, 230),
        (210, 245),
        (240, 245),
        (300, 180),
        (360,  80),
    ],
    "Leaded (Example)": [
        (0,   25),
        (60,  140),
        (120, 165),
        (180, 205),
        (210, 215),
        (240, 215),
        (300, 170),
        (360,  80),
    ],
}
# -----------------------------------------------

# Profile selection + display
profile_var = StringVar()
profile_var.set(list(REFLOW_PROFILES.keys())[0])

SetpointStr = StringVar()
ElapsedStr = StringVar()
SetpointStr.set("Setpoint: ---- C")
ElapsedStr.set("t = ---- s")

profile_start_time = None  # starts when we first connect and begin measuring


def Just_Exit():
    top.destroy()
    try:
        ser.close()
    except:
        pass


def log_to_csv(tc_c, sp_c):
    try:
        diff = tc_c - sp_c
        with open(CSV_FILENAME, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([tc_c, sp_c, round(diff, 1)])
    except:
        pass


def profile_setpoint_C(profile_points, t_s):
    """
    Piecewise-linear interpolation of setpoint temperature vs time.
    - If t_s is before first point: return first temp
    - If t_s is after last point: return last temp (hold)
    """
    if not profile_points:
        return None

    if t_s <= profile_points[0][0]:
        return float(profile_points[0][1])

    for i in range(len(profile_points) - 1):
        t0, y0 = profile_points[i]
        t1, y1 = profile_points[i + 1]
        if t0 <= t_s <= t1:
            if t1 == t0:
                return float(y1)
            frac = (t_s - t0) / (t1 - t0)
            return float(y0 + frac * (y1 - y0))

    return float(profile_points[-1][1])


def update_temp():
    global ser, connected, profile_start_time

    if connected==0:
        top.after(5000, FindPort) # Not connected, try to reconnect again in 5 seconds
        return

    # Start timing once we're connected (so profile time aligns to measurement start)
    if profile_start_time is None:
        profile_start_time = time.time()

    try:
        strin = ser.readline() # Read the requested value, for example "+0.234E-3 VDC"
        strin = strin.rstrip()
        strin = strin.decode()
        print(strin)
        ser.readline() # Read and discard the prompt "=>"
        if len(strin)>1:
            if strin[1]=='>': # Out of sync?
                strin = ser.readline() # Read the value again
        ser.write(b"MEAS1?\r\n") # Request next value from multimeter
    except:
        connected=0
        DMMout.set("----")
        Temp.set("----");
        portstatus.set("Communication Lost")
        DMM_Name.set ("--------")
        SetpointStr.set("Setpoint: ---- C")
        ElapsedStr.set("t = ---- s")
        profile_start_time = None
        top.after(5000, FindPort) # Try to reconnect again in 5 seconds
        return

    strin_clean = strin.replace("VDC","") # get rid of the units as the 'float()' function doesn't like it

    if len(strin_clean) > 0:
       DMMout.set(strin.replace("\r", "").replace("\n", "")) # display the information received from the multimeter

       try:
           val=float(strin_clean)*1000.0 # Convert from volts to millivolts
           valid_val=1;
       except:
           valid_val=0

       try:
          cj=float(CJTemp.get()) # Read the cold junction temperature in degrees centigrade
       except:
          cj=0.0 # If the input is blank, assume cold junction temperature is zero degrees centigrade

       if valid_val == 1 :
           ktemp=round(kconvert.mV_to_C(val, cj),1)

           elapsed_s = time.time() - profile_start_time
           points = REFLOW_PROFILES.get(profile_var.get(), [])
           sp = round(profile_setpoint_C(points, elapsed_s), 1)


           if sp is None:
               SetpointStr.set("Setpoint: ---- C")
           else:
               SetpointStr.set("Setpoint: %.1f C" % sp)

           ElapsedStr.set("t = %d s" % int(elapsed_s))

           if ktemp < -200:
               Temp.set("UNDER")
           elif ktemp > 1372:
               Temp.set("OVER")
           else:
               Temp.set(ktemp)
               if sp is not None:
                   log_to_csv(ktemp, sp)
       else:
           Temp.set("----");
    else:
       Temp.set("----");
       connected=0;

    top.after(500, update_temp) # The multimeter is slow and the baud rate is slow: two measurement per second tops!


def FindPort():
   global ser, connected, profile_start_time
   try:
       ser.close()
   except:
       dummy=0

   connected=0
   DMM_Name.set ("--------")
   profile_start_time = None

   portlist=list(serial.tools.list_ports.comports())
   for item in reversed(portlist):
      portstatus.set("Trying port " + item[0])
      top.update()
      try:
         ser = serial.Serial(item[0], 9600, timeout=0.5)
         ser.write(b"\x03") # Request prompt from possible multimeter
         pstring = ser.readline() # Read the prompt "=>"
         pstring=pstring.rstrip()
         pstring=pstring.decode()
         # print(pstring)
         if len(pstring) > 1:
            if pstring[1]=='>':
               ser.timeout=3  # Three seconds timeout to receive data should be enough
               portstatus.set("Connected to " + item[0])
               ser.write(b"VDC; RATE S; *IDN?\r\n") # Measure DC voltage, set scan rate to 'Slow' for max resolution, get multimeter ID
               devicename=ser.readline()
               devicename=devicename.rstrip()
               devicename=devicename.decode()
               DMM_Name.set(devicename.replace("\r", "").replace("\n", ""))
               ser.readline() # Read and discard the prompt "=>"
               ser.write(b"MEAS1?\r\n") # Request first value from multimeter
               connected=1
               top.after(1000, update_temp)
               break
            else:
               ser.close()
         else:
            ser.close()
      except:
         connected=0
   if connected==0:
      portstatus.set("Multimeter not found")
      top.after(5000, FindPort) # Try again in 5 seconds


Label(top, text="Cold Junction Temperature:").grid(row=1, column=0)
Entry(top, bd =1, width=7, textvariable=CJTemp).grid(row=2, column=0)

Label(top, text="Reflow Profile:").grid(row=3, column=0)
OptionMenu(top, profile_var, *REFLOW_PROFILES.keys()).grid(row=4, column=0)

Label(top, textvariable=ElapsedStr, width=20, font=("Helvetica", 12)).grid(row=5, column=0)
Label(top, textvariable=SetpointStr, width=20, font=("Helvetica", 12)).grid(row=6, column=0)

Label(top, text="Multimeter reading:").grid(row=7, column=0)
Label(top, text="xxxx", textvariable=DMMout, width=20, font=("Helvetica", 20), fg="red").grid(row=8, column=0)

Label(top, text="Thermocouple Temperature (C)").grid(row=9, column=0)
Label(top, textvariable=Temp, width=5, font=("Helvetica", 100), fg="blue").grid(row=10, column=0)

Label(top, text="xxxx", textvariable=portstatus, width=40, font=("Helvetica", 12)).grid(row=11, column=0)
Label(top, text="xxxx", textvariable=DMM_Name, width=40, font=("Helvetica", 12)).grid(row=12, column=0)
Button(top, width=11, text = "Exit", command = Just_Exit).grid(row=13, column=0)

CJTemp.set ("22")
DMMout.set ("NO DATA")
DMM_Name.set ("--------")

top.after(500, FindPort)
top.mainloop()
