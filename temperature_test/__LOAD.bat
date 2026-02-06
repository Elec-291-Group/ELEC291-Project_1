@echo off
::This file was created automatically by CrossIDE to load a hex file using Quartus_stp.
"C:\intelFPGA_lite\24.1std\quartus\bin64\quartus_stp.exe" -t "D:\CrossIDE\Load_Script.tcl" "D:\Coding\ELEC291-Project_1\temperature_test\ADC_to_voltage_temp_7seg.HEX" | find /v "Warning (113007)"
