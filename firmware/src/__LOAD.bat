@echo off
::This file was created automatically by CrossIDE to load a hex file using Quartus_stp.
"D:\intelFPGA\quartus\bin64\quartus_stp.exe" -t "D:\CrossIDE\CrossIDE\Load_Script.tcl" "D:\my51code\51aCode\ELEC291-Project_1\Firmware\src\project_1_frame.HEX" | find /v "Warning (113007)"
