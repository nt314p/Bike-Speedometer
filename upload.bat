@echo off

for /f "delims=" %%A in ('python find_arduino.py') do set "PORT=%%A"

ccloader %PORT% bin/CC2541-HID.bin 0