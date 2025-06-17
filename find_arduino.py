import serial.tools.list_ports

ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(ports): # find port with Arduino
    if "Arduino Uno" in desc:
        print(port.split("COM")[1])
        break