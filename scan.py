import asyncio
from bleak import BleakScanner

# Replace with the BLE address of your target device
TARGET_ADDRESS = "A0:6C:65:D9:56:B0"

def parse_manufacturer_data(data: bytes):
    distance = int.from_bytes(data[0:3], byteorder="little")
    speed = int.from_bytes(data[3:5], byteorder="little")

    print(f"Distance (revolutions): {distance}")
    print(f"Speed (ms/revolution): {speed}")

def on_device_discovery_callback(device, advertisement_data):
    if device.address != TARGET_ADDRESS:
        return
    
    parse_manufacturer_data(advertisement_data.manufacturer_data[65535])

async def main():
    scanner = BleakScanner(on_device_discovery_callback)

    await scanner.start()
    await asyncio.sleep(600)

asyncio.run(main())