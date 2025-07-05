import asyncio
from bleak import BleakClient, BleakScanner

TARGET_ADDRESS = "A0:6C:65:D9:56:B0"
SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
WHEEL_CIRCUMFERENCE = 2.2

def parse_data(data: bytearray):
    if len(data) != 5:
        print(f"Unexpected data length: {len(data)}")
        return

    distance = data[0] | (data[1] << 8) | (data[2] << 16)
    speed = data[3] | (data[4] << 8)
    print()
    print(f"Distance: {distance * WHEEL_CIRCUMFERENCE:.1f} m")
    if speed != 0:
        print(f"Speed: {WHEEL_CIRCUMFERENCE * 3600 / speed:.1f} km/h")
    else:
        print("Speed: 0 km/h")

async def main():
    device = await BleakScanner.find_device_by_address(TARGET_ADDRESS, timeout=10)

    if not device:
        print("Device not found")
        return

    print("Found device, connecting")

    async with BleakClient(device) as client:
        if not client.is_connected:
            print("Failed to connect to the device.")
            return

        print("Connected!")
        print("Attempting to subscribe to notifications")

        # Subscribe to notifications
        await client.start_notify(CHARACTERISTIC_UUID, lambda _, data: parse_data(data))

        print("Listening for notifications")
        await asyncio.sleep(600)

if __name__ == "__main__":
    asyncio.run(main())