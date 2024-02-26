import asyncio
from bleak import BleakScanner, BleakClient

# on the pi:
# - bluetoothctl
# - devices Paired
# - devices Connected
# - tab works btw
# - raspberrypi mac address: D8:3A:DD:3A:F1:E3
# - pc mac address: 

# on the pc: 
# - manually establish connection
# - go to bluetooth > more bluetooth settings > check the com ports for serial 
#   communication
#   in our case, com7 is used for outgoing (from the pc) ti raspberrypi (serial port)
#   and the com8 is used for incoming from raspberrypi

# Define the address (MAC address) of the Raspberry Pi device
raspberry_pi_address = "D8:3A:DD:3A:F1:E3"  
# Use the Characteristic User Description UUID
characteristic_uuid = "00002901-0000-1000-8000-00805f9b34fb"

async def receive_data():
    async with BleakClient(raspberry_pi_address) as client:
        while True:
            try:
                # Read data from the specified characteristic
                data = await client.read_gatt_char(characteristic_uuid)
                print("Received data:", data.decode())
            except Exception as e:
                print("Error:", e)
                break

asyncio.run(receive_data())