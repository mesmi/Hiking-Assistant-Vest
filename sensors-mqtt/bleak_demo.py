import asyncio
import platform
import sys
import threading
import paho.mqtt.client as mqtt

import bleak.exc
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

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

# The reason for the distinction between Darwin kernel OS and other OSes
# is because Apple (which uses the Darwin kernel) uses a BT address that is 
# colon separated, and other OSes use a different format
# ADDRESS = (
#     "XX-XX-XX-XX-XX-XX" # kemal's iphone's bluetooth address
#     if platform.system() != "Darwin"
#     else "----"
# )

ADDRESS = "04:68:65:CD:70:69"

sensor_data = {
    "data1" : "Initial data"
}


# MQTT Client Setup
broker_address = "127.0.0.1"
broker_port = 1883
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("havpi/sensordata")
    else:
        print(f"Failed to connect, return code {rc}\n")


def on_message(client, userdata, message):
    sensor_data["data1"] = message.payload.decode()
    print("Message received: ", sensor_data["data1"])

def run_mqtt_client():
    mqtt_client.connect(broker_address, port=broker_port)
    mqtt_client.loop_forever()

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message


async def main(ble_address: str):
    try:
        device = await BleakScanner.find_device_by_address(ble_address, timeout=10.0)
        if not device:
            raise BleakError(f"A device with address {ble_address} could not be found.")
        print("Device connected!")
        async with BleakClient(device, timeout=22.0) as client:
            # print("Services:")
            # for service in client.services:
            #     print(service)
            get_data_thread = threading.Thread(target=run_mqtt_client)
            get_data_thread.start()
            
 

    except bleak.exc.BleakError as e:
        print(e)

while True:
    asyncio.run(main(sys.argv[1] if len(sys.argv) == 2 else ADDRESS))