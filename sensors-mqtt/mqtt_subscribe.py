import paho.mqtt.client as mqtt
import threading

# MQTT Client Setup
broker_address = "127.0.0.1"
broker_port = 1883
mqtt_client = mqtt.Client()
sensor_data = {}


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("havpi/temperature")
        client.subscribe("havpi/steps")
        client.subscribe("havpi/pressure")
    else:
        print(f"Failed to connect, return code {rc}\n")


def on_message(client, userdata, message):
    global sensor_data
    sensor_data["data1"] = message.payload.decode()
    print("Message received: ", sensor_data["data1"])


mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

def subscribe_loop():
    mqtt_client.connect(broker_address, port=broker_port)
    mqtt_client.loop_forever()
    
def test():
    #while True:
    print("Testingggg")
    
#if __name__ == "__main__":
    # Start MQTT Client
mqtt_thread = threading.Thread(target=subscribe_loop)
mqtt_thread.start()

test_thread = threading.Thread(target=test)
test_thread.start()

