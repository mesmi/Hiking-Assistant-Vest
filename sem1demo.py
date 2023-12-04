import bme680
import paho.mqtt.client as mqtt
import json
import time
import board
import pandas as pd
import numpy as np
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Failed to connect, return code %d\n", rc)

def on_publish(client, userdata, result):
    print("Data published to MQTT Broker")

# MQTT settings
broker_address = "127.0.0.1"
broker_port = 1883
# username = "YOUR_MQTT_USERNAME"  # if applicable
# password = "YOUR_MQTT_PASSWORD"  # if applicable

# Initialize MQTT Client
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_publish = on_publish
# mqtt_client.username_pw_set(username, password)  # Remove if not needed
mqtt_client.connect(broker_address, broker_port)

# Constants
ACCEL_THRESHOLD = 1.0  # Acceleration threshold for fall detection (in m/s^2) 3
GYRO_THRESHOLD = 1.0   # Gyro threshold for fall detection (in radians/s) 5
STEP_THRESHOLD = 0.5   # Threshold for detecting a step
altitude=0
print("Press Ctrl+C to exit!")
data_to_save = []  # List for data
last_accel_y = 0
step_count = 0

# Initialize BME680 sensor
try:
    sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
except (RuntimeError, IOError):
    sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

# BME680 sensor configuration
sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)
sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
sensor.set_gas_heater_temperature(320)
sensor.set_gas_heater_duration(150)
sensor.select_gas_heater_profile(0)

# Initialize LSM6DSOX sensor
i2c = board.I2C()
sensor2 = LSM6DSOX(i2c)
sensor2.accelerometer_range = AccelRange.RANGE_2G
sensor2.accelerometer_data_rate = Rate.RATE_26_HZ
sensor2.gyro_data_rate = Rate.RATE_26_HZ
sensor2.pedometer_enable = True

def detect_fall(acceleration, gyro):
    # Fall detection algorithm
    accel_magnitude = np.sqrt(np.sum(np.square(acceleration)))
    gyro_magnitude = np.sqrt(np.sum(np.square(gyro)))
    return accel_magnitude > ACCEL_THRESHOLD and gyro_magnitude > GYRO_THRESHOLD

def count_step(acceleration_y, last_accel_y, step_count):
    # Pedometer algorithm
    if acceleration_y > STEP_THRESHOLD and last_accel_y <= STEP_THRESHOLD:
        step_count += 1
    return step_count, acceleration_y

def calculate_altitude(pressure, temperature_celsius):
    P0 = 1013.25  # Sea level standard atmospheric pressure in hPa
    T0 = temperature_celsius + 273.15  # Convert temperature to Kelvin
    altitude = (T0 / 0.05) * ((P0 / pressure) ** (1 / 5.257) - 1)
    return altitude

try:
    while True:
        # Initialize sensor_data dictionaries
        sensor_data = {}
        sensor2_data = {}
        
       
            

        # Pedometer (custom algorithm)
        step_count, last_accel_y = count_step(sensor2.acceleration[1], last_accel_y, step_count)
        print("Step Count: ", step_count)

        # Collect data from LSM6DSOX sensor
        sensor2_data = {
            "Steps": sensor2.pedometer_steps,
            "Acceleration_X": sensor2.acceleration[0],
            "Acceleration_Y": sensor2.acceleration[1],
            "Acceleration_Z": sensor2.acceleration[2],
            "Gyro_X": sensor2.gyro[0],
            "Gyro_Y": sensor2.gyro[1],
            "Gyro_Z": sensor2.gyro[2],
            
        }

        # Try to collect data from BME680 sensor
        try:
            if sensor.get_sensor_data():
                sensor_data = {
                    "Temperature": sensor.data.temperature,
                    "Pressure": sensor.data.pressure,
                    "Humidity": sensor.data.humidity,
                    "GasResistance": sensor.data.gas_resistance if sensor.data.heat_stable else None
                }
                altitude = calculate_altitude(sensor_data["Pressure"], sensor_data["Temperature"])
                sensor_data["Altitude"] = altitude
                
        except Exception as e:
            print(f"Error retrieving data from BME680 sensor: {e}")



        # Add the collected data to the list
    
        combined_data = {**sensor2_data, **sensor_data, "CustomStepCount": step_count, "Altitude":altitude}
        data_json = json.dumps(combined_data)
        mqtt_client.publish("havpi/sensordata", data_json)
        data_to_save.append(combined_data)
        #data_json = json.dumps(combined_data)

        # Publish data to MQTT topic
        #mqtt_client.publish("havpi/sensordata", data_json)
        
        if detect_fall(sensor2.acceleration, sensor2.gyro):
            print("Fall detected!") 

        # Print steps and acceleration for monitoring
        print("Sensor2 Steps: ", sensor2.pedometer_steps)
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor2.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor2.gyro))
        print("Altitude: {:.2f} meters".format(altitude))
        print("")

        # Save data to a CSV file every minute
        if len(data_to_save) >= 60:
            df = pd.DataFrame(data_to_save)
            df.to_csv('sensor_data.csv', index=False)
            data_to_save.clear()

        time.sleep(1)

except KeyboardInterrupt:
    # Convert any remaining data to DataFrame and save
    if data_to_save:
        df = pd.DataFrame(data_to_save)