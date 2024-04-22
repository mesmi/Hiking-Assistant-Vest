import time
import board
import numpy as np
import logging
import json
import math
import paho.mqtt.client as mqtt
import bme680
import threading
import pygame
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
 
# Initialize logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] - %(message)s")
 
# Initialize pygame for sound
pygame.init()
fall_sound = pygame.mixer.Sound('alarm.wav')  # Update the path to your sound file
 
# MQTT settings
broker_address = "127.0.0.1"
broker_port = 1883
 
# Constants
ACCEL_THRESHOLD = 10.0
GYRO_THRESHOLD = 0.5
STEP_THRESHOLD = 1.0
FALL_PITCH_THRESHOLD = 45.0
FALL_ROLL_THRESHOLD = 45.0
ALTITUDE_REF_PRESSURE = 1013.25
 
# Kalman filter parameters
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
x_bias = 0.0
y_bias = 0.0
XP_00, XP_01, XP_10, XP_11 = 0.0, 0.0, 0.0, 0.0
YP_00, YP_01, YP_10, YP_11 = 0.0, 0.0, 0.0, 0.0
xf, yf = 0.0, 0.0
dt = 0.05  # Time step
 
# Initialize sensors
def initialize_sensors():
    try:
        bme_sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
    except (RuntimeError, IOError):
        bme_sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)
 
    bme_sensor.set_humidity_oversample(bme680.OS_2X)
    bme_sensor.set_pressure_oversample(bme680.OS_4X)
    bme_sensor.set_temperature_oversample(bme680.OS_8X)
    bme_sensor.set_filter(bme680.FILTER_SIZE_3)
    bme_sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
    bme_sensor.set_gas_heater_temperature(320)
    bme_sensor.set_gas_heater_duration(150)
    bme_sensor.select_gas_heater_profile(0)
 
    lsm6dsox_sensor = LSM6DSOX(board.I2C())
    lsm6dsox_sensor.accelerometer_range = AccelRange.RANGE_16G
    lsm6dsox_sensor.accelerometer_data_rate = Rate.RATE_104_HZ
    lsm6dsox_sensor.gyro_data_rate = Rate.RATE_104_HZ
    lsm6dsox_sensor.pedometer_enable = True
 
    return bme_sensor, lsm6dsox_sensor
 
# MQTT connection callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info("Connected to MQTT Broker!")
    else:
        logging.error("Failed to connect to MQTT Broker, return code %d", rc)
 
def on_publish(client, userdata, result):
    logging.info("Data published to MQTT Broker")
 
# Initialize MQTT client
def initialize_mqtt_client():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.connect(broker_address, broker_port)
    return client
 
# Detect fall based on acceleration and gyroscope data
def detect_fall(acceleration, gyro, pitch_angle, roll_angle):
    global fall_sound
    acceleration = (acceleration[0], acceleration[1], acceleration[2])
    accel_magnitude = np.sqrt(np.sum(np.square(acceleration)))
    gyro_magnitude = np.sqrt(np.sum(np.square(gyro)))
    
    #testing:
    pitch_in_deg = (pitch_angle * 180) / 3.14
    roll_in_deg = (roll_angle * 180) / 3.14 
    print("accel_magnitudde:", accel_magnitude)
    print("gyro_magnitudde:", gyro_magnitude)
    print("pitch_in_deg: ",abs(pitch_in_deg))
    print("roll_in_deg: ", abs(roll_in_deg))
    
 
    if accel_magnitude > ACCEL_THRESHOLD and gyro_magnitude > GYRO_THRESHOLD:
        if abs(pitch_in_deg) > FALL_PITCH_THRESHOLD or abs(roll_in_deg) > FALL_ROLL_THRESHOLD:
            fall_sound.play()
            return True
    return False
 
# Count steps using a custom algorithm
def count_step(acceleration_x, acceleration_y, last_accel_x, last_accel_y, step_count):
    acceleration = (acceleration_x, acceleration_y)
    last_acceleration = (last_accel_x, last_accel_y)
    accel_magnitude = np.sqrt(np.sum(np.square(acceleration)))
    last_accel_magnitude = np.sqrt(np.sum(np.square(last_acceleration)))
 
    if accel_magnitude - last_accel_magnitude > STEP_THRESHOLD:
        step_count += 1
    return step_count, acceleration_x, acceleration_y
 
# Calculate altitude from pressure and temperature
def calculate_altitude(pressure, temperature_celsius):
    T0 = temperature_celsius + 273.15
    altitude = (T0 / 0.05) * ((ALTITUDE_REF_PRESSURE / pressure) ** (1 / 5.257) - 1)
    return altitude
 
 # Function to log data
def log_data(data):
    logging.info(data)
 
# Kalman filter for pitch and roll angles
def kalman_filter(pitch, roll):
    global xf, yf, x_bias, y_bias, XP_00, XP_01, XP_10, XP_11, YP_00, YP_01, YP_10, YP_11
 
    # Kalman filter for X-axis (roll)
    XP_00 = XP_00 + dt * (XP_10 + XP_01 + Q_angle)
    XP_01 = XP_01 - dt * XP_11
    XP_10 = XP_10 - dt * XP_00
    XP_11 = XP_11 + Q_gyro * dt
 
    roll_error = roll - xf
    S_roll = XP_00 + R_angle
    K_roll_0 = XP_00 / S_roll
    K_roll_1 = XP_10 / S_roll
 
    xf = xf + K_roll_0 * roll_error
    x_bias = x_bias + K_roll_1 * roll_error
    XP_00 = XP_00 - K_roll_0 * XP_00
    XP_01 = XP_01 - K_roll_0 * XP_01
    XP_10 = XP_10 - K_roll_1 * XP_00
    XP_11 = XP_11 - K_roll_1 * XP_01
 
    # Kalman filter for Y-axis (pitch)
    YP_00 = YP_00 + dt * (YP_10 + YP_01 + Q_angle)
    YP_01 = YP_01 - dt * YP_11
    YP_10 = YP_10 - dt * YP_00
    YP_11 = YP_11 + Q_gyro * dt
 
    pitch_error = pitch - yf
    S_pitch = YP_00 + R_angle
    K_pitch_0 = YP_00 / S_pitch
    K_pitch_1 = YP_10 / S_pitch
 
    yf = yf + K_pitch_0 * pitch_error
    y_bias = y_bias + K_pitch_1 * pitch_error
    YP_00 = YP_00 - K_pitch_0 * YP_00
    YP_01 = YP_01 - K_pitch_0 * YP_01
    YP_10 = YP_10 - K_pitch_1 * YP_00
    YP_11 = YP_11 - K_pitch_1 * YP_01
 
    return yf, xf  # Return pitch and roll
 
# Calibration parameters
CALIBRATION_SAMPLES = 500
 
# Global variables for offsets
accel_offset = (0, 0, 0)
gyro_offset = (0, 0, 0)
 
def calibrate_sensors(lsm6dsox_sensor):
    global accel_offset, gyro_offset
    accel_sum = np.array([0.0, 0.0, 0.0])
    gyro_sum = np.array([0.0, 0.0, 0.0])
 
    for _ in range(CALIBRATION_SAMPLES):
        accel_sum += np.array(lsm6dsox_sensor.acceleration)
        gyro_sum += np.array(lsm6dsox_sensor.gyro)
        time.sleep(0.01)  # Small delay between samples
 
    accel_offset = accel_sum / CALIBRATION_SAMPLES
    gyro_offset = gyro_sum / CALIBRATION_SAMPLES
 
    logging.info(f"Calibration complete: Accel Offset = {accel_offset}, Gyro Offset = {gyro_offset}")
 
def main():
    global altitude
    logging.info("Press Ctrl+C to exit!")
    last_accel_y = 0
    last_accel_x = 0
    step_count = 0
 
    bme_sensor, lsm6dsox_sensor = initialize_sensors()
    mqtt_client = initialize_mqtt_client()
 
    #input("Calibrating sensors, please keep the device stationary and press Enter to continue...")
    print("Calibrating sensors, please keep the device stationary")
    calibrate_sensors(lsm6dsox_sensor)
 
    try:
        while True:
            # Subtract offsets from sensor readings
            accel_raw = np.array(lsm6dsox_sensor.acceleration) - accel_offset
            gyro_raw = np.array(lsm6dsox_sensor.gyro) - gyro_offset
 
            sensor_data = {
                "Timestamp": time.time(),
                "Steps": lsm6dsox_sensor.pedometer_steps,
                "Acceleration_X": accel_raw[0],
                "Acceleration_Y": accel_raw[1],
                "Acceleration_Z": accel_raw[2],
                "Gyro_X": gyro_raw[0],
                "Gyro_Y": gyro_raw[1],
                "Gyro_Z": gyro_raw[2],
            }
 
            sensor_data["Temperature"] = bme_sensor.data.temperature
            sensor_data["Pressure"] = bme_sensor.data.pressure
            sensor_data["Humidity"] = bme_sensor.data.humidity
            sensor_data["GasResistance"] = bme_sensor.data.gas_resistance if bme_sensor.data.heat_stable else None
            altitude = calculate_altitude(sensor_data["Pressure"], sensor_data["Temperature"])
            sensor_data["Altitude"] = altitude
            
 
            pitch_angle = math.atan2(accel_raw[1], math.sqrt(accel_raw[0]**2 + accel_raw[2]**2))
            roll_angle = math.atan2(accel_raw[0], math.sqrt(accel_raw[1]**2 + accel_raw[2]**2))
 
            pitch_angle, roll_angle = kalman_filter(pitch_angle, roll_angle)
 
            sensor_data["PitchAngle"] = pitch_angle
            sensor_data["RollAngle"] = roll_angle
 
            if detect_fall(accel_raw, gyro_raw, pitch_angle, roll_angle):
                logging.warning("Fall detected!")
                
            
            
            
 
            step_count, last_accel_x, last_accel_y = count_step(accel_raw[0], accel_raw[1], last_accel_x, last_accel_y, step_count)
            sensor_data["CustomStepCount"] = step_count // 3
 
            log_data(sensor_data)
            
            mqtt_client.publish("havpi/steps", sensor_data["CustomStepCount"])
            mqtt_client.publish("havpi/accel_x", sensor_data["Acceleration_X"])
            mqtt_client.publish("havpi/accel_y", sensor_data["Acceleration_Y"])
            mqtt_client.publish("havpi/accel_z", sensor_data["Acceleration_Z"])
            mqtt_client.publish("havpi/gyro_x", sensor_data["Gyro_X"])
            mqtt_client.publish("havpi/gyro_y", sensor_data["Gyro_Y"])
            mqtt_client.publish("havpi/gyro_z", sensor_data["Gyro_Z"])
            mqtt_client.publish("havpi/temperature", sensor_data["Temperature"])
            mqtt_client.publish("havpi/pressure", sensor_data["Pressure"])
            mqtt_client.publish("havpi/humidity", sensor_data["Humidity"])
            mqtt_client.publish("havpi/gas_resistance", sensor_data["GasResistance"])
            mqtt_client.publish("havpi/altitude", sensor_data["Altitude"])
            mqtt_client.publish("havpi/pitch_angle", sensor_data["PitchAngle"])
            mqtt_client.publish("havpi/roll_angle", sensor_data["RollAngle"])
            #mqtt_client.publish("havpi/sensordata", json.dumps(sensor_data))
 
            time.sleep(0.1)
 
    except KeyboardInterrupt:
        logging.info("Program terminated by user")
 
if __name__ == "__main__":
    main()
