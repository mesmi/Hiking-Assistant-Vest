import time
import board
import numpy as np
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX, Rate, AccelRange, GyroRange
import logging
from scipy.linalg import block_diag
import bme680
import pandas as pd
# Constants
ACCEL_THRESHOLD = 1.5  # Adjust based on sensitivity baseline 2.5
GYRO_THRESHOLD = 0.5   # Adjust based on sensitivity bawseline 1
STEP_THRESHOLD = 0.5   # Threshold for detecting a step
FALL_PITCH_THRESHOLD = 45.0  # Threshold for pitch angle during a fall base 45
FALL_ROLL_THRESHOLD = 45.0   # Threshold for roll angle during a fall base 45

# Kalman filter parameters
Q_accel = 0.001  # Process noise covariance for accelerometer data
Q_gyro = 0.001   # Process noise covariance for gyroscope data
R = 0.01         # Measurement noise covariance

# Initialize logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] - %(message)s")

# Initialize the BME680 sensor (configure it once)
try:
    sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
except (RuntimeError, IOError):
    sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)
sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
sensor.set_gas_heater_temperature(320)
sensor.set_gas_heater_duration(150)
sensor.select_gas_heater_profile(0)

# Initialize the LSM6DSOX sensor
i2c = board.I2C()
sensor2 = LSM6DSOX(i2c)
sensor2.accelerometer_range = AccelRange.RANGE_2G
sensor2.accelerometer_data_rate = Rate.RATE_104_HZ  # Increased data rate for more accuracy
sensor2.gyro_data_rate = Rate.RATE_104_HZ  # Increased data rate for more accuracy
sensor2.pedometer_enable = True

# Function to detect a fall based on acceleration and gyroscope data
def detect_fall(acceleration, gyro, pitch_angle, roll_angle):
    # Calculate acceleration and gyro magnitudes
    accel_magnitude = np.sqrt(np.sum(np.square(acceleration)))
    gyro_magnitude = np.sqrt(np.sum(np.square(gyro)))

    # Check if both acceleration and gyro magnitudes exceed thresholds
    if accel_magnitude > ACCEL_THRESHOLD and gyro_magnitude > GYRO_THRESHOLD:
        # Check if pitch and roll angles exceed thresholds during a fall
        if abs(pitch_angle) > FALL_PITCH_THRESHOLD or abs(roll_angle) > FALL_ROLL_THRESHOLD:
            return True
    return False

# Function to count steps using a custom algorithm
def count_step(acceleration_y, last_accel_y, step_count):
    if acceleration_y > STEP_THRESHOLD and last_accel_y <= STEP_THRESHOLD:
        step_count += 1
    return step_count, acceleration_y

# Function to calculate altitude from pressure and temperature
def calculate_altitude(pressure, temperature_celsius):
    P0 = 1013.25
    T0 = temperature_celsius + 273.15
    altitude = (T0 / 0.05) * ((P0 / pressure) ** (1 / 5.257) - 1)
    return altitude

# Function to log data
def log_data(data):
    logging.info(data)

# Kalman filter functions (same as before)

def main():
    print("Press Ctrl+C to exit!")
    last_accel_y = 0
    step_count = 0

    # Kalman filter initialization (same as before)

    try:
        while True:
            sensor_data = {
                "Timestamp": time.time(),
                "Steps": sensor2.pedometer_steps,
                "Acceleration_X": sensor2.acceleration[0],
                "Acceleration_Y": sensor2.acceleration[1],
                "Acceleration_Z": sensor2.acceleration[2],
                "Gyro_X": sensor2.gyro[0],
                "Gyro_Y": sensor2.gyro[1],
                "Gyro_Z": sensor2.gyro[2],
            }

            try:
                if sensor.get_sensor_data():
                    sensor_data.update({
                        "Temperature": sensor.data.temperature,
                        "Pressure": sensor.data.pressure,
                        "Humidity": sensor.data.humidity,
                        "GasResistance": sensor.data.gas_resistance if sensor.data.heat_stable else None
                    })
                    altitude = calculate_altitude(sensor_data["Pressure"], sensor_data["Temperature"])
                    sensor_data["Altitude"] = altitude

                # Kalman filter prediction and update steps (same as before)

                # Extract pitch and roll angles from the Kalman filter state vector
                pitch_angle, roll_angle = np.degrees(X)

                sensor_data["PitchAngle"] = pitch_angle
                sensor_data["RollAngle"] = roll_angle

                # Improved fall detection using pitch and roll angles
                if detect_fall(sensor2.acceleration, sensor2.gyro, pitch_angle, roll_angle):
                    logging.warning("Fall detected!")

            except Exception as e:
                logging.error(f"Error retrieving data from BME680 sensor: {e}")

            step_count, last_accel_y = count_step(sensor2.acceleration[1], last_accel_y, step_count)
            sensor_data["CustomStepCount"] = step_count
            sensor_data["Altitude"] = altitude

            log_data(sensor_data)

            time.sleep(1)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()

