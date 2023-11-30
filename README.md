# Hiking-Assistant-Vest
A capstone project as part of the Computer Engineering undergraduate degree at University of Ottawa. 

## Authors
- Baris Aydin
- Brandon Rodrigues 
- Kemal Kilic 
- Mobina Esmi 
- Rohin Sharma


## Project Description 
A wearable IoT vest that is designed to enhance the hiking experience for outdoor nature  adventurists on their hiking and camping trails by providing an information hub for essential data that is automatically and manually recorded, for the purposes of: safety regarding the hiker’s personal and surrounding metrics such as altitude above sea level, atmospheric pressure, air quality, oxygen level, fall-detection, temperature/humidity, and step count; as well as for logging all data related to the hiking experience, allowing for saving and/or sharing hiking trail paths and creating logs of geotagged information that is saved automatically by the system and manually by the user; through an interconnected hardware & software system that interacts with a mobile app and the internet that brings together all functionality proposed as part of the vest & app system. 

## Project Statement
The purpose of this project is to provide an all-in-one solution to many of the problems that hikers and campers face while also enhancing their experience when doing their favorite hobbies. The project consists of two parts: the vest which contains the hardware and the phone application which can be used to interface with the device. The vest focuses on two major aspects: safety and experience enhancement. Primarily, users are able to use the vest system to avoid or be alerted of emergency situations. Some examples of features include: detecting falls, checking oxygen levels and other important atmospheric metrics in the current environment, sounding an alarm for emergencies, and showing your location. On the other hand, users are able to use the device for more fun activities such as tracking statistics during a hike, sharing specific trail paths with friends and so on. With these features, the goal of the project is to increase the safety of the user while also making their experience outdoors more enjoyable and informative. 

## Technologies Involved
### Hardware
The hardware components used in this project include the Raspberry Pi 4 2 GB, Pimoroni BME680 sensor, Adafruit LSM6DSOX 6 DoF Accelerometer and Gyroscope and a 20000mah battery bank. 

#### Raspberry Pi 4 2GB specifications:
*• Quad-core 64-bit ARM-Cortex A72 running at 1.5 GHz
• 2 Gigabyte LPDDR4 RAM
• 802.11 b/g/n/ac Wireless LAN
• Bluetooth 5.0 with BLE 
• 1x SD Card 
• 2x micro HDMI ports supporting dual displays up to 4Kp60 resolution 
• 1x Gigabit Ethernet port (supports PoE with add-on PoE HAT)
• 28x user GPIO supporting various interface options
• idle= 5V 600 mA, stress= 5 V 1250 mA*

#### Pimoroni BME680 specifications:
*• Bosch BME680 temperature, pressure, humidity, altitude, air quality sensor (datasheet)
• I2C interface, with address select via ADDR solder bridge (0x76 or 0x77)
• 3.3 V or 5 V compatible
• Reverse polarity protection
• 3.7 µA at 1 Hz humidity, pressure and temperature* 

#### Adafruit LSM6DSOX specifications:
*• 6-DOF
• Temperature range of -40 to +105 °C
• Accelerometers have a range of: ±2/±4/±8/±16 g at 1.6 Hz to 6.7KHz update rate
• Gyroscopes: ±125/±250/±500/±1000/±2000 dps
• 3 V or 5 V power/logic
• 0.55 mA*

#### Battery bank specifications:
*• 20000 mAh battery capacity
• 65 W maximum output*

