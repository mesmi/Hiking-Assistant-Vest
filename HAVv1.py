import time
import smbus
import board
import adafruit_bme680
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

bus3=smbus.SMBus(3) #Aditional 12c bus, configured in config.txt
bus4=smbus.SMBus(4)

i2c = board.I2C() 

bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)

sensor = LSM6DSOX(i2c)

bme680.sea_level_pressure = 1013.25

temperature_offset = -5


while True:

    print("\nTemperature: %0.1f C" % (bme680.temperature + temperature_offset))
    print("Gas: %d ohm" % bme680.gas)
    print("Humidity: %0.1f %%" % bme680.relative_humidity)
    print("Pressure: %0.3f hPa" % bme680.pressure)
    print("Altitude = %0.2f meters" % bme680.altitude)
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
    print("")

    time.sleep(1)
    
#HELPFUL WEBSITES 
# https://forums.raspberrypi.com/viewtopic.php?t=271200  
# https://github.com/JJSlabbert/Raspberry_PI_i2C_conficts
# https://www.reddit.com/r/raspberry_pi/comments/ydelqb/how_to_use_two_i2c_devices_on_rpi4b/
 
# EXAMPLES AND DIFFERENT WAYS TO IMPLEMENT 
 
# IMPORTANT: Add the following 2 lines to /boot/config.txt to generate aditional i2c busses 3 and 4
# dtoverlay=i2c-gpio,bus=3,i2c_gpio_delay_us=1
# dtoverlay=i2c-gpio,bus=4,i2c_gpio_delay_us=1,i2c_gpio_sda=17,i2c_gpio_scl=27

''' 
bus3=smbus.SMBus(3) #Aditional 12c bus, configured in config.txt
bus4=smbus.SMBus(4) #Aditional 12c bus, configured in config.txt
address=0x25
scale_factor=255/60 #255/60 for SDP810-500 PA and 255/240 for SDP810-125 PA

bus3.write_i2c_block_data(address, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
bus4.write_i2c_block_data(address, 0x3F, [0xF9]) #Stop any cont measurement of the sensor
time.sleep(0.1)
bus3.write_i2c_block_data(address, 0x36, [0X03]) #Start Continuous Measurement 
bus4.write_i2c_block_data(address, 0x36, [0X03]) #Start Continuous Measurement
time.sleep(0.1)

while True:
    time.sleep(0.05)
    #Reading Sensor on i2c bus 3
    reading3=bus3.read_i2c_block_data(address,0,9)
    pressure_value3=reading3[0]+float(reading3[1])/255
    if pressure_value3>=0 and pressure_value3<128:
        diffirential_pressure3=pressure_value3*scale_factor#scale factor adjustment
    elif pressure_value3>128 and pressure_value3<=256:
        diffirential_pressure3=-(256-pressure_value3)*scale_factor #scale factor adjustment
    elif pressure_value3==128:
        diffirential_pressure3=99999999 #Out of range
    print("Diffirential Pressure 3: "+str(diffirential_pressure3)+" PA")


    #Reading Sensor on i2c bus 4
    reading4=bus4.read_i2c_block_data(address,0,9)
    pressure_value4=reading4[0]+float(reading4[1])/255
    if pressure_value4>=0 and pressure_value4<128:
        diffirential_pressure4=pressure_value4*scale_factor #scale factor adjustment
    elif pressure_value4>128 and pressure_value4<=256:
        diffirential_pressure4=-(256-pressure_value4)*scale_factor #scale factor adjustment
    elif pressure_value4==128:
        diffirential_pressure4=99999999 #Out of range
    print("Diffirential Pressure 4: "+str(diffirential_pressure4)+" PA"+"\n")

 /////////////////////////////////////////////////////////////////////////////////////////////////////////
#I modified code from controle everything 


import smbus
import time

# Get I2C bus
bus3 = smbus.SMBus(3)
bus4 = smbus.SMBus(4)
delay=0.05

while True:
    #Reading Data from i2c bus3 3
    b1 = bus3.read_i2c_block_data(0x76, 0x88, 24)

    # Convert the data
    # Temp coefficents
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536

    # Pressure coefficents
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536

    # BMP280 address, 0x76(118)
    # Select Control measurement register, 0xF4(244)
    #		0x27(39)	Pressure and Temperature Oversampling rate = 1
    #					Normal mode
    bus3.write_byte_data(0x76, 0xF4, 0x27)
    # BMP280 address, 0x76(118)
    # Select Configuration register, 0xF5(245)
    #		0xA0(00)	Stand_by time = 1000 ms
    bus3.write_byte_data(0x76, 0xF5, 0xA0)

    time.sleep(delay)

    # BMP280 address, 0x76(118)
    # Read data back from 0xF7(247), 8 bytes
    # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
    # Temperature xLSB, Humidity MSB, Humidity LSB
    data = bus3.read_i2c_block_data(0x76, 0xF7, 8)

    # Convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

    # Temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
    var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
    t_fine = (var1 + var2)
    cTemp = (var1 + var2) / 5120.0
    fTemp = cTemp * 1.8 + 32

    # Pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (dig_P6) / 32768.0
    var2 = var2 + var1 * (dig_P5) * 2.0
    var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
    var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (dig_P1)
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (dig_P9) * p * p / 2147483648.0
    var2 = p * (dig_P8) / 32768.0
    pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100

    # Output data to screen
    print("Reading sensor on i2c bus 3")
    print "Temperature in Celsius : %.2f C" %cTemp
    print "Temperature in Fahrenheit : %.2f F" %fTemp
    print "Pressure : %.2f hPa " %pressure+"\n"


    #Reading BMP280 on i2c bus3 4
        # BMP280 address, 0x76(118)
    # Read data back from 0x88(136), 24 bytes
    b1 = bus4.read_i2c_block_data(0x76, 0x88, 24)

    # Convert the data
    # Temp coefficents
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536

    # Pressure coefficents
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536

    # BMP280 address, 0x76(118)
    # Select Control measurement register, 0xF4(244)
    #		0x27(39)	Pressure and Temperature Oversampling rate = 1
    #					Normal mode
    bus4.write_byte_data(0x76, 0xF4, 0x27)
    # BMP280 address, 0x76(118)
    # Select Configuration register, 0xF5(245)
    #		0xA0(00)	Stand_by time = 1000 ms
    bus4.write_byte_data(0x76, 0xF5, 0xA0)

    time.sleep(delay)

    # BMP280 address, 0x76(118)
    # Read data back from 0xF7(247), 8 bytes
    # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
    # Temperature xLSB, Humidity MSB, Humidity LSB
    data = bus4.read_i2c_block_data(0x76, 0xF7, 8)

    # Convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

    # Temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
    var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
    t_fine = (var1 + var2)
    cTemp = (var1 + var2) / 5120.0
    fTemp = cTemp * 1.8 + 32

    # Pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (dig_P6) / 32768.0
    var2 = var2 + var1 * (dig_P5) * 2.0
    var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
    var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (dig_P1)
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (dig_P9) * p * p / 2147483648.0
    var2 = p * (dig_P8) / 32768.0
    pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100

    # Output data to screen
    print("Reading sensor on i2c bus 4")
    print "Temperature in Celsius : %.2f C" %cTemp
    print "Temperature in Fahrenheit : %.2f F" %fTemp
    print "Pressure : %.2f hPa " %pressure +"\n"
    '''