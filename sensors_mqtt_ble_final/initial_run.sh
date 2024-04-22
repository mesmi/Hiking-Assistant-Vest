#!/bin/sh
sleep 5
echo 'Starting sensor data collection!'
python3 '/home/havpi/Documents/demosem_5.py' &
echo 'Successfully started demosem_5.py'
echo 'Starting BLE peripheral setup'
python3 '/home/havpi/Documents/dbus_test2.py' &
echo 'Successfully started dbus_test2.py'
