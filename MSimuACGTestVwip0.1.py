#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 4/15/2020     Ron King    initial dev

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import Sensor


p1 = LegoPort(INPUT_1)
# 'serial0-0:S1'
# spi0.1:S1:i2c1
# spi0.1:S1:i2c17
# spi0.1:S1:i2c34
# p1 = LegoPort(address='serial0-0:S1:i2c22')

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device """

# set device name and i2c address (in hex)
p1.set_device = 'ms-absolute-imu 0x11'

# allow for some time for setup
time.sleep(0.5)

# Connect imu to sensor port 1
# imu = Sensor(INPUT_1)
# imu = Sensor(address='serial0-0:S1:i2c1')
# imu = Sensor(address='serial0-0:S1:i2c17')
# imu = Sensor(address='serial0-0:S1:i2c34')
# imu = Sensor(address='serial0-0:S1:i2c22')
# imu = Sensor('serial0-0:S1:i2c11')
imu = Sensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)


# Connect imu to sensor port 1
# imu = Sensor(INPUT_1)
# imu = Sensor(address='serial0-0:S1:i2c1')

# allow for some time to load the new drivers
# time.sleep(0.5)


# p1.set_device = 'ms-absolute-imu'
# time.sleep(0.5)


print("#######################################################################")
print("################   starting in compass mode   #########################")
print("#######################################################################")

imu.mode = 'COMPASS'

compassVal = 0
prev_compassVal = 0

startTime = time.time()

while (time.time() - startTime < 55):
    compassVal = imu.value(0)
    if compassVal != prev_compassVal:
        print("compassVal = ", compassVal)
        prev_compassVal = compassVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("#################   switching to GYRO mode   ##########################")
print("#######################################################################")

imu.mode = 'GYRO'

gyroVal = 0
prev_gyroVal = 0

startTime = time.time()

while (time.time() - startTime < 55):
    gyroVal = imu.value(0)
    if gyroVal != prev_gyroVal:
        print("gyroVal = ", gyroVal)
        prev_gyroVal = gyroVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
