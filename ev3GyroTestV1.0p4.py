#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 5/10/2021     Ron King    initial dev

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import Sensor


p4 = LegoPort(INPUT_4)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p4.mode = 'ev3-uart'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device 

dmesg -w

"""

# set device name and i2c address (in hex)
p4.set_device = 'lego-ev3-gyro'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 1
gyro = Sensor(INPUT_4)

# allow for some time to load the new drivers
time.sleep(0.5)


print("#######################################################################")
print("################    starting in gyro mode     #########################")
print("#######################################################################")

gyro.mode = 'GYRO-ANG'

gyroVal = 0
prev_gyroVal = 0

startTime = time.time()

while (time.time() - startTime < 55):
    gyroVal = gyro.value(0)
    if gyroVal != prev_gyroVal:
        print("gyroVal = ", gyroVal)
        prev_gyroVal = gyroVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("#################   switching to GYRO-RATE mode   #####################")
print("#######################################################################")

gyro.mode = 'GYRO-RATE'

gyroVal = 0
prev_gyroVal = 0

startTime = time.time()

while (time.time() - startTime < 55):
    gyroVal = gyro.value(0)
    if gyroVal != prev_gyroVal:
        print("gyroVal = ", gyroVal)
        prev_gyroVal = gyroVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
