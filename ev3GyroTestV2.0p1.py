#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 5/10/2021     Ron King    initial dev
#
# 5/31/2021     Ron King    - work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
#

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor


p1 = LegoPort(INPUT_1)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'ev3-uart'

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
p1.set_device = 'lego-ev3-gyro'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 3
gyro = GyroSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)


print("#######################################################################")
print("#############   starting in gyro angle mode   #########################")
print("#######################################################################")

#gyro.mode = 'GYRO-ANG'

gyro.mode = 'GYRO-FAS'

gyroVal = 0
prev_gyroVal = -1

time.sleep(2)     # give some time to stabilize gyro before reading values

startTime = time.time()

badValCount = 0

while (1):
    gyroVal = gyro.angle
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if gyroVal == 0 and abs(gyroVal - prev_gyroVal) > 1:
        badValCount += 1
        print("BAD gyroVal = ", gyroVal, "badValCount = ", badValCount, "  prev_gyroVal (good value) = ", prev_gyroVal)   # print for testing; comment for more 'normal' use
        gyroVal = prev_gyroVal
    if gyroVal != prev_gyroVal:
        print("gyroVal = ", gyroVal)
        prev_gyroVal = gyroVal
    time.sleep (0.01) # Give the CPU a rest

