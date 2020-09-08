#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 4/16/2020     Ron King    initial dev

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import Sensor
# import ev3dev2.sensor

p1 = LegoPort(INPUT_1)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'nxt-i2c'

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device 

dmesg -w

"""

# set device name and i2c address (in hex)
p1.set_device = 'ht-nxt-compass 0x01'


# allow for some time to load the new drivers
time.sleep(0.5)

# Connect sensor to sensor port 1
compass = Sensor(INPUT_1)


# allow for some time to load the new drivers
# time.sleep(0.5)

compassVal = 0
prev_compassVal = 0


print("#######################################################################")
print("####################     read compass     #############################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 55):
    compassVal = compass.value(0)
    if compassVal != prev_compassVal:
        print("compassVal = ", compassVal)
        prev_compassVal = compassVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
