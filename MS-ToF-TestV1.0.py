#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 11/11/2022    Ron King    initial dev; note that this sensor is supported in ev3dev-buster

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import Sensor


p1 = LegoPort(INPUT_1)

# https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-buster/brickpi3.html#input-ports
p1.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-buster/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device 

dmesg -w

"""

# set device name and i2c address (in hex)
p1.set_device = 'ms-tof 0x01'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 1
ToF = Sensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)


ToF.mode = 'DIST'   # distance in mm

distmm = 0
prev_distmm = 0


while True:
    distmm = ToF.value(0)
    if distmm != prev_distmm:
        print("distmm = ", distmm)
        prev_distmm = distmm
    time.sleep (0.05) # Give the CPU a rest

