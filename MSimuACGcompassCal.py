#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 4/26/2020     Ron King    initial dev

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.sensor import INPUT_2
from ev3dev2.sensor.lego import Sensor

doExit = False
tty.setcbreak(sys.stdin)

p2 = LegoPort(INPUT_2)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p2.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device """

# set device name and i2c address (in hex)
p2.set_device = 'ms-absolute-imu 0x11'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 1
imu = Sensor(INPUT_2)

# allow for some time to load the new drivers
time.sleep(0.5)


print("#######################################################################")
print("###############   starting compass calibration   ######################")
print("#######################################################################")

print("")
print("  Sensor PORT 2  ")
print("")

imu.mode = 'COMPASS'
time.sleep(.5)

imu.command = 'BEGIN-COMP-CAL'  # send the command to begin compass calibration

print ("Hit x to end calibration and exit...")

while (doExit == False):
    x = ord(sys.stdin.read(1))
    if x == 120: # x key pushed
        print("x key pushed - command accepted")
        imu.command = 'END-COMP-CAL'  # send the end cal command
        time.sleep(5)
        print("calibration cycle complete. Exiting.")
        break
    time.sleep(.2)