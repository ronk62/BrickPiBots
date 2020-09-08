#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 4/11/2020     Ron King    initial dev

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import UltrasonicSensor


###  Ref: https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p1.set_device = 'lego-ev3-us'


# Connect infrared to any sensor port
us = UltrasonicSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

usDistCmVal = 0
prev_usDistCmVal = 0

usDistInVal = 0
prev_usDistInVal = 0

print("#######################################################################")
print("###############  us.distance_centimeters mode  ########################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 55):
    usDistCmVal = us.distance_centimeters
    if usDistCmVal != prev_usDistCmVal:
        print("usDistCmVal = ", usDistCmVal)
        prev_usDistCmVal = usDistCmVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("#################  switching to inches mode  ##########################")
print("#######################################################################")

startTime = time.time()

usDistInVal = 0
prev_usDistInVal = 0


while (time.time() - startTime < 55):
    usDistInVal = us.distance_inches
    if (usDistInVal != prev_usDistInVal):
            print("usDistInVal ", usDistInVal)
            prev_usDistInVal = usDistInVal
    time.sleep (0.2) # Give the CPU a rest

print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
