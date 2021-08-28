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

startTime = time.time()

badValCount = 0

while (1):
    usDistCmVal = us.distance_centimeters
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if usDistCmVal == 0 and abs(usDistCmVal - prev_usDistCmVal) > 1:
        badValCount += 1
        print("BAD usDistCmVal = ", usDistCmVal, "badValCount = ", badValCount, "  prev_usDistCmVal (good value) = ", prev_usDistCmVal)   # print for testing; comment for more 'normal' use
        usDistCmVal = prev_usDistCmVal
    if usDistCmVal != prev_usDistCmVal:
        print("usDistCmVal = ", usDistCmVal)
        prev_usDistCmVal = usDistCmVal
    time.sleep (0.05) # Give the CPU a rest

