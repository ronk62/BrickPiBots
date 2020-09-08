#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 4/12/2020     Ron King    initial dev

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor

p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'nxt-analog'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html
p1.set_device = 'lego-nxt-touch'


# Connect infrared to any sensor port
ts = TouchSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

tsVal = False
prev_tsVal = False

print("#######################################################################")
print("#######  touch sensor 'is_pressed' True(1)/False(0)  ##################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 55):
    tsVal = ts.is_pressed
    if tsVal != prev_tsVal:
        print("tsVal (is pressed) = ", tsVal)
        prev_tsVal = tsVal
    time.sleep (0.05) # Give the CPU a rest


print("Additional methods are available (wait_for_pressed, wait_for_released, wait_for_bump)")

print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
