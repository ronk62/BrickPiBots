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
from ev3dev2.sensor.lego import LightSensor

p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'nxt-analog'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html
p1.set_device = 'lego-nxt-light'


# Connect infrared to any sensor port
ls = LightSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

lsAmbVal = 0
prev_lsAmbVal = 0

lsReflVal = 0
prev_lsReflVal = 0

print("#######################################################################")
print("#################  light sensor ambient mode  #########################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 55):
    lsAmbVal = ls.ambient_light_intensity
    if lsAmbVal != prev_lsAmbVal:
        print("lsAmbVal = ", lsAmbVal)
        prev_lsAmbVal = lsAmbVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("###############  switching to reflected mode  #########################")
print("#######################################################################")

startTime = time.time()

lsReflVal = 0
prev_lsReflVal = 0


while (time.time() - startTime < 55):
    lsReflVal = ls.reflected_light_intensity
    if (lsReflVal != prev_lsReflVal):
            print("lsReflVal = ", lsReflVal)
            prev_lsReflVal = lsReflVal
    time.sleep (0.2) # Give the CPU a rest

print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
