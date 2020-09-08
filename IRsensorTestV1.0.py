#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 3/22/2020     Ron King    initial dev

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import InfraredSensor

p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p1.set_device = 'lego-ev3-ir'


# Connect infrared to any sensor port
ir = InfraredSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

irProxVal = 0
prev_irProxVal = 0
irDistVal = 0
prev_irDistVal = 0
irHeadVal = 0
prev_irHeadVal = 0


print("#######################################################################")
print("################  starting in proximity mode  #########################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 15):
    irProxVal = ir.proximity
    if irProxVal != prev_irProxVal:
        print("irProxVal = ", irProxVal)
        prev_irProxVal = irProxVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("#################  switching to seeker mode  ##########################")
print("#######################################################################")

startTime = time.time()

irDistVal = 0
prev_irDistVal = 0
irHeadVal = 0
prev_irHeadVal = 0


while (time.time() - startTime < 99):
    irDistVal = ir.distance()
    irHeadVal = ir.heading()
    if (irDistVal != prev_irDistVal) or (irHeadVal != prev_irHeadVal):
            print("irDistVal, irHeadVal ", irDistVal, irHeadVal)
            prev_irDistVal = irDistVal
            prev_irHeadVal = irHeadVal
    time.sleep (0.2) # Give the CPU a rest

print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
