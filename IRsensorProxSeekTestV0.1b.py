#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 3/22/2020     Ron King    initial dev
# 5/3/2020      Ron King    modify to use Proximity and Beacon/Seek (distance, heading) mode simultaneously
#                           discovered that the IR beacon signal interfers with the proximity mode readings

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
time.sleep(3)

irProxVal = 0
prev_irProxVal = 0
irDistVal = 0
prev_irDistVal = 0
irHeadVal = 0
prev_irHeadVal = 0

print("")

print("#######################################################################")
print("##########################   starting   ###############################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 600):
    irHeadVal = ir.heading()
    # time.sleep(0.2)
    irDistVal = ir.distance()
    if irDistVal is None:
        irDistVal = 100

    #  Uncomment this code to default to beacon and use proximity only when beacon not found
    #  Keep in mind that collisions could occur since proximity is blind if beacon is ANYWHERE in range
    #  if (irDistVal < 0) or (irDistVal > 99):
    #     time.sleep(0.2)
    #     print("Using proximity mode")
    #     irProxVal = ir.proximity
    
    #  Comment the below 2 lines of code if you uncomment the 'if' block above
    time.sleep(0.2)
    irProxVal = ir.proximity

    if (irProxVal != prev_irProxVal) or (irDistVal != prev_irDistVal) or (irHeadVal != prev_irHeadVal):
            print("irProxVal, irDistVal, irHeadVal  ", irProxVal, irDistVal, irHeadVal)
            prev_irProxVal = irProxVal
            prev_irDistVal = irDistVal
            prev_irHeadVal = irHeadVal
    
    time.sleep (0.2) # Give the CPU a rest


print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
