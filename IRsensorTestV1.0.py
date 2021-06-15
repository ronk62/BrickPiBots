#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 3/22/2020     Ron King    initial dev
# 6/14/2020     Ron King    changed to input port 3 with no time limit, IR-PROX mode only
#                           added work-around for bogus intermittent readings of '0'
#

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import InfraredSensor

p3 = LegoPort(INPUT_3)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p3.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p3.set_device = 'lego-ev3-ir'


# Connect infrared to any sensor port
ir = InfraredSensor(INPUT_3)

# allow for some time to load the new drivers
time.sleep(0.5)

ir.mode = 'IR-PROX'
irProxVal = 100
prev_irProxVal = 0

badValCount = 0

print("")
print("")

startTime = time.time()

print("startTime =  ", startTime)

print("#######################################################################")
print("################  starting in proximity mode  #########################")
print("#######################################################################")


while (1):
    irProxVal = ir.proximity

    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if irProxVal == 0 and abs(irProxVal - prev_irProxVal) > 1:
        badValCount += 1
        print("BAD irProxVal = ", irProxVal, "badValCount = ", badValCount, "  prev_irProxVal (good value) = ", prev_irProxVal, "elapsed time = ",  time.time() - startTime)   # print for testing; comment for more 'normal' use
        irProxVal = prev_irProxVal

    if irProxVal != prev_irProxVal:
        print("irProxVal = ", irProxVal)
        prev_irProxVal = irProxVal
    time.sleep (0.05) # Give the CPU a rest
