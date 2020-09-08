#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 4/15/2020     Ron King    initial dev

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import Sensor


p1 = LegoPort(INPUT_1)
# 'serial0-0:S1'
# spi0.1:S1:i2c1
# spi0.1:S1:i2c17
# spi0.1:S1:i2c34
# p1 = LegoPort(address='serial0-0:S1:i2c22')

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p1.set_device = 'nxt-i2c-sensor'

# allow for some time for mode to setup
time.sleep(0.5)

# Connect imu to sensor port 1
# imu = Sensor(INPUT_1)
# imu = Sensor(address='serial0-0:S1:i2c1')
# imu = Sensor(address='serial0-0:S1:i2c17')
# imu = Sensor(address='serial0-0:S1:i2c34')
# imu = Sensor(address='serial0-0:S1:i2c22')
# imu = Sensor('serial0-0:S1:i2c11')
imu = Sensor('serial0-0:S1:i2c17')

# allow for some time to load the new drivers
time.sleep(0.5)


# Connect imu to sensor port 1
# imu = Sensor(INPUT_1)
imu = Sensor(address='serial0-0:S1:i2c1')

# allow for some time to load the new drivers
time.sleep(0.5)


# p1.set_device = 'ms-absolute-imu'
# time.sleep(0.5)

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

while (time.time() - startTime < 55):
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


while (time.time() - startTime < 55):
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
