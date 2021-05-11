#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
# Date          Author      Change Notes
# 5/7/2021      Ron King    initial dev, using port 4
# 5/8-10/2021   Ron King    numerous failed attempts to get this code and sensor working
#                           seems to fail at the driver level and cmd line
#                           the port and sensor attributes seem ok in Bricman; only i2c comms seem dead
#                           mixed results using i2ctools, but i have little exp there

# ref. https://github.com/ev3dev/ev3dev-lang-python-demo/blob/stretch/platform/brickpi3-motor-and-sensor.py

import time, tty, sys
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import OUTPUT_A, LargeMotor, SpeedPercent
from ev3dev2.sensor import INPUT_2
from ev3dev2.sensor.lego import Sensor
from ev3dev2.sensor import I2cSensor

print("wait 1")
# allow for some time to stabilze
time.sleep(3)

p2 = LegoPort(INPUT_2)

print("valid port (p2) modes are...", p2.modes)
print("")

print("wait 2")

# allow for some time to stabilze
time.sleep(3)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p2.mode = 'nxt-i2c'   ## this works on most of the other i2c sensors used so far (Ron King, 5/7/2021)

print("wait 3")

# allow for some time to stabilze
time.sleep(3)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device 

dmesg -w

"""

# set device name and i2c address (in hex)
p2.set_device = 'mi-xg1300l 0x01'

print("wait 4")

# allow for some time to load the new drivers
time.sleep(3)

# p2 = LegoPort('spi0.1:S2:i2c1')

# Connect sensor to sensor port ?
mi = Sensor(INPUT_2)
# mi = Sensor('spi0.1:S2:i2c1')  # unclear if this change helped in any way or not
# mi = I2cSensor('spi0.1:S2:i2c1')  # unclear if this change helped in any way or not
# mi = I2cSensor(INPUT_2)  # unclear if this change helped in any way or not

print("wait 5")

# allow for some time to stabilze
time.sleep(3)

# mi.poll_ms = 0   # didn't work

# set the mi sensor mode
mi.command = 'RESET'

print("wait 6")

# allow for some time to stabilze
time.sleep(3)

# set the mi sensor mode
mi.mode = 'ANGLE'
# mi.mode = 'ALL'

print("wait 7")
print("")
print("")

# allow for some time to stabilze
time.sleep(3)

miVal = 0
prev_miVal = 18001

### For testing
print("mi driver name is...         ", mi.driver_name)
print("mi address is...             ", mi.address)
print("mi mode is...                ", mi.mode)
print("mi available modes are...    ", mi.modes)
print("mi available commands are... ", mi.commands)
print("")


print("#######################################################################")
print("####################       read mi        #############################")
print("#######################################################################")


while (1):
    miVal = mi.value(0)
    print("miVal = ", miVal)
    time.sleep (1) # Give the CPU a rest


