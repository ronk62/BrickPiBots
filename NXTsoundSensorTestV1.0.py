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
from ev3dev2.sensor.lego import SoundSensor

p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'nxt-analog'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html
p1.set_device = 'lego-nxt-sound'


# Connect infrared to any sensor port
ss = SoundSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

ssDbVal = 0
prev_ssDbVal = 0

ssDbaVal = 0
prev_ssDbaVal = 0

print("#######################################################################")
print("###################  sound sensor db mode  ############################")
print("#######################################################################")

startTime = time.time()

while (time.time() - startTime < 55):
    ssDbVal = ss.sound_pressure
    if ssDbVal != prev_ssDbVal:
        print("ssDbVal = ", ssDbVal)
        prev_ssDbVal = ssDbVal
    time.sleep (0.05) # Give the CPU a rest


print("#######################################################################")
print("##################  sound sensor dba mode  ############################")
print("#######################################################################")

startTime = time.time()

ssDbaVal = 0
prev_ssDbaVal = 0


while (time.time() - startTime < 55):
    ssDbaVal = ss.sound_pressure_low
    if (ssDbaVal != prev_ssDbaVal):
            print("ssDbaVal = ", ssDbaVal)
            prev_ssDbaVal = ssDbaVal
    time.sleep (0.2) # Give the CPU a rest

print("#######################################################################")
print("####################   time's up - exiting    #########################")
print("#######################################################################")
