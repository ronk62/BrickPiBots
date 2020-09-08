#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 3/19/2020     Ron King    Initial dev


import time, tty, sys
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank, MoveSteering
# from ev3dev2.sensor import INPUT_1

print ("Please attach Large motors, Left motor to PORT_A and Right motor to PORT_B")

time.sleep(1)

""" 
This program provides a test platform to measure position and speed synchronization between
motors A and B under different conditions.

Findings/results:

* As expected, both MoveTank and MoveSteering failed to provide motor synchronization.

* Both MoveTank and MoveSteering did provide very good motor regulation reasonable position symetry, although
MoveTank was better than MoveSteering at position symetry, for some reason.

====

* A new approach using a master/slave configuration and allowing for constant A/B ratio even if motor A or B
is slowed by external load.

High-level proposal...
- measure the difference (error) between queried speed and desired/set speed of each motor
- use those differences to set variables 'errorA' and 'errorB'
-- ex. errorA = mA.speed - mAspeed
- use MoveTank (rather than MoveSteering) to keep math more simple
- use some logic and a formula to periodically correct speeds based on error

-- this example works only when A/B ratio is 1 (i.e. both motors are set to same speed)
if errorA > errorB:
    tank_pair.on(mAspeed, (mBspeed - error))
if errorB > errorA:
    tank_pair.on((mAspeed - error), mBspeed)

-- here's the basis of a more universal method that will allow non-1:1 ratio...
desiredRatio = mAspeed/mBspeed
turn motors on at mAspeed and mBspeed
while true:
    actualRatio = mA.speed/mB.speed
    if actualRatio >= desiredRatio:
        do the needful (calculate speed reduction for motorA and apply)
    if actualRatio < desiredRatio:
        do the needful (calculate speed reduction for motorB and apply)

--- calculations and application
if actualRatio >= desiredRatio:
        #do the needful (calculate speed reduction for motorA and apply)
        corrected_mAspeed = mB.speed * desiredRatio
        tank_pair.on(corrected_mAspeed, mBspeed)

if actualRatio < desiredRatio:
        #do the needful (calculate speed reduction for motorB and apply)
        corrected_mBspeed = mA.speed / desiredRatio
        tank_pair.on(mAspeed, corrected_mBspeed)


---- easy test code

dR = 1/1
for i in range(-10,11):
    a=100+i
    b=100
    aR=a/b
    print("before corrections  ", i,a,b,aR)
    if aR >= dR:
        a = b * dR
    if aR < dR:
        b = a / dR
    aR=a/b
    print("after corrections  ", i,a,b,aR)
    print("")

---- alt version (mess with b instead of a)

dR = 1/1
for i in range(-10,11):
    a=100
    b=100+i
    aR=a/b
    print("before corrections  ", i,a,b,aR)
    if aR >= dR:
        a = b * dR
    if aR < dR:
        b = a / dR
    aR=a/b
    print("after corrections  ", i,a,b,aR)
    print("")

 """

tty.setcbreak(sys.stdin)

mA = LargeMotor(OUTPUT_A)
time.sleep(0.5)

# mA.reset()
# time.sleep(2)
# mAspd = 0
# mA.stop_action = 'coast'
# mA.position = 0
 

mB = LargeMotor(OUTPUT_B)
time.sleep(0.5)
# mB.reset()
# time.sleep(2)
# mBspd = 0
# mB.stop_action = 'coast'
# mB.position = 0

# tank_pair = MoveTank(OUTPUT_A, OUTPUT_B)
# time.sleep(0.5)
# tank_pair.reset()

steer_pair = MoveSteering(OUTPUT_A, OUTPUT_B)
time.sleep(0.5)
steer_pair.reset()

print ("starting motors...")

# tank_pair.on(left_speed=SpeedDPS(200), right_speed=SpeedDPS(200))
# tank_pair.on(8, 8)
steer_pair.on(steering=0, speed=8)
time.sleep(0.5)

while (True):
    print("motor positions (A, B, difference) ", mA.position, mB.position, (mB.position - mA.position))
    print("motor speeds (A and B) ", mA.speed, mB.speed)
    print("")
    time.sleep(1)

