#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 2/22/2020     Ron King    Port from MindSensors Pistorms version
# 3/1/2020      Ron King    Fixed issues in speed change control logic and sequencing

import time, tty, sys
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1


doExit = False

tty.setcbreak(sys.stdin)

mLift = LargeMotor(OUTPUT_C)
time.sleep(0.5)

mLift.reset()   # avoid using; on pistorms problem when 'reset' is followed by motot 'on' at low speed
################ 'reset' and 'position = 0' result in pos of 0, and motor controller behaves poorly
################  when pos != 226 when starting the motor

time.sleep(2)
mLiftspd = 0
mLift.stop_action = 'coast'
mLift.polarity = 'inversed'
# mLift.position = 226 ### FAILED work around for pistorms 'reset' followed by motor 'on' at low speed issue
####################### FAIL; can't write anything but '0' to position; the problem is at the driver level
mLift.position = 0  ### avoid using on pistorms; motor controller behaves poorly when pos != 226 when starting the motor
 

mGrab = LargeMotor(OUTPUT_D)
time.sleep(0.5)
mGrab.reset()
time.sleep(2)
mGrabspd = 0
mGrab.stop_action = 'coast'
mGrab.polarity = 'inversed'
mGrab.position = 0

print ("Ready for keyboard commands...")

while (doExit == False):
    x = ord(sys.stdin.read(1))
    if x == 120: # x key pushed - exit
        mLiftspd = 0
        mGrabspd = 0
        mLift.on(mLiftspd, brake=False)
        mGrab.on(mGrabspd, brake=False)
        break
    if x == 32: # space key pushed
        mLiftspd = 0
        mGrabspd = 0
        mLift.on(mLiftspd, brake=False)
        mGrab.on(mGrabspd, brake=False)
    if x == 117: # u key pushed
        if mLiftspd < 60:
            mLiftspd = mLiftspd + 2
        mLift.on(mLiftspd, brake=False)
    if x == 106: # j key pushed
        if mLiftspd > -60:
            mLiftspd = mLiftspd - 2

        mLift.on(mLiftspd, brake=False)
    if x == 107: # k key pushed
        if mGrabspd < 60:
            mGrabspd = mGrabspd + 2
        mGrab.on(mGrabspd, brake=False)
    if x == 104: # h key pushed
        if mGrabspd > -60:
            mGrabspd = mGrabspd - 2
        mGrab.on(mGrabspd, brake=False)
    
    print("x, mLiftspd, mGrabspd  ", x, mLiftspd, mGrabspd)
    time.sleep(.05)

