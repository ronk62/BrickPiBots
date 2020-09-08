#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 2/22/2020     Ron King    Port from MindSensors Pistorms version

import time, tty, sys
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach Large motors, Left motor to PORT_A and Right motor to PORT_B")

time.sleep(1)

""" 
This program provides awsd keyboard remote control of the
MineLoader (lego model 42049) with large ev3 motors driving 
the front wheels in differential-drive configuration.

motor 1, bank A = Left motor
motor 2, bank A = Right motor

Both motors are mounted such that positive power = forward motion
 """

doExit = False

tty.setcbreak(sys.stdin)

mL = LargeMotor(OUTPUT_A)
time.sleep(0.5)

mL.reset()   # avoid using; on pistorms problem when 'reset' is followed by motot 'on' at low speed
################ 'reset' and 'position = 0' result in pos of 0, and motor controller behaves poorly
################  when pos != 226 when starting the motor

time.sleep(2)
mLspd = 0
mL.stop_action = 'coast'
# mL.position = 226 ### FAILED work around for pistorms 'reset' followed by motot 'on' at low speed issue
####################### ### FAIL; can't write anything but '0' to position; the problem is at the driver level
mL.position = 0  ### avoid using on pistorms; motor controller behaves poorly when pos != 226 when starting the motor
 

mR = LargeMotor(OUTPUT_B)
time.sleep(0.5)
mR.reset()
time.sleep(2)
mRspd = 0
mR.stop_action = 'coast'
mR.position = 0

print ("Ready for keyboard commands...")

while (doExit == False):
    x = ord(sys.stdin.read(1))
    if x == 120: # x key pushed - exit
        mLspd = 0
        mRspd = 0
        mL.on(mLspd, brake=False)
        mR.on(mRspd, brake=False)
        break
    if x == 32: # space key pushed
        mLspd = 0
        mRspd = 0
        mL.on(mLspd, brake=False)
        mR.on(mRspd, brake=False)
    if x == 119: # w key pushed
        if mLspd < 90:
            mRspd = mLspd = mLspd + 5
            mL.on(mLspd, brake=False)
            mR.on(mRspd, brake=False)
    if x == 115: # s key pushed
        if mLspd > -30:
            mRspd = mLspd = mLspd - 5
            mL.on(mLspd, brake=False)
            mR.on(mRspd, brake=False)
    if x == 97: # a key pushed
        if mLspd == 0.7 * mRspd:
            mLspd = 0.5 * mRspd
        if mLspd == 0.8 * mRspd:
            mLspd = 0.7 * mRspd
        if mLspd == mRspd:
            mLspd = 0.8 * mRspd
        if mRspd < mLspd:
            mLspd = 0.8 * mRspd
            mL.on(mLspd, brake=False)
            mR.on(mRspd, brake=False)
    if x == 100: # d key pushed
        if mRspd == 0.7 * mLspd:
            mRspd = 0.5 * mLspd
        if mRspd == 0.8 * mLspd:
            mRspd = 0.7 * mLspd
        if mRspd == mLspd:
            mRspd = 0.8 * mLspd
        if mLspd < mRspd:
            mRspd = 0.8 * mLspd
            mL.on(mLspd, brake=False)
            mR.on(mRspd, brake=False)
    
    print("x, mLspd, mRspd  ", x, mLspd, mRspd)
    time.sleep(.05)

