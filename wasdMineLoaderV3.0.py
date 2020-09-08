#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 2/22/2020     Ron King    Port from MindSensors Pistorms version
# 3/1/2020      Ron King    Fixed issues in speed change control logic and sequencing
# 3/15/2020     Ron King    Significant rewrite and refactoring

import time, tty, sys
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach Large motors, Left motor to PORT_A and Right motor to PORT_B")

time.sleep(1)

""" 
This program provides awsd keyboard remote control of the
MineLoader (lego model 42049) with large ev3 motors driving 
the front wheels in differential-drive configuration.

motor A = Left motor
motor B = Right motor

Both motors are mounted such that positive power = backward motion, so polarity had to be inversed
 """

doExit = False

tty.setcbreak(sys.stdin)

mL = LargeMotor(OUTPUT_A)
time.sleep(0.5)

mL.reset()
time.sleep(2)
mLspd = 0
mL.stop_action = 'coast'
mL.polarity = 'inversed'
mL.position = 0
 

mR = LargeMotor(OUTPUT_B)
time.sleep(0.5)
mR.reset()
time.sleep(2)
mRspd = 0
mR.stop_action = 'coast'
mR.polarity = 'inversed'
mR.position = 0

spd = 0        # set this value to -30 to +90; use to set outer wheel/motor speed; default to mL for driving straight
turnRatio = 0.0  # set this value to -0.5 to +0.5; subtract from 1 to set turn ratio; multiply * spd to set inside motor speed

print ("Ready for keyboard commands...")

while (doExit == False):
    x = ord(sys.stdin.read(1))
    if x == 32 or x == 120: # space or x key pushed
        spd = 0
        turnRatio = 0
    if x == 119: # w key pushed
        if spd < 90:  # limit max frwd speed to 90
            spd = spd + 5
    if x == 115: # s key pushed
        if spd > -30:  # limit max rvrs speed to -30
            spd = spd - 5
    if x == 100: # d key pushed (turn more to the Right)
        if turnRatio > -0.5:
            turnRatio = turnRatio - 0.1
    if x == 97: # a key pushed (turn more to the Left)
        if turnRatio < 0.5:
            turnRatio = turnRatio + 0.1

    if turnRatio <= 0:
        mLspd = spd
        mRspd = spd * (1 + turnRatio)
    if turnRatio > 0:
        mRspd = spd
        mLspd = spd * (1 - turnRatio)

    mL.on(mLspd, brake=False)
    mR.on(mRspd, brake=False)
    if x == 120: # x key means exit
        break

    print("x, spd, turnRatio, mLspd, mRspd  ", x, spd, turnRatio, mLspd, mRspd)
    time.sleep(.05)

