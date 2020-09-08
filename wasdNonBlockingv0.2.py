#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 3/29/2020     Ron King    initial dev to test non-blocking wasd keyboard control


""" 
This program provides wasd keyboard remote control testing with methods aimed at not blocking other code.

- Starting with some example code from Dexter Industries 'simplebot' code design.
- replace `x = ord(sys.stdin.read(1))` with `x = str(input())`
-- this approach failed

- Trying a multi-thread approach (see https://realpython.com/intro-to-python-threading/)
- created method to read x and moved `x = ord(sys.stdin.read(1))` there
- instantiated `x = 0` outside of any code blocks to make it global
- declared x as global within keyboardInput()
- had to set `x = 0` near the end of the main loop to prevent the equiv of a repeat/stuck keyboad key
-- this approach works great!


Motors (if needed)

motor A = Left motor
motor B = Right motor

 """

import time, tty, sys, threading
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1


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

x = 0   # set here to make global

def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        # print("keyboardInput(1) executing, x=  ", x)
        x = ord(sys.stdin.read(1))
        # print("keyboardInput(1) executing, new value for x=  ", x)
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")

    while (True):
        # x = ord(sys.stdin.read(1))
        # print("about to call keyboardInput(1), x=  ", x)
        # keyboardInput(1)
        # print("returned from call keyboardInput(1), x=  ", x)
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

    #   mL.on(mLspd, brake=False)
    #   mR.on(mRspd, brake=False)
        if x == 120: # x key means exit
            break

        print("x, spd, turnRatio, mLspd, mRspd  ", x, spd, turnRatio, mLspd, mRspd)
        x = 0
        time.sleep(0.2)

