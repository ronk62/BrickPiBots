#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 2/22/2020     Ron King    Port from MindSensors Pistorms version
# 3/1/2020      Ron King    Fixed issues in speed change control logic and sequencing
# 3/22/2020     Ron King    v2.0 - change from regulated 'on', 'speed' to duty_cycle
#                           Removed all traces of Pistorms version (issues and comments)

import time, tty, sys
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

tty.setcbreak(sys.stdin)

mLift = LargeMotor(OUTPUT_C)
time.sleep(0.5)
mLift.reset()
time.sleep(2)
mLift.duty_cycle_sp = 0
mLiftDtyMax = 36
mLift.stop_action = 'coast'
mLift.polarity = 'inversed'
mLift.position = 0
 

mGrab = LargeMotor(OUTPUT_D)
time.sleep(0.5)
mGrab.reset()
time.sleep(2)
mGrab.duty_cycle_sp = 0
mGrabDtyMax = 36
mGrab.stop_action = 'coast'
mGrab.polarity = 'inversed'
mGrab.position = 0


# Turn motors on with run_direct and duty_cycle_sp of '0'
mLift.run_direct()
mGrab.run_direct()

print ("Ready for keyboard commands...")

while (True):
    x = ord(sys.stdin.read(1))
    if x == 120: # x key pushed - exit
        mLift.duty_cycle_sp = 0
        mGrab.duty_cycle_sp = 0
        mLift.off(brake=False)
        mGrab.off(brake=False)
        break
    if x == 32: # space key pushed
        mLift.duty_cycle_sp = 0
        mGrab.duty_cycle_sp = 0
    if x == 117: # u key pushed
        if mLift.duty_cycle_sp < mLiftDtyMax:
            mLift.duty_cycle_sp = mLift.duty_cycle_sp + 2
    if x == 106: # j key pushed
        if mLift.duty_cycle_sp > -mLiftDtyMax:
            mLift.duty_cycle_sp = mLift.duty_cycle_sp - 2
    if x == 107: # k key pushed
        if mGrab.duty_cycle_sp < mGrabDtyMax:
            mGrab.duty_cycle_sp = mGrab.duty_cycle_sp + 2
    if x == 104: # h key pushed
        if mGrab.duty_cycle_sp > -mGrabDtyMax:
            mGrab.duty_cycle_sp = mGrab.duty_cycle_sp - 2
    
    print("x, mLiftDty, mGrabDty  ", x, mLift.duty_cycle_sp, mGrab.duty_cycle_sp)
    time.sleep(.05)

