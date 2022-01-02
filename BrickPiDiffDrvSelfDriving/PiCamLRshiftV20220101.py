#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 1/1/2022      Ron King    - Started with file MLtestGrabLiftV2.0.py
#                           - this is the initial code testing the Left Right PiCam shuttle mechanism
#                             designed to simulate a Stereo camera
#

import time, tty, sys
from ev3dev2.motor import LargeMotor, OUTPUT_D, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

tty.setcbreak(sys.stdin)


mShuttle = LargeMotor(OUTPUT_D)
time.sleep(0.5)
mShuttle.reset()
time.sleep(2)
mShuttle.duty_cycle_sp = 0
mShuttleDtyMax = 22
mShuttle.stop_action = 'coast'
mShuttle.polarity = 'inversed'
mShuttle.position = 0

prev_mShuttle_pos = 0

# Turn motor on with run_direct and duty_cycle_sp of '0'
mShuttle.run_direct()

print ("Ready for keyboard commands...")

while (True):
    x = ord(sys.stdin.read(1))
    if x == 120: # x key pushed - exit
        mShuttle.duty_cycle_sp = 0
        mShuttle.off(brake=False)
        break
    if x == 32: # space key pushed
        mShuttle.duty_cycle_sp = 0
    if x == 43: # + key pushed
        if mShuttle.duty_cycle_sp < mShuttleDtyMax:
            mShuttle.duty_cycle_sp = mShuttle.duty_cycle_sp + 2
    if x == 45: # - key pushed
        if mShuttle.duty_cycle_sp > -mShuttleDtyMax:
            mShuttle.duty_cycle_sp = mShuttle.duty_cycle_sp - 2

    if x == 60: # < key pushed
        prev_mShuttle_pos = mShuttle.position
        mShuttle.duty_cycle_sp = 22
        time.sleep(0.25)
        mShuttle.duty_cycle_sp = 20
        while prev_mShuttle_pos != mShuttle.position:
            prev_mShuttle_pos = mShuttle.position
            print("< key pressed... x, mShuttleDty vals =  ", x, mShuttle.duty_cycle_sp)
            time.sleep(0.25)
        mShuttle.duty_cycle_sp = 0
        print("motion stopped, stopped motor and set x to 32")
        x = 32
        

    if x == 62: # > key pushed
        prev_mShuttle_pos = mShuttle.position
        mShuttle.duty_cycle_sp = -22
        time.sleep(0.25)
        mShuttle.duty_cycle_sp = -20
        while prev_mShuttle_pos != mShuttle.position:
            prev_mShuttle_pos = mShuttle.position
            print("> key pressed... x, mShuttleDty vals =  ", x, mShuttle.duty_cycle_sp)
            time.sleep(0.25)
        mShuttle.duty_cycle_sp = 0
        print("motion stopped, stopped motor and set x to 32")
        x = 32
    

    print("x, mShuttleDty  ", x, mShuttle.duty_cycle_sp)
    time.sleep(.05)

