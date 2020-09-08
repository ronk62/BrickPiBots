#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 5/9/2020      Ron King    initial dev; ported from MLtestGrabLiftV2.0.py
#                           this program will test the ability to detect MineLoader
#                           steering articulation limits using duty_cycle and motor stall detection
# 5/10/2020     Ron King    updated design to use stall mechanism with mL.duty_cycle_sp found experimentally
#                           and stored in var 'mLDtyMax'

import time, tty, sys, threading
from ev3dev2 import list_devices
from ev3dev2.motor import LargeMotor, OUTPUT_A, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

# some keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global


# setup motor
mL = LargeMotor(OUTPUT_A)
time.sleep(0.5)
mL.reset()
time.sleep(2)
mL.duty_cycle_sp = 0
mLDtyMax = 12
mL.stop_action = 'coast'
mL.polarity = 'inversed'
mL.position = 0

StallDetected = False
mLlastPos = 0
mLposU = 0
mLposJ = 0
i = 0


# Turn motor on with run_direct and duty_cycle_sp of '0'
mL.run_direct()

def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")



#### Main Loop
while (True):
    print ("Initial mL.position =  ", mL.position)
    while not StallDetected:        ##### consider this rethunk <wink>
        if (mL.duty_cycle_sp != 0) and (mL.position == mLlastPos):
            i += 1
        else:
            i = 0
        if i >= 5:
            StallDetected = True
            if mL.duty_cycle_sp > 0:
                mLposU = mL.position
            elif mL.duty_cycle_sp < 0:
                mLposJ = mL.position
            mL.duty_cycle_sp = 0
            print("motor stalled; killing power")
            print("x, mL.duty_cycle_sp, mL.position, mLlastPos  ", x, mL.duty_cycle_sp, mL.position, mLlastPos)
            break
        if x == 120: # x key pushed - exit
            mL.duty_cycle_sp = 0
            mL.off(brake=False)
            break
        if x == 32: # space key pushed
            mL.duty_cycle_sp = 0
        if x == 117: # u key pushed
            mL.duty_cycle_sp = mLDtyMax
        if x == 106: # j key pushed
            mL.duty_cycle_sp = -mLDtyMax
        if mL.position != mLlastPos:
            print("x, mL.duty_cycle_sp, mL.position, mLlastPos  ", x, mL.duty_cycle_sp, mL.position, mLlastPos)
        mLlastPos = mL.position
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(.1)

    if x == 120: # x key pushed - exit
            mL.duty_cycle_sp = 0
            mL.off(brake=False)
            break
    
    print("Reseting StallDetected to 'False'")
    print("current mL.position, mLposU, mLposJ  ", mL.position, mLposU, mLposJ)
    time.sleep(2)
    StallDetected = False
