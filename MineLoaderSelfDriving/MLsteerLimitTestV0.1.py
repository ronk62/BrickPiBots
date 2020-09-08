#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 5/9/2020      Ron King    initial dev; ported from MLtestGrabLiftV2.0.py
#                           this program will test the ability to detect MineLoader
#                           steering articulation using duty_cycle and motor stall detection

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
mLDtyMax = 16
mL.stop_action = 'coast'
mL.polarity = 'inversed'
mL.position = 0

StallDetected = False
mLlastPos = 0
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
    ##### FAIL - need to rethink the below mL.position based stall detection
    ##### FAIL - mL.position > mLlastPos results in false stall detection
    ##### FAIL - consider when motor not powered
    ##### FAIL - and when motor in reverse direction
    ##### FAIL - perhaps go back to what worked in "pi-push-ups.py" see next line
    #### while abs(large_motor.duty_cycle) <= abs(abs(mAspeed) + abs(duty_cycle_margin)):
    #### etc...
    #### Optionally, need to add conditional logic that ensures
    ####     1) mL.duty_cycle_sp != 0 before checking mL.position > mLlastPos or mL.position < mLlastPos
    ####     2) check motor direction before checking mL.position > mLlastPos or mL.position < mLlastPos
    while not StallDetected: ##### FAIL - need to rethink this
        # if not mL.position > mLlastPos:
        #     i += 1
        # else:
        #     i = 0
        # if i >= 5:
        #     StallDetected = True
        #     mL.duty_cycle_sp = 0
        #     mL.off(brake=False)
        #     print("motor stalled; killing power")
        #     break
        mLlastPos = mL.position
        if x == 120: # x key pushed - exit
            mL.duty_cycle_sp = 0
            mL.off(brake=False)
            break
        if x == 32: # space key pushed
            mL.duty_cycle_sp = 0
        if x == 117: # u key pushed
            if mL.duty_cycle_sp < mLDtyMax:
                mL.duty_cycle_sp = mL.duty_cycle_sp + 2
        if x == 106: # j key pushed
            if mL.duty_cycle_sp > -mLDtyMax:
                mL.duty_cycle_sp = mL.duty_cycle_sp - 2
        print("x, mL.duty_cycle_sp  ", x, mL.duty_cycle_sp)
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(.1)

    if x == 120: # x key pushed - exit
            mL.duty_cycle_sp = 0
            mL.off(brake=False)
            break
    
    print("Reseting StallDetected to 'False'; current mL.position =  ", mL.position)
    time.sleep(2)
    StallDetected = False
