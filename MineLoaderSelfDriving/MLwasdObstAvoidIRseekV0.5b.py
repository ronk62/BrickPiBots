#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 4/19/2020     Ron King    used file /home/robot/ev3dev2Projects/wasdNonBlockingv1.0.py as starting point
#                           trimmed out some superfluous comments
#                           modified to add ir sensor (INPUT_1) in proximity mode and imu (INPUT_2) in compass mode
# 4/26/2020     Ron King    Replaced Mindsendors IMU with HiTechnic NXT Compass
#                           Added the scaffolding for auto-pilot; partially implemented
# 5/3/2020      Ron King    Added IR Beacon data (distance, heading) - difficulties with simultaneous Prox and Seek
#                           This iteration has lead me to believe a better approach would be to use US to avoid obstacles
# 5/4/2020      Ron King    Added the US sensor (on port S3) and replaced IR Proximity process with code using the
#                           US distance_centimeters data for obstacle avoidance.


""" 
This program provides wasd keyboard remote control testing with obstacle avoidance (ir proximity sensor).

Intermediate test programs and trial runs

 1) drive straight toward obstacle and stop at {distance from obstacle}

Non-blocking...
- Using a multi-thread approach (see https://realpython.com/intro-to-python-threading/)
- created method to read x and moved `x = ord(sys.stdin.read(1))` there
- instantiated `x = 0` outside of any code blocks to make it global
- declared x as global within keyboardInput() method/thread
- had to set `x = 0` near the end of the main loop to prevent the equiv of a repeat/stuck keyboad key
-- this approach works great!


Motors

motor A = Left motor
motor B = Right motor

 """

import time, tty, sys, threading
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import InfraredSensor, UltrasonicSensor
from ev3dev2.sensor.lego import Sensor


p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p1.set_device = 'lego-ev3-ir'


# Connect infrared to sensor port
ir = InfraredSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

# irProxVal = 100
# prev_irProxVal = 0
irDistVal = 0
prev_irDistVal = 0
irHeadVal = 0
prev_irHeadVal = 0



p2 = LegoPort(INPUT_2)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p2.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device """

# set device name and i2c address (in hex)
p2.set_device = 'ht-nxt-compass 0x01'

# allow for some time for setup
time.sleep(0.5)

# Connect ht-nxt-compass sensor to sensor port
cmp = Sensor(INPUT_2)

# allow for some time to load the new drivers
time.sleep(0.5)

# cmp.mode = 'COMPASS'  ###  Not req'd for this sensor as it only has one mode

compassVal = 0
prev_compassVal = 0
compassGoal = 220     # with ht-compass mounted sideways, this translates to mostly North



p3 = LegoPort(INPUT_3)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p3.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p3.set_device = 'lego-ev3-us'



# Connect infrared to any sensor port
us = UltrasonicSensor(INPUT_3)

# allow for some time to load the new drivers
time.sleep(0.5)

usDistCmVal = 0
prev_usDistCmVal = 0



# some keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global
pilot_mode = 1  # start in human mode, '1'; '-1' means auto-pilot

# some Nav control vars
initSearch = False      # have we completed the initial search circle, looking for ir beacon?
beaconLock = False      # has the beacon been found and data read?
cmpGenHeading = False   # do we have a lock on compass heading for North?
beaconSearch = False    # are we searching for the beacon right now?
cmpSearch = False       # are we searching for North with the compass right now?
navPrint = 10           # print the Nav msgs every nth cycle
i = 0                   # iterator for testing

# setup motors
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


def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")


### Main Loop
    while (True):
        usDistCmVal = us.distance_centimeters
        if usDistCmVal != prev_usDistCmVal:
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        irDistVal = ir.distance()
        irHeadVal = ir.heading()
        if (irDistVal != prev_irDistVal) or (irHeadVal != prev_irHeadVal):
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_irDistVal = irDistVal
            prev_irHeadVal = irHeadVal
            # time.sleep(0.2)

        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal
            if compassVal < (compassGoal - 20) or compassVal > (compassGoal + 20):
                cmpGenHeading = False

        if usDistCmVal < 50:
            print("Obstacle Detected! Forward motion stopped.")
            if spd > 0:
                spd = 0
            if pilot_mode < 0:
                # stop motors
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                print("Auto-pilot collision avoidance - backing up")
                # backup at speed 5 with equal speed applied to left and right motors
                spd = -5
                # do so for 3 seconds
                mL.on(speed=spd, brake=False)
                mR.on_for_seconds(speed=spd, seconds=3, brake=False, block=True)
                # stop motors
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                print("Auto-pilot collision avoidance - turning")
                spd = 5
                if turnRatio >= 0:   # if previously going straight or turning left, turn hard right
                    turnRatio = -0.5
                    mLspd = spd
                    mRspd = spd * (1 + turnRatio)
                if turnRatio < 0:    # if previously turning right, turn hard left
                    turnRatio = 0.5
                    mRspd = spd
                    mLspd = spd * (1 - turnRatio)
                # proceed forward for 4 seconds
                mL.on(speed=mLspd, brake=False)
                mR.on_for_seconds(speed=mRspd, seconds=4, brake=False)
                # stop motors
                mL.on(0, brake=False)
                mR.on(0, brake=False)

                # all bets are off that we have a fix on orientation or position;  reset Nav vars
                initSearch = False
                beaconLock = False
                cmpGenHeading = False
                beaconSearch = False
                cmpSearch = False
                navPrint = 10
                print("Auto-pilot collision avoidance done - resuming normal ops with Nav vars reset")

                x = 33 # set x to non-zero so we print spd, etc; 33 = ascii ! 

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

        if x == 112: # p pushed - toggle pilot mode between human and auto
            pilot_mode *= -1
            print("toggled pilot_mode; now set to...  ", pilot_mode)

        if pilot_mode < 0:
            if beaconLock:   # if we have a good lock on the ir beacon, use this for beacon Nav
                # speed and steering controlled by beacon Nav
                # set speed (slow)
                spd = 0   #####  set to 0 for testing
                # circle CW looking for compassGoal +/- 5
                turnRatio = 0   #####  set to 0 for testing
            elif cmpGenHeading:  # if compass Nav has us going mostly North, use compass Nav
                # speed (slow) and steering controlled by compass Nav
                # set speed (slow)
                spd = 0   #####  set to 0 for testing
                # circle CW looking for compassGoal +/- 5
                turnRatio = 0   #####  set to 0 for testing
            elif not initSearch:  # if we have not yet circled, looking for ir beacon, do so now
                beaconSearch = True
                # set speed (slow)
                spd = 0   #####  set to 0 for testing
                # circle CW looking for compassGoal +/- 5
                turnRatio = 0   #####  set to 0 for testing
                # initiate a CW 360 deg circle while looking for beacon
                i +=1
                if i > 25:
                    initSearch = True
                    beaconSearch = False
                    print("Failed to find beacon. Continuing with compass North strategy")
                    i = 0
                if beaconLock:  # once the beacon is found, set initSearch True and beaconSearch False
                    initSearch = True
                    beaconSearch = False
            else:  # default action is to find North (compassGoal = ~220) and head that way while looking for beacon                
                cmpSearch = True
                # set speed (slow)
                spd = 10
                # circle CW looking for compassGoal +/- 5
                turnRatio = -0.5
                if compassVal > (compassGoal - 5) and compassVal < (compassGoal + 5):
                    cmpGenHeading = True
                    cmpSearch = False
                    print("We found North. Done with cmpSearch")

        # calculate motor speeds for turning right
        if turnRatio <= 0:
            mLspd = spd
            mRspd = spd * (1 + turnRatio)
        
        # calculate motor speeds for turning left
        if turnRatio > 0:
            mRspd = spd
            mLspd = spd * (1 - turnRatio)

        # apply motor speeds
        mL.on(mLspd, brake=False)
        mR.on(mRspd, brake=False)

        if x == 120: # x key means exit
            break

        if x != 0:
            print("x, spd, turnRatio, mLspd, mRspd, pilot_mode  ", x, spd, turnRatio, mLspd, mRspd, pilot_mode)
        
        if navPrint == 10:
            print("NAV:  x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock  ", x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock)
            navPrint = 0
        navPrint += 1
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(0.2)

