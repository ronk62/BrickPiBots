#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 5/31/2020     Ron King    used file /home/robot/ev3dev2Projects/MineLoaderSelfDriving/MLwasdObstAvoidIRseekV0.13b.py as starting point

""" 
This program provides SLAM testing with wasd keyboard remote control and some obstacle avoidance.

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
compassGoal = 245     # with ht-compass mounted sideways, this translates to mostly North, slightly East



p3 = LegoPort(INPUT_3)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p3.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p3.set_device = 'lego-ev3-us'


# Connect lego-ev3-us to any sensor port
us = UltrasonicSensor(INPUT_3)

# allow for some time to load the new drivers
time.sleep(0.5)

usDistCmVal = 0
prev_usDistCmVal = 0



# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
pilot_mode = 1  # start in human mode, '1'; '-1' means auto-pilot
obstacleNear = False    # there is an obstacle nearby - need to stop, turn, backup, etc (and go another way)
# obstacleMed = False     # there is an obstacle ahead - veer away
backingUp = False       # use this to control state of obstacle avoidance maneuvers
backingUpTime = 0.0     # use to control how long we backup
evadeLeft = False       # turn hard left for obstacle avoidance
evadeRight = False      # turn hard right for obstacle avoidance
evadeTime = 0.0         # use to control how long we evade left or right
searchSLAM1 = True     # are we exexcuting the initial SLAM circle?
beaconLock = False      # has the beacon been found and data read?
cmpGenHeading = False   # do we have a lock on compass heading for North?
# beaconSearch = False    # are we searching for the beacon right now?
cmpSearch = False       # are we searching for North with the compass right now?
navPrint = 10           # print the Nav msgs every nth cycle
i = 0                   # iterator for testing

# SLAM vars
polarCoords = []    # list to hold US distance (cm) with index = approx compass-angle (deg)


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
            # if beacon has been found, set beaconLock True
            if irDistVal is not None:
                if irDistVal < 100:
                    beaconLock = True
                if irDistVal == 100:
                    beaconLock = False
            else:
                beaconLock = False

        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal
            # if beaconLock is False and we're outside the compass North range, reset cmpGenHeading to False
            if compassVal < (compassGoal - 80) or compassVal > (compassGoal + 80):
                cmpGenHeading = False

        ### if usDistCmVal < 50:
        if usDistCmVal < 10:    ### changed from '50' to '10' for SLAM testing
            if obstacleNear == False:
                print("Obstacle Detected! Forward motion stopped.")
                if spd > 0:
                    spd = 0
                    if pilot_mode < 0:
                        # stop and return to human pilot mode
                        mL.on(0, brake=False)
                        mR.on(0, brake=False)
                        ### added for initial SLAM testing
                        pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
                        print("obstacle Near - returning to human pilot mode")
                        ### commented for initial SLAM testing
                        # obstacleNear = True
                    x = 33 # set x to non-zero so we print spd, etc; 33 = ascii !
        # elif (usDistCmVal >= 30)  and (usDistCmVal < 100):
        #     if pilot_mode < 0:
        #         obstacleMed = True

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

        if irDistVal is not None:
            if pilot_mode < 0 and irDistVal <= 25:
                # stop motors
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                spd = 0
                turnRatio = 0
                pilot_mode = 1    # destination reached; return to human pilot mode
                print("")
                print("")
                print("destination reached; returned to human pilot mode")
                print("")
                print("")
                time.sleep(3)

        if pilot_mode < 0:
            if  obstacleNear == True:
                # obstacleMed = False
                if not (backingUp or evadeLeft or evadeRight):
                    # stop and backup
                    mL.on(0, brake=False)
                    mR.on(0, brake=False)
                    ### added for initial SLAM testing
                    pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
                    print("obstacle Near - returning to human pilot mode")
                    ### commented the below to omit "normal" obstacle avoid routines during initial SLAM testing
                    # backingUp = True
                    # backingUpTime = time.time()
                # if backingUp == True:
                #     print("Auto-pilot collision avoidance - backing up")
                #     # backup at speed 5
                #     spd = -10
                #     if not (evadeLeft or evadeRight):
                #         # determine next action based on compass orientaion
                #         if (compassVal >= compassGoal) or (compassVal < (compassGoal - 180)):
                #             evadeLeft = True
                #             turnRatio = 0.5
                #             print("Turning Left; swing backend to vehical Right")
                #         elif (compassVal < (compassGoal)) or (compassVal >= (compassGoal - 180)):
                #             evadeRight = True
                #             turnRatio = -0.5
                #             print("Turning Right; swing backend to vehical Left")
                #     # backup for only ~8 seconds - then stop
                #     if time.time() - backingUpTime >= 8:
                #         print(time.time() - backingUpTime)
                #         backingUp = False
                #         spd = 0
                #         turnRatio = 0
                #         mL.on(spd, brake=False)
                #         mR.on(spd, brake=False)                        
                #         print("Auto-pilot collision avoidance - turning")
                #         evadeTime = time.time()
                #         spd = 10
                # if backingUp == False and evadeLeft == True:
                #     turnRatio = 0.5
                # elif backingUp == False and evadeRight == True:
                #     turnRatio = -0.5
                # # evade left or right for 8 seconds
                # if backingUp == False and (evadeLeft or evadeRight) and (time.time() - evadeTime >= 8):
                #         spd = 0
                #         turnRatio = 0
                #         mL.on(spd, brake=False)
                #         mR.on(spd, brake=False)
                #         obstacleNear = False
                #         evadeLeft = False
                #         evadeRight = False

            elif beaconLock:   # if we have a good lock on the ir beacon, use this for beacon Nav
                # beaconSearch = False
                # speed and steering controlled by beacon Nav
                # set speed (medium)
                spd = 20
                # use beacon heading to correct steering
                # correct left
                if irHeadVal < -24:
                    turnRatio = 0.5
                elif irHeadVal < -20:
                    turnRatio = 0.4
                elif irHeadVal < -15:
                    turnRatio = 0.3
                elif irHeadVal < -10:
                    turnRatio = 0.2
                elif irHeadVal < -5:
                    turnRatio = 0.1
                # correct right
                elif irHeadVal > 24:
                    turnRatio = -0.5
                elif irHeadVal > 20:
                    turnRatio = -0.4
                elif irHeadVal > 15:
                    turnRatio = -0.3
                elif irHeadVal > 10:
                    turnRatio = -0.2
                elif irHeadVal > 5:
                    turnRatio = -0.1
                else:
                    turnRatio = 0
                
            elif cmpGenHeading:  # if compass Nav has us going mostly North, use compass Nav
                # beaconSearch = True
                # speed (slow) and steering controlled by compass Nav
                # set speed (slow)
                spd = 15   #####  set to 0 for testing
                # use compass heading to correct steering
                # correct left
                if compassVal > (compassGoal + 25) :
                    turnRatio = 0.5
                elif compassVal > (compassGoal + 20):
                    turnRatio = 0.3
                elif compassVal > (compassGoal + 15):
                    turnRatio = 0.2
                elif compassVal > (compassGoal + 10):
                    turnRatio = 0.1
                # correct right
                elif compassVal < (compassGoal - 25):
                    turnRatio = -0.5
                elif compassVal < (compassGoal - 20):
                    turnRatio = -0.3
                elif compassVal < (compassGoal - 15):
                    turnRatio = -0.2
                elif compassVal < (compassGoal - 10):
                    turnRatio = -0.1
                else:
                    turnRatio = 0
                
            ### new SLAM routine; initial 360 deg data capture
            elif searchSLAM1:  # if we have not yet circled for initial SLAM, do so now
                ### comment this code block and uncomment the below section for stationary testing
                # set speed (slow)
                spd = 10
                # initiate a CW 360 deg circle while capturing SLAM data; progress in segments to ensure full circle
                turnRatio = -0.5
                if i == 0:
                    print("Initiating polarCoords capture... ", polarCoords, i)
                    # turn to compass val between 350 and 360 to initialize rotational position
                    if compassVal > (350) and compassVal < (360):
                        i = 1
                elif i == 1:
                    # turn to compass val 36 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 2
                elif i == 2:
                    # turn to compass val 72 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 3
                elif i == 3:
                    # turn to compass val 108 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 4
                elif i == 4:
                    # turn to compass val 144 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 5
                elif i == 5:
                    # turn to compass val 180 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 6
                elif i == 6:
                    # turn to compass val 216 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 7
                elif i == 7:
                    # turn to compass val 252 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 8
                elif i == 8:
                    # turn to compass val 288 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 9
                elif i == 9:
                    # turn to compass val 324 +/- 5
                    if compassVal > ((i * 36) -5) and compassVal < ((i * 36) +5):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, i)
                        i = 10                        
                elif i == 10:
                    # turn to compass val between 350 and 360
                    if compassVal > (350) and compassVal < (360):
                        polarCoords.append(usDistCmVal)
                        i = 0
                        searchSLAM1 = False
                        print("SLAM1 data collection complete")
                        ### added for initial SLAM testing
                        # stop and return to human pilot mode
                        mL.on(0, brake=False)
                        mR.on(0, brake=False)
                        print("polarCoords:", polarCoords)
                        pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
                else:
                    i = 0

            else:  # default action is to find North (compassGoal = ~275) and head that way while looking for beacon                
                cmpSearch = True
                # set speed (slow)
                spd = 10
                if compassVal > (compassGoal - 10) and compassVal < (compassGoal + 10):
                    cmpGenHeading = True
                    cmpSearch = False
                    print("We found North. Done with cmpSearch")
                # determine shortest path to circle North - CW or CCW
                elif (compassVal > (compassGoal + 10)) or (compassVal < (compassGoal - 180)):
                    turnRatio = 0.5
                    print("Doing cmpSearch - turing Left")
                elif (compassVal < (compassGoal - 10)) or (compassVal >= (compassGoal - 180)):
                    turnRatio = -0.5
                    print("Doing cmpSearch - turing Right")
                

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

        # if x != 0:
        #     print("x, spd, turnRatio, mLspd, mRspd, pilot_mode  ", x, spd, turnRatio, mLspd, mRspd, pilot_mode)
        
        if navPrint == 10:
            #print("NAV:  x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock  ", x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock)
            print("NAV:  x, spd, turnRatio, pilot_mode, searchSLAM1, cmpSearch, cmpGenHeading, beaconLock  ", x, spd, turnRatio, pilot_mode, searchSLAM1, cmpSearch, cmpGenHeading, beaconLock)
            navPrint = 0
        navPrint += 1
        # if (obstacleNear or obstacleMed):
        #     print("obstacleNear ", obstacleNear,  "obstacleMed ", obstacleMed )
        if obstacleNear:
            print("obstacleNear ", obstacleNear)
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(0.2)

