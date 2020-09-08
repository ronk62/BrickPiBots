#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 7/4/2020      Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMdataAccurV5.01b as starting point
#                           - this program will allow collection of IR distance data along with motor encoder ticks and compass angle to help
#                             refine dead reckoning and IR-Compass triangulation routines.
#                           - Numpy arrays and tools will be used to capture and analyze the data samples
#                           - circular routes (simple pivont turns - for now - with only one motor running)
#
# 7/28/2020     Ron King    - removed more stubbed out code
#

""" 
This program provides SLAM testing with wasd keyboard remote control and data collection.

Motors

motor A = Left motor
motor B = Right motor

 """

import time, tty, sys, threading
import numpy as np
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
p1.set_device = 'lego-ev3-us'


# Connect lego-ev3-us to any sensor port
us = UltrasonicSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

usDistCmVal = 0
prev_usDistCmVal = 0



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
compassGoal = 0     # with ht-compass mounted face-front, this translates to North



p3 = LegoPort(INPUT_3)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p3.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p3.set_device = 'lego-ev3-ir'


# Connect infrared to sensor port
ir = InfraredSensor(INPUT_3)

# allow for some time to load the new drivers
time.sleep(0.5)

# irProxVal = 100
# prev_irProxVal = 0
irDistVal = 0
prev_irDistVal = 0
irHeadVal = 0
prev_irHeadVal = 0


# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
sample_mode = -1        # start in sample mode off, '-1'; '1' means sample mode is enabled
obstacleNear = False    # there is an obstacle nearby - need to stop, turn, backup, etc (and go another way)
backingUp = False       # use this to control state of obstacle avoidance maneuvers
backingUpTime = 0.0     # use to control how long we backup
searchSLAM1 = False      # are we exexcuting the initial SLAM circle?
cmpGenHeading = False   # do we have a lock on compass heading for North?
cmpSearch = False       # are we searching for North with the compass right now?
navPrint = 10           # print the Nav msgs every nth cycle
printVerbose = 1        # toggle verbose data/msg printing to terminal (-1 is 'disabled')
i = 0                   # "outer" iterator for dist point sampling
j = 0                   # "inner" iterator for dist point sampling
prev_mRposition = 0     # var to help detect when to save a new OD pair (motor pos, cmpVal) to array or print to screen

# SLAM vars
polarCoordsUSr = np.array([], dtype=np.int32)       # arrary to hold 'r' - US distance r (in cm)
polarCoordscmpDeg = np.array([], dtype=np.int32)    # arrary to hold 'thetaDeg' - compass-angle  in degrees
# added IR
polarCoordsIRr = np.array([], dtype=np.int32)       # arrary to hold 'r' - IR distance r (in arb units)
# added array motor encoder ticks for odometry
ODmRencoderVal = np.array([], dtype=np.int32)       # arrary to hold right motor encoder-count while moving
# added another array for compass odometry
ODcmpDeg = np.array([], dtype=np.int32)             # array to hold associated compass angle (degrees) for each motor tick


# setup motors
mL = LargeMotor(OUTPUT_A)
time.sleep(0.5)

mL.reset()
time.sleep(2)
mLspd = 0
mL.stop_action = 'coast'
# mL.polarity = 'inversed'
mL.position = 0
 

mR = LargeMotor(OUTPUT_B)
time.sleep(0.5)
mR.reset()
time.sleep(2)
mRspd = 0
mR.stop_action = 'coast'
# mR.polarity = 'inversed'
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
            if printVerbose > 0:
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        irDistVal = ir.distance()
        irHeadVal = ir.heading()
        if (irDistVal != prev_irDistVal):
            if printVerbose > 0:
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
            prev_irDistVal = irDistVal
            prev_irHeadVal = irHeadVal


        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            if printVerbose > 0:
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal
            # if beaconLock is False and we're outside the compass North range, reset cmpGenHeading to False
            if compassVal < (compassGoal - 80) or compassVal > (compassGoal + 80):
                cmpGenHeading = False

        if mR.position != prev_mRposition:
            prev_mRposition = mR.position
            if printVerbose > 0:
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed

        ### if usDistCmVal < 50:
        if usDistCmVal < 5:    ### changed from '10' to '5' for US data accuracy testing
            if obstacleNear == False:
                print("Obstacle Detected! Forward motion stopped.")
                if spd > 0:
                    spd = 0


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
            if turnRatio > -1:
                turnRatio = turnRatio - 0.1
        if x == 97: # a key pushed (turn more to the Left)
            if turnRatio < 1:
                turnRatio = turnRatio + 0.1
        
        if x == 118: # v pushed - toggle verbosity on and off
            printVerbose *= -1
            print("toggled printVerbose mode (v)")

        if x == 112: # p pushed - print sample arrays to screen
            if polarCoordsUSr.size < 1 or polarCoordscmpDeg.size < 1 or polarCoordsIRr.size < 1 or ODmRencoderVal.size < 1 or ODcmpDeg.size < 1:
                print("no data at this time")
            else:
                print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
                print("")
                print ("polarCoordsUSr = ", polarCoordsUSr)
                print("")
                print ("polarCoordsIRr.size = ", polarCoordsIRr.size)
                print("")
                print ("polarCoordsIRr = ", polarCoordsIRr)
                print("")
                print ("ODmRencoderVal.size = ", ODmRencoderVal.size)
                print("")
                print ("ODmRencoderVal = ", ODmRencoderVal)
                print("")
                print ("ODcmpDeg.size = ", ODcmpDeg.size)
                print("")
                print ("ODcmpDeg = ", ODcmpDeg)
                print("")
                print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
                print("")
                print ("polarCoordscmpDeg = ", polarCoordscmpDeg)
                print("")

        if x == 102: # f pushed - save sample data to files
            print("")
            if polarCoordsUSr.size < 1 or polarCoordscmpDeg.size < 1 or polarCoordsIRr.size < 1 or ODmRencoderVal.size < 1 or ODcmpDeg.size < 1:
                print("no data at this time")
            else:
                print ("saving sample data to csv files")
                # save to csv files
                np.savetxt('polarCoordsUSrV601b.csv', polarCoordsUSr, delimiter=',')
                np.savetxt('polarCoordsIRrV601b.csv', polarCoordsIRr, delimiter=',')
                np.savetxt('ODmRencoderValV601b.csv', ODmRencoderVal, delimiter=',')
                #np.savetxt('ODcmpDegV601b.csv', ODcmpDeg, delimiter=',')
                np.savetxt('polarCoordscmpDegV601b.csv', polarCoordscmpDeg, delimiter=',')
                print ("saves commplete")
                print("")

        if x == 114: # r pushed - reinitializing arrays
            polarCoordsUSr = np.array([], dtype=np.int32)
            polarCoordscmpDeg = np.array([], dtype=np.int32)
            polarCoordsIRr = np.array([], dtype=np.int32)
            ODmRencoderVal = np.array([], dtype=np.int32)
            ODcmpDeg = np.array([], dtype=np.int32)
            print("r pushed - reinitialized polarCoords* arrays")

        if x == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        
        if sample_mode > 0:
            try:
                # init the data collection arrays for each sample collection
                polarCoordsUSr = np.array([], dtype=np.int32)
                polarCoordscmpDeg = np.array([], dtype=np.int32)
                polarCoordsIRr = np.array([], dtype=np.int32)
                ODmRencoderVal = np.array([], dtype=np.int32)
                ODcmpDeg = np.array([], dtype=np.int32)

                # set motor encoder to 0
                mR.position = 0
                prev_mRposition = 0

                # take samples while moving
                while usDistCmVal > 20:
                    #mL.on(5, brake=False)
                    mR.on(5, brake=False)
                    usDistCmVal = us.distance_centimeters
                    irDistVal = ir.distance()
                    if irDistVal == None:
                        irDistVal = -1      ### set to -1 instead of None or numpy.savetxt will complain
                    compassVal = cmp.value(0)
                    polarCoordsUSr = np.append(polarCoordsUSr, usDistCmVal)
                    polarCoordsIRr = np.append(polarCoordsIRr, irDistVal)
                    polarCoordscmpDeg = np.append(polarCoordscmpDeg, compassVal)
                    #if mR.position != prev_mRposition:
                    #    prev_mRposition = mR.position
                    ODmRencoderVal = np.append(ODmRencoderVal, mR.position)
                    ODcmpDeg = np.append(ODcmpDeg, compassVal)
                    if polarCoordsUSr.size > 100000:
                        sample_mode = -1
                        print("")
                        print("")
                        print ("Exiting sample mode due to sample size too large...")
                        print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
                        print("")
                    time.sleep(.05)             
                mL.on(0, brake=True)
                mR.on(0, brake=True)
                print("")
                print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
                print("")
                print ("polarCoordsUSr = ", polarCoordsUSr)
                print("")
                print ("polarCoordsIRr.size = ", polarCoordsIRr.size)
                print("")
                print ("polarCoordsIRr = ", polarCoordsIRr)
                print("")
                print ("ODmRencoderVal.size = ", ODmRencoderVal.size)
                print("")
                print ("ODmRencoderVal = ", ODmRencoderVal)
                print("")
                print ("ODcmpDeg.size = ", ODcmpDeg.size)
                print("")
                print ("ODcmpDeg = ", ODcmpDeg)
                print("")
                print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
                print("")
                print ("polarCoordscmpDeg = ", polarCoordscmpDeg)
                print("")
                print("")
                
            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                sample_mode = -1

            sample_mode = -1
            print("Sampling complete. Returning to keyboard cmd mode.")


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
            # print("NAV:  x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock  ", x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock)
            # print("NAV:  x, spd, turnRatio, pilot_mode, searchSLAM1, cmpSearch, cmpGenHeading, beaconLock  ", x, spd, turnRatio, pilot_mode, searchSLAM1, cmpSearch, cmpGenHeading, beaconLock)
            if printVerbose > 0:
                print("NAV:  x, spd, turnRatio, sample_mode, searchSLAM1, cmpSearch, cmpGenHeading  ", x, spd, turnRatio, sample_mode, searchSLAM1, cmpSearch, cmpGenHeading)
            navPrint = 0
        navPrint += 1
        # if (obstacleNear or obstacleMed):
        #     print("obstacleNear ", obstacleNear,  "obstacleMed ", obstacleMed )
        if obstacleNear:
            print("obstacleNear ", obstacleNear)
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(0.2)
