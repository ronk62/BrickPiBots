#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 7/2/2020      Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMdataAccurV2.01b.py as starting point
#                           - this program will allow collection of US distance data at 4 compass angles from 9 specific locations 
#                             in dwnstrs b-room room. Numpy arrays and tools will be used to capture and analyze the data samples
#                           - the 4 specific angles are: 40, 120, 225, 325  (roughly perpendicular to related walls)
#                           - location series will start at 40cm from SE wall, 30cm from S wall (door)
#                           - subsequent locations with be at 15cm increments northerly (from wall)     
#                           - generic autonomous driving mode features and code were removed along with other vestigial code
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



""" p3 = LegoPort(INPUT_3)
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
prev_irHeadVal = 0 """


# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
sample_mode = -1        # start in sample mode off, '-1'; '1' means sample mode is enabled
obstacleNear = False    # there is an obstacle nearby - need to stop, turn, backup, etc (and go another way)
backingUp = False       # use this to control state of obstacle avoidance maneuvers
backingUpTime = 0.0     # use to control how long we backup
searchSLAM1 = False      # are we exexcuting the initial SLAM circle?
# beaconLock = False      # has the beacon been found and data read?
cmpGenHeading = False   # do we have a lock on compass heading for North?
# beaconSearch = False    # are we searching for the beacon right now?
cmpSearch = False       # are we searching for North with the compass right now?
navPrint = 10           # print the Nav msgs every nth cycle
printVerbose = 1        # toggle verbose data/msg printing to terminal (-1 is 'disabled')
i = 0                   # "outer" iterator for dist point sampling
j = 0                   # "inner" iterator for dist point sampling
a = 0                   # iteration var for compass angle

# SLAM vars
SEWALLDIST = 40                                     # distanve from main reference wall (SE wall)
NEwalldeg = 40                                      # angle in deg to NE wall (i.e. the sink cabinet)
SEwalldeg = 120                                     # angle in deg to SE wall
sDoordeg = 225                                      # angle in deg to southern "wall" (i.e. the door)
SWwalldeg = 325                                     # angle in deg to SW (curved) wall
polarCoordsUSr = np.array([], dtype=np.int32)       # array to hold 'r' - US distance r (in cm)
polarCoordscmpDeg = np.array([], dtype=np.int32)    # array to hold 'thetaDeg' - compass-angle  in degrees
polarCoordsUSmean = np.array([], dtype=np.int32)    # array to hold processed data US-mean
polarCoordsUSstd = np.array([], dtype=np.int32)     # array to hold processed data US-stdev
polarCoordscmpMean = np.array([], dtype=np.int32)   # array to hold processed data cmps-mean

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
                print("usDistCmVal = ", int(usDistCmVal), "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        """ irDistVal = ir.distance()
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
                beaconLock = False """

        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            if printVerbose > 0:
                print("usDistCmVal = ", int(usDistCmVal), "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal
            # if beaconLock is False and we're outside the compass North range, reset cmpGenHeading to False
            if compassVal < (compassGoal - 80) or compassVal > (compassGoal + 80):
                cmpGenHeading = False

        ### if usDistCmVal < 50:
        if usDistCmVal < 5:    ### changed from '10' to '5' for US data accuracy testing
            if obstacleNear == False:
                print("Obstacle Detected! Forward motion stopped.")
                if spd > 0:
                    spd = 0
                    # if pilot_mode < 0:
                    #     # stop and return to human pilot mode
                    #     mL.on(0, brake=False)
                    #     mR.on(0, brake=False)
                    #     ### added for initial SLAM testing
                    #     pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
                    #     print("obstacle Near - returning to human pilot mode")
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
            if turnRatio > -1:
                turnRatio = turnRatio - 0.1
        if x == 97: # a key pushed (turn more to the Left)
            if turnRatio < 1:
                turnRatio = turnRatio + 0.1
        
        if x == 118: # v pushed - toggle verbosity on and off
            printVerbose *= -1
            print("toggled printVerbose mode (v)")

        if x == 112: # p pushed - print sample statistics to screen
            if polarCoordsUSmean.size < 1 or polarCoordsUSstd.size < 1 or polarCoordscmpMean.size < 1:
                print("no data at this time")
            else:
                print ("polarCoordsUSmean.size = ", polarCoordsUSmean.size)
                print("")
                print ("polarCoordsUSmean = ", polarCoordsUSmean)
                print("")
                print ("polarCoordsUSstd.size = ", polarCoordsUSstd.size)
                print("")
                print ("polarCoordsUSstd = ", polarCoordsUSstd)
                print("")
                print ("polarCoordscmpMean.size = ", polarCoordscmpMean.size)
                print("")
                print ("polarCoordscmpMean = ", polarCoordscmpMean)
                print("")

        if x == 102: # f pushed - save sample statistics to file
            print("")
            if polarCoordsUSmean.size < 1 or polarCoordsUSstd.size < 1 or polarCoordscmpMean.size < 1:
                print("no data at this time")
            else:
                print ("saving sample data to csv files")
                # save to csv files
                np.savetxt('polarCoordsUSmeanV401b.csv', polarCoordsUSmean, delimiter=',')
                np.savetxt('polarCoordsUSstdV401b.csv', polarCoordsUSstd, delimiter=',')
                np.savetxt('polarCoordscmpMeanV401b.csv', polarCoordscmpMean, delimiter=',')
                print ("saves commplete")
                print("")

        if x == 114: # r pushed - reinitializing arrays
            polarCoordsUSr = np.array([], dtype=np.int32)
            polarCoordscmpDeg = np.array([], dtype=np.int32)
            polarCoordsUSmean = np.array([], dtype=np.int32)
            polarCoordsUSstd = np.array([], dtype=np.int32)
            polarCoordscmpMean = np.array([], dtype=np.int32)
            print("r pushed - reinitialized polarCoords* arrays")

        if x == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        
        if sample_mode > 0:
            try:
                # cycle through positions and take data samples
                for i in range(30,151,15):        ### cycle through increasing distances from door
                    ## move BPDDbot to potision i
                    ### rotate to face SE wall
                    print("moving to position ", i, "  ...rotating to face SE wall")
                    while compassVal < SEwalldeg - 1 or compassVal > SEwalldeg + 1:
                        compassVal = cmp.value(0)
                        #print("compasVal = ", compassVal)
                        mL.on(3, brake=False)
                    mL.on(0, brake=True)
                    time.sleep(.25)
                    print("compassVal =  ", compassVal)
                    usDistCmVal = us.distance_centimeters
                    print("usDistCmVal =  ", usDistCmVal)
                    
                    ## drive to distance SEWALLDIST (40cm)
                    print("moving to position ", i, "  ...driving to correct distance from SE wall")
                    while usDistCmVal < SEWALLDIST - 1 or usDistCmVal > SEWALLDIST + 1:
                        usDistCmVal = us.distance_centimeters
                        if usDistCmVal < SEWALLDIST - 1:
                            mL.on(-5, brake=False)
                            mR.on(-5, brake=False)
                        if usDistCmVal > SEWALLDIST + 1:
                            mL.on(5, brake=False)
                            mR.on(5, brake=False)
                    mL.on(0, brake=True)
                    mR.on(0, brake=True)

                    ### rotate to face southern "wall" (i.e. the door)
                    print("moving to position ", i, "  ...rotating to face southern 'wall' (i.e. the door)")
                    while compassVal < sDoordeg - 1 or compassVal > sDoordeg + 1:
                        compassVal = cmp.value(0)
                        #print("compasVal = ", compassVal)
                        mL.on(3, brake=False)
                    mL.on(0, brake=True)
                    time.sleep(.25)
                    print("compassVal =  ", compassVal)
                    usDistCmVal = us.distance_centimeters
                    print("usDistCmVal =  ", usDistCmVal)

                    ## drive to distance i
                    print("moving to position ", i, "  ...driving to correct distance from southern wall")
                    while usDistCmVal < i - 1 or usDistCmVal > i + 1:
                        usDistCmVal = us.distance_centimeters
                        if usDistCmVal < i - 1:
                            mL.on(-5, brake=False)
                            mR.on(-5, brake=False)
                        if usDistCmVal > i + 1:
                            mL.on(5, brake=False)
                            mR.on(5, brake=False)
                    mL.on(0, brake=True)
                    mR.on(0, brake=True)
                    time.sleep(.25)

                    ## recheck distance relative to SE wall
                    ### rotate to face SE wall
                    print("moving to position ", i, "  ...rotating to face SE wall for recheck")
                    while compassVal < SEwalldeg - 1 or compassVal > SEwalldeg + 1:
                        compassVal = cmp.value(0)
                        #print("compasVal = ", compassVal)
                        mL.on(-3, brake=False)
                    mL.on(0, brake=True)
                    time.sleep(.25)
                    print("compassVal =  ", compassVal)
                    usDistCmVal = us.distance_centimeters
                    print("usDistCmVal =  ", usDistCmVal)
                    
                    ## drive to distance SEWALLDIST (40cm)
                    print("moving to position ", i, "  ...driving to correct distance from SE wall (receck)")
                    while usDistCmVal < SEWALLDIST - 1 or usDistCmVal > SEWALLDIST + 1:
                        usDistCmVal = us.distance_centimeters
                        if usDistCmVal < SEWALLDIST - 1:
                            mL.on(-5, brake=False)
                            mR.on(-5, brake=False)
                        if usDistCmVal > SEWALLDIST + 1:
                            mL.on(5, brake=False)
                            mR.on(5, brake=False)
                    mL.on(0, brake=True)
                    mR.on(0, brake=True)
                    time.sleep(.25)

                    ### rotate CCW to NEwalldeg -10 deg to setup for data collection; speed related addition
                    print("moving CCW to NEwalldeg -10 deg to setup for data collection (speed related addition)")
                    while compassVal < NEwalldeg - 11 or compassVal > NEwalldeg - 9:
                        compassVal = cmp.value(0)
                        #print("compasVal = ", compassVal)
                        mL.on(-6, brake=False)
                    mL.on(0, brake=True)
                    time.sleep(.25)
                    print("compassVal =  ", compassVal)
                    usDistCmVal = us.distance_centimeters
                    print("usDistCmVal =  ", usDistCmVal)

                    # rotate to the four reference angles (point at the walls) and gather sample data
                    print("rotating to the four reference angles (point at the walls) and gathering sample data")
                    for a in (NEwalldeg, SEwalldeg, sDoordeg, SWwalldeg):
                        while compassVal < a - 1 or compassVal > a + 1:
                            compassVal = cmp.value(0)
                            #print("compasVal = ", compassVal)
                            mL.on(3, brake=False)
                        mL.on(0, brake=True)
                        time.sleep(2)   # pause to let the shaking stop before collecting data

                        # init the data collection arrays for each angle/collection series
                        polarCoordsUSr = np.array([], dtype=np.int32)
                        polarCoordscmpDeg = np.array([], dtype=np.int32)
                        # take 100 samples
                        for j in range(100):
                            usDistCmVal = us.distance_centimeters
                            compassVal = cmp.value(0)
                            polarCoordsUSr = np.append(polarCoordsUSr, usDistCmVal)
                            polarCoordscmpDeg = np.append(polarCoordscmpDeg, compassVal)
                        print("")
                        print("value of a = ", a)
                        print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
                        print("min      max      mean      std")
                        print(np.min(polarCoordsUSr), np.max(polarCoordsUSr), np.mean(polarCoordsUSr), np.std(polarCoordsUSr))
                        print("")
                        print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
                        print("min      max      mean      std")
                        print(np.min(polarCoordscmpDeg), np.max(polarCoordscmpDeg), np.mean(polarCoordscmpDeg), np.std(polarCoordscmpDeg))
                        print("")
                        polarCoordsUSmean = np.append(polarCoordsUSmean, np.mean(polarCoordsUSr))
                        polarCoordsUSstd = np.append(polarCoordsUSstd, np.std(polarCoordsUSr))
                        polarCoordscmpMean = np.append(polarCoordscmpMean, np.mean(polarCoordscmpDeg))

                        if polarCoordsUSmean.size > 100000:
                            sample_mode = -1
                            print("")
                            print("")
                            print ("Exiting sample mode due to sample size too large...")
                            print ("polarCoordsUSmean.size = ", polarCoordsUSmean.size)
                            print("")
                
            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                sample_mode = -1

            sample_mode = -1
            print("Sampling complete. Returning to keyboard cmd mode.")


        """ if irDistVal is not None:
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
                time.sleep(3) """

        # if pilot_mode < 0:
        #     if  obstacleNear == True:
        #         # obstacleMed = False
        #         if not (backingUp or evadeLeft or evadeRight):
        #             # stop and backup
        #             mL.on(0, brake=False)
        #             mR.on(0, brake=False)
        #             ### added for initial SLAM testing
        #             pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
        #             print("obstacle Near - returning to human pilot mode")
                    
            # elif beaconLock:   # if we have a good lock on the ir beacon, use this for beacon Nav
            #     # beaconSearch = False
            #     # speed and steering controlled by beacon Nav
            #     # set speed (medium)
            #     spd = 20
            #     # use beacon heading to correct steering
            #     # correct left
            #     if irHeadVal < -24:
            #         turnRatio = 0.5
            #     elif irHeadVal < -20:
            #         turnRatio = 0.4
            #     elif irHeadVal < -15:
            #         turnRatio = 0.3
            #     elif irHeadVal < -10:
            #         turnRatio = 0.2
            #     elif irHeadVal < -5:
            #         turnRatio = 0.1
            #     # correct right
            #     elif irHeadVal > 24:
            #         turnRatio = -0.5
            #     elif irHeadVal > 20:
            #         turnRatio = -0.4
            #     elif irHeadVal > 15:
            #         turnRatio = -0.3
            #     elif irHeadVal > 10:
            #         turnRatio = -0.2
            #     elif irHeadVal > 5:
            #         turnRatio = -0.1
            #     else:
            #         turnRatio = 0
                
            # elif cmpGenHeading:  # if compass Nav has us going mostly North, use compass Nav
            #     # beaconSearch = True
            #     # speed (slow) and steering controlled by compass Nav
            #     # set speed (slow)
            #     spd = 15   #####  set to 0 for testing
            #     # use compass heading to correct steering
            #     # correct left
            #     if compassVal > (compassGoal + 25) :
            #         turnRatio = 0.5
            #     elif compassVal > (compassGoal + 20):
            #         turnRatio = 0.3
            #     elif compassVal > (compassGoal + 15):
            #         turnRatio = 0.2
            #     elif compassVal > (compassGoal + 10):
            #         turnRatio = 0.1
            #     # correct right
            #     elif compassVal < (compassGoal - 25):
            #         turnRatio = -0.5
            #     elif compassVal < (compassGoal - 20):
            #         turnRatio = -0.3
            #     elif compassVal < (compassGoal - 15):
            #         turnRatio = -0.2
            #     elif compassVal < (compassGoal - 10):
            #         turnRatio = -0.1
            #     else:
            #         turnRatio = 0
                
            ### new SLAM routine; initial 360 deg data capture
            # elif searchSLAM1:  # if we have not yet circled for initial SLAM, do so now
            #     ### comment this code block and uncomment the below section for stationary testing
            #     # set speed (slow)
            #     spd = 5
            #     # initiate a CW 360 deg circle while capturing SLAM data; progress in segments to ensure full circle
            #     turnRatio = -1
            #     if i == 0:
            #         print("Initiating polarCoords capture... ", polarCoords, i)
            #         # turn to compass val between 350 and 360 to initialize rotational position
            #         if compassVal > (350) and compassVal < (360):
            #             i = 1
            #             j = 1
            #     elif i == 1:                # iterate through 350 deg
            #         # turn to compass val j * 10 +/- 3
            #         if compassVal > ((j * 10) -3) and compassVal < ((j * 10) +3):
            #             polarCoords.append(usDistCmVal)
            #             print("appended polarCoords...  ", polarCoords, j)
            #             j = j + 1
            #             if j == 36:          # iterate all but the final data point
            #                 i = 2           # update state to move to next/final data point collection
            #     elif i == 2:                # collect final data point and reset vars
            #         # turn to compass val 355 +/-3
            #         if compassVal > (352) and compassVal < (358):
            #             polarCoords.append(usDistCmVal)
            #             i = 0
            #             j = 0
            #             searchSLAM1 = False
            #             print("SLAM1 data collection complete")
            #             ### added for initial SLAM testing
            #             # stop and return to human pilot mode
            #             mL.on(0, brake=False)
            #             mR.on(0, brake=False)
            #             print("polarCoords:", polarCoords)
            #             pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
            #     else:
            #         i = 0

            # else:  # default action is to find North (compassGoal = ~275) and head that way while looking for beacon                
            #     cmpSearch = True
            #     # set speed (slow)
            #     spd = 10
            #     if compassVal > (compassGoal - 10) and compassVal < (compassGoal + 10):
            #         cmpGenHeading = True
            #         cmpSearch = False
            #         print("We found North. Done with cmpSearch")
            #     # determine shortest path to circle North - CW or CCW
            #     elif (compassVal > (compassGoal + 10)) or (compassVal < (compassGoal - 180)):
            #         turnRatio = 0.5
            #         print("Doing cmpSearch - turing Left")
            #     elif (compassVal < (compassGoal - 10)) or (compassVal >= (compassGoal - 180)):
            #         turnRatio = -0.5
            #         print("Doing cmpSearch - turing Right")
                

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
