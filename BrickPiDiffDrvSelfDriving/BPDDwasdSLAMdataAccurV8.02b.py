#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 8/2/2020      Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMdataAccurV3.01c as starting point
#                           - this program will allow collection of US distance data, 360 deg rotation at 1 deg increments from a specific 
#                           place in a room. Numpy arrays and tools will be used to capture and analyze the data samples
#                           - generic autonomous driving mode features and code were removed along with other vestigial code
#                           - using IR sensor (distance mode in cm) and two channels with two beacons, which will enable
#                           triangulation of two landmark, and possibly some use of the distance data as well
#                           - also adding ability to plot the data in this same program
#                           - Note: irDistVal is now set to int(3.19 * ir.distance()) to approximate distance in cm
#                           - DON'T Forget to start xming and export DISPLAY=10.0.0.9:0.0  (change IP addr as req'd)
#

""" 
This program provides SLAM testing with wasd keyboard remote control and data collection.

Motors

motor A = Left motor
motor B = Right motor

 """

import time, tty, sys, threading
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, radians
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

#irProxVal = 100
#prev_irProxVal = 0
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
# beaconLock = False      # has the beacon been found and data read?
cmpGenHeading = False   # do we have a lock on compass heading for North?
# beaconSearch = False    # are we searching for the beacon right now?
cmpSearch = False       # are we searching for North with the compass right now?
navPrint = 10           # print the Nav msgs every nth cycle
printVerbose = 1        # toggle verbose data/msg printing to terminal (-1 is 'disabled')
i = 0                   # "outer" iterator for dist point sampling
j = 0                   # "inner" iterator for dist point sampling

# SLAM vars
polarCoordsUSr = np.array([], dtype=np.int32)       # array to hold 'r' - US distance r (in cm)
polarCoordscmpDeg = np.array([], dtype=np.int32)    # array to hold 'thetaDeg' - compass-angle  in degrees
polarCoordsUSmean = np.array([], dtype=np.int32)    # array to hold processed data US-mean
polarCoordsUSstd = np.array([], dtype=np.int32)     # array to hold processed data US-stdev
polarCoordscmpMean = np.array([], dtype=np.int32)   # array to hold processed data cmps-mean
# added IR
polarCoordsIRr = np.array([], dtype=np.int32)       # array to hold 'r' - IR distance r (in approx cm)
polarCoordsIRrmean = np.array([], dtype=np.int32)   # array to hold processed data IRr-mean
polarCoordsIRrstd = np.array([], dtype=np.int32)    # array to hold processed data IRr-stdev
polarCoordsIRh = np.array([], dtype=np.int32)       # array to hold 'h' - IR heading h (in +/- deg off-center)
polarCoordsIRhmean = np.array([], dtype=np.int32)   # array to hold processed data IR heading mean
polarCoordsIRhstd = np.array([], dtype=np.int32)    # array to hold processed data IR heading stdev
ODt = np.array([], dtype=np.int32)                  # array to index each time/tick (x axis independant var)
# for better graphing
polarCoordsUSstdX10 = np.array([], dtype=np.int32)      # array to hold orig array vals * 10
polarCoordsIRrstdX10 = np.array([], dtype=np.int32)     # array to hold orig array vals * 10
polarCoordsIRhmeanX10 = np.array([], dtype=np.int32)    # array to hold orig array vals * 10
polarCoordsIRhstdX10 = np.array([], dtype=np.int32)     # array to hold orig array vals * 10

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
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        irDistVal = ir.distance(channel=1)
        if irDistVal == None:
            irDistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
        else:
            irDistVal = int(3.19 * irDistVal)
        irHeadVal = ir.heading(channel=1)
        if (irDistVal != prev_irDistVal):
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_irDistVal = irDistVal
            prev_irHeadVal = irHeadVal

        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal

        ### halt if usDistCmVal < 5 
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
            if polarCoordsUSmean.size < 1 or polarCoordsUSstd.size < 1 or polarCoordsIRrmean.size < 1 or polarCoordsIRrstd.size < 1 or polarCoordscmpMean.size < 1:
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
                print ("polarCoordsIRrmean.size = ", polarCoordsIRrmean.size)
                print("")
                print ("polarCoordsIRrmean = ", polarCoordsIRrmean)
                print("")
                print ("polarCoordsIRrstd.size = ", polarCoordsIRrstd.size)
                print("")
                print ("polarCoordsIRrstd = ", polarCoordsIRrstd)
                print("")
                print ("polarCoordsIRhmean.size = ", polarCoordsIRhmean.size)
                print("")
                print ("polarCoordsIRhmean = ", polarCoordsIRhmean)
                print("")
                print ("polarCoordsIRhstd.size = ", polarCoordsIRhstd.size)
                print("")
                print ("polarCoordsIRhstd = ", polarCoordsIRhstd)
                print("")
                print ("polarCoordscmpMean.size = ", polarCoordscmpMean.size)
                print("")
                print ("polarCoordscmpMean = ", polarCoordscmpMean)
                print("")

        if x == 102: # f pushed - save sample statistics to file
            print("")
            if polarCoordsUSmean.size < 1 or polarCoordsUSstd.size < 1 or polarCoordsIRrmean.size < 1 or polarCoordsIRrstd.size < 1 or polarCoordscmpMean.size < 1:
                print("no data at this time")
            else:
                print ("saving sample data to csv files")
                # save to csv files
                np.savetxt('polarCoordsUSmean-V802b-5.csv', polarCoordsUSmean, delimiter=',')
                np.savetxt('polarCoordsUSstd-V802b-5.csv', polarCoordsUSstd, delimiter=',')
                np.savetxt('polarCoordsIRrmean-V802b-5.csv', polarCoordsIRrmean, delimiter=',')
                np.savetxt('polarCoordsIRrstd-V802b-5.csv', polarCoordsIRrstd, delimiter=',')
                np.savetxt('polarCoordsIRhmean-V802b-5.csv', polarCoordsIRhmean, delimiter=',')
                np.savetxt('polarCoordsIRhstd-V802b-5.csv', polarCoordsIRhstd, delimiter=',')
                np.savetxt('polarCoordscmpMean-V802b-5.csv', polarCoordscmpMean, delimiter=',')
                print ("saves commplete")
                print("")

        if x == 108: # l pushed - loading sample data from csv files
            print("")
            print ("loading sample data from csv files")
            # load from csv files
            polarCoordsUSmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsUSmean-V802b-5.csv', delimiter=',')
            polarCoordsUSstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsUSstd-V802b-5.csv', delimiter=',')
            polarCoordsIRrmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsIRrmean-V802b-5.csv', delimiter=',')
            polarCoordsIRrstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsIRrstd-V802b-5.csv', delimiter=',')
            polarCoordsIRhmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsIRhmean-V802b-5.csv', delimiter=',')
            polarCoordsIRhstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsIRhstd-V802b-5.csv', delimiter=',')
            polarCoordscmpMean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordscmpMean-V802b-5.csv', delimiter=',')
            print ("loads commplete")
            print("")
            # rescaling raw *std and polarCoordsIRhmean values (multiply by 10) for better graphing
            print("rescaling raw *std and polarCoordsIRhmean values (multiply by 10)...")
            print("")
            for i in range(0,len(polarCoordsUSstd)):
                polarCoordsUSstdX10 = np.append(polarCoordsUSstdX10, (10 * polarCoordsUSstd[i]))
                polarCoordsIRrstdX10 = np.append(polarCoordsIRrstdX10, (10 * polarCoordsIRrstd[i]))
                polarCoordsIRhmeanX10 = np.append(polarCoordsIRhmeanX10, (10 * polarCoordsIRhmean[i]))
                polarCoordsIRhstdX10 = np.append(polarCoordsIRhstdX10, (10 * polarCoordsIRhstd[i]))
            # calculating additional values
            print("creating ODt for use as x axis values for plotting/graphing...")
            print("")
            for i in range(0,len(polarCoordsUSmean)):
                ODt = np.append(ODt, i)
                #print("i = ", i)
            #print("ODt = ", ODt)
            print("")
            print("values have been created")
            print("")


        if x == 114: # r pushed - reinitializing arrays
            polarCoordsUSr = np.array([], dtype=np.int32)
            polarCoordscmpDeg = np.array([], dtype=np.int32)
            polarCoordsUSmean = np.array([], dtype=np.int32)
            polarCoordsUSstd = np.array([], dtype=np.int32)
            polarCoordscmpMean = np.array([], dtype=np.int32)
            polarCoordsIRr = np.array([], dtype=np.int32)
            polarCoordsIRrmean = np.array([], dtype=np.int32)
            polarCoordsIRrstd = np.array([], dtype=np.int32)
            polarCoordsIRh = np.array([], dtype=np.int32)
            polarCoordsIRhmean = np.array([], dtype=np.int32)
            polarCoordsIRhstd = np.array([], dtype=np.int32)
            print("r pushed - reinitialized polarCoords* arrays")

        if x == 103: # g pushed - graph the (mostly) raw data
            plt.plot(ODt,polarCoordsUSmean, label='polarCoordsUSmean')
            plt.plot(ODt,polarCoordsIRrmean, label='polarCoordsIRrmean')
            plt.plot(ODt,polarCoordsIRhmeanX10, label='polarCoordsIRhmeanX10')
            plt.plot(ODt,polarCoordscmpMean, label='polarCoordscmpMean')
            plt.plot(ODt,polarCoordsUSstdX10, label='polarCoordsUSstdX10')
            plt.plot(ODt,polarCoordsIRrstdX10, label='polarCoordsIRrstdX10')
            plt.plot(ODt,polarCoordsIRhstdX10, label='polarCoordsIRhstdX10')

            plt.xlabel('ODt')
            plt.ylabel('sensor values')
            #plt.ylabel('processed data')
            plt.title('sensor data V802b')
            plt.legend()
            plt.show()


        if x == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        
        if sample_mode > 0:
            try:
                # rototate to cmp angle and take data samples
                #for i in range(3):  ### testing
                for i in range(360):
                    ## move motor to new cmps angle
                    while compassVal < i - 1 or compassVal > i + 1:
                        compassVal = cmp.value(0)
                        #print("compasVal = ", compassVal)
                        mL.on(1, brake=False)
                    mL.on(0, brake=True)
                    time.sleep(1)   # pause to let the shaking stop before collecting data
                    # init the data collection arrays for each angle/collection series
                    polarCoordsUSr = np.array([], dtype=np.int32)
                    polarCoordscmpDeg = np.array([], dtype=np.int32)
                    polarCoordsIRr = np.array([], dtype=np.int32)
                    polarCoordsIRh = np.array([], dtype=np.int32)
                    # take 100 samples
                    for j in range(100):
                        usDistCmVal = us.distance_centimeters
                        irDistVal = ir.distance(channel=1)
                        if irDistVal == None:
                            irDistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                        else:
                            irDistVal = int(3.19 * irDistVal)
                        irHeadVal = ir.heading(channel=1)
                        compassVal = cmp.value(0)
                        polarCoordsUSr = np.append(polarCoordsUSr, usDistCmVal)
                        polarCoordsIRr = np.append(polarCoordsIRr, irDistVal)
                        polarCoordsIRh = np.append(polarCoordsIRh, irHeadVal)
                        polarCoordscmpDeg = np.append(polarCoordscmpDeg, compassVal)
                    print("")
                    print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsUSr), np.max(polarCoordsUSr), np.mean(polarCoordsUSr), np.std(polarCoordsUSr))
                    print("")
                    print ("polarCoordsIRr.size = ", polarCoordsIRr.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsIRr), np.max(polarCoordsIRr), np.mean(polarCoordsIRr), np.std(polarCoordsIRr))
                    print("")
                    print ("polarCoordsIRh.size = ", polarCoordsIRh.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsIRh), np.max(polarCoordsIRh), np.mean(polarCoordsIRh), np.std(polarCoordsIRh))
                    print("")
                    print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordscmpDeg), np.max(polarCoordscmpDeg), np.mean(polarCoordscmpDeg), np.std(polarCoordscmpDeg))
                    print("")
                    polarCoordsUSmean = np.append(polarCoordsUSmean, np.mean(polarCoordsUSr))
                    polarCoordsUSstd = np.append(polarCoordsUSstd, np.std(polarCoordsUSr))
                    polarCoordsIRrmean = np.append(polarCoordsIRrmean, np.mean(polarCoordsIRr))
                    polarCoordsIRrstd = np.append(polarCoordsIRrstd, np.std(polarCoordsIRr))
                    polarCoordsIRhmean = np.append(polarCoordsIRhmean, np.mean(polarCoordsIRh))
                    polarCoordsIRhstd = np.append(polarCoordsIRhstd, np.std(polarCoordsIRh))
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
