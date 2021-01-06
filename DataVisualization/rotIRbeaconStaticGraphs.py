#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 1/5/2021      Ron King    - created new based on BPDDwasdSLAMdataAccurV9.12b.py
#                           - this version provides data collection and graphing in the scheme where
#                             the beacon rotates 360 continuously and the IR sensor is manually pointed
#                             at various angles in relation to beacon
#                           - initally, well gather 40 seconds of data while the beacon rotates
#                             ...at the current rotation rate, that will be 4 beacon rotations

#                           - DON'T Forget to start xming and export DISPLAY=10.0.0.9:0.0  (change IP addr as req'd)


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

#ir1ProxVal = 100
#prev_ir1ProxVal = 0
ir1DistVal = 0
prev_ir1DistVal = 0
ir1HeadVal = 0
prev_ir1HeadVal = 0
ir1Scale = 1


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
USr = np.array([], dtype=np.int32)       # array to hold 'r' - US distance r (in cm)
cmpDeg = np.array([], dtype=np.int32)    # array to hold 'thetaDeg' - compass-angle  in degrees
irData = np.array([[],[],[]], dtype=np.int32)    # array to hold merged data
ODt = np.array([], dtype=np.int32)       # array to index each time/tick (x axis independant var)
# added IR1
ir1r = np.array([], dtype=np.int32)       # array to hold 'r' - ir1 distance r (in approx cm)
ir1h = np.array([], dtype=np.int32)       # array to hold 'h' - ir1 heading h (in +/- deg off-center)


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
                print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        ir1DistVal = ir.distance(channel=1)
        if ir1DistVal == None:
            ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
        else:
            ir1DistVal = int(ir1Scale * ir1DistVal)
        ir1HeadVal = ir.heading(channel=1)
        if (ir1DistVal != prev_ir1DistVal):
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_ir1DistVal = ir1DistVal
            prev_ir1HeadVal = ir1HeadVal


        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
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


        if x == 102: # f pushed - save sample statistics to file
            print("")
            if irData.size < 1:
                print("no data at this time")
            else:
                print ("saving sample data to csv files")
                # save to csv files
                np.savetxt('rotIRbeaconStaticGraphs20210105-1.csv', irData, delimiter=',')
                print ("saves commplete")
                print("")


        if x == 108: # l pushed - loading sample data from csv files
            print("")
            print ("loading sample data from csv files")
            # load from csv files
            irData = np.loadtxt('/home/robot/ev3dev2Projects/DataVisualization/Datafiles/rotIRbeaconStaticGraphs20210105-1.csv', delimiter=',')
            print ("loads commplete, extracting irData into separate arrays")
            print("")
            

        if x == 103: # g pushed - graph the data
            plt.figure(1)
            plt.plot(ODt,ir1r, label='ir1r')
            plt.plot(ODt,ir1h, label='ir1h')
            
            plt.xlabel('ODt')
            plt.ylabel('sensor values')
            plt.title('sensor data Jan 2021 rotIRbeacon') 
            plt.legend()
            plt.show()

            
            
            ####
            # plt.figure(2)
            # plt.plot(ODt,cmpMean, label='cmpMean')
            # plt.plot(ODt,ir2rmean, label='ir2rmean')
            # plt.plot(ODt,lpfir2rmean, label='lpfir2rmean')
            # plt.plot(ODt,ir2hmeanX10, label='ir2hmeanX10')                        
            # plt.plot(ODt,ir2rstdX10, label='ir2rstdX10')
            # plt.plot(ODt,ir2hstdX10, label='ir2hstdX10')
            # # plt.plot(ODt,ir2hmean, label='ir2hmean')
            # plt.plot(ODt,lpfir2hmean, label='lpfir2hmean')

            # plt.xlabel('ODt')
            # plt.ylabel('sensor values')
            # plt.title('sensor data V9x2b')      ### csv input files are V902 1-4 and V912 5-7
            # plt.legend()
            # plt.show()


        if x == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        
        if sample_mode > 0:
            try:
                # initialize arrays and vars
                irData = np.array([[],[],[]], dtype=np.int32)  # initialize/clear here
                tic = time.time()  # var for timing functions
                
                # take samples for n seconds (start with 40)
                print("taking samples for 40 seconds...  ")
                while (time.time() - tic < 41):
                    # usDistCmVal = us.distance_centimeters
                    ir1DistVal = ir.distance(channel=1)
                    if ir1DistVal == None:
                        ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                    ir1HeadVal = ir.heading(channel=1)

                    # update arrays
                    ODt = np.append(ODt, (time.time() - tic))
                    ir1r = np.append(ir1r, ir1DistVal)
                    ir1h = np.append(ir1h, ir1HeadVal)
                    # compassVal = cmp.value(0)
                    # USr = np.append(USr, usDistCmVal)
                    # cmpDeg = np.append(cmpDeg, compassVal)

                    if ODt.size > 100000:
                        sample_mode = -1
                        print("")
                        print("")
                        print ("Exiting sample mode due to sample size too large...")
                        print ("ODt.size = ", ODt.size)
                        print("")


                # combine all the data from the arrays
                # print("")
                # print ("merging data into irData ... ")
                # for i in range(len(ODt)):
                #     a = ODt[i]
                #     b = ir1r[i]
                #     c = ir1h[i]
                #     irData = np.append(irData, [[a], [b], [c]], axis=0)

                print("")
                print ("ODt.size = ", ODt.size)
                print("min      max      mean      std")
                print(np.min(ODt), np.max(ODt), np.mean(ODt), np.std(ODt))
                print("")
                print ("ir1r.size = ", ir1r.size)
                print("min      max      mean      std")
                print(np.min(ir1r), np.max(ir1r), np.mean(ir1r), np.std(ir1r))
                print("")
                print ("ir1h.size = ", ir1h.size)
                print("min      max      mean      std")
                print(np.min(ir1h), np.max(ir1h), np.mean(ir1h), np.std(ir1h))
                print("")
                print("")
                # print ("irData.size = ", irData.size)
                # print("")
                # print ("irData.shape = ", irData.shape)
                # print("")

                
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
