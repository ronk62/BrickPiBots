#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 9/12/2020     Ron King    - extending and updating BPDDwasdSLAMdataAccurV9.12b
#                           - this program will allow collection of US distance data, 360 deg rotation with
#                             emphisis on data collection speed/efficiency without sacrificing fidelity
#                           - Numpy arrays and tools will be used to capture and analyze the data samples
#                           - generic autonomous driving mode features and code were previously removed
#                           - using IR sensor (distance mode in cm) and two channels with two beacons, which will enable
#                           triangulation of two landmarks, and possibly some use of the distance data as well
#                           - also adding ability to plot the data in this same program
#                           - updated this version to find beacon angle using live, rotation, seek routine
#                           - added time.sleep(0.01) during 100 sample collection
#                           - added collection/computation of standard deviations (IR distance and heading)
#                           - replaced hailmarry compass with hailmarry timeout
#                           - tuned some thresholds and tweaked some settings
#                           - still seeing some strange heading values when beacon is too far left or right,
#                             esp when the beacon is slightly behind - and a short distance from - the IR sensor
#                               ---> to address the heading value problem above, I added a vertical separator in front
#                               of the IR sensor to block unwanted peripheral IR beacon signals; this seems to have
#                               helped significantly
#                           - I noticed I was missing the statement ir.mode = 'IR-SEEK' so I added that (9/19/2020)
#                           - added self-calibration routines for IR sensors
#                           - corrected my implementation stats.linregress (I had effectively reversed x and y)
#                           - removed sections of code related to data array storage and retrieval (save/load csv files)
#                             as well as printing arrays to the screen
#                           - added the calculations to transform robot and beacon postions to world-frame values
#                           - heavily revised the plotting/graphing section to show robot and world frame position data

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
from scipy import stats
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

ir.mode = 'IR-SEEK'
#ir1ProxVal = 100
#prev_ir1ProxVal = 0
ir1DistVal = 0
prev_ir1DistVal = 0
ir1HeadVal = 0
prev_ir1HeadVal = 0

#ir2ProxVal = 100
#prev_ir2ProxVal = 0
ir2DistVal = 0
prev_ir2DistVal = 0
ir2HeadVal = 0
prev_ir2HeadVal = 0


# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
sample_mode = -1        # start in sample mode off, '-1'; '1' means sample mode is enabled
obstacleNear = False    # there is an obstacle nearby - need to stop, turn, backup, etc (and go another way)
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
USmean = np.array([], dtype=np.int32)    # array to hold processed data US-mean
USstd = np.array([], dtype=np.int32)     # array to hold processed data US-stdev
cmpMean = np.array([], dtype=np.int32)   # array to hold processed data cmps-mean
ODt = np.array([], dtype=np.int32)       # array to index each time/tick (x axis independant var)
rot_zeroir1hangle = 400                  # rotated for std cartesian graph and converted to radians
rot_zeroir2hangle = 400                  # rotated for std cartesian graph and converted to radians

# added IR1
ir1r = np.array([], dtype=np.int32)       # array to hold 'r' - ir1 distance r (in approx cm)
ir1rmean = np.array([], dtype=np.int32)   # array to hold processed data ir1r-mean
ir1rstd = np.array([], dtype=np.int32)    # array to hold processed data ir1r-stdev
ir1h = np.array([], dtype=np.int32)       # array to hold 'h' - ir1 heading h (in +/- deg off-center)
ir1hmean = np.array([], dtype=np.int32)   # array to hold processed data ir1 heading mean
ir1hstd = np.array([], dtype=np.int32)    # array to hold processed data ir1 heading stdev

# added IR2
ir2r = np.array([], dtype=np.int32)       # array to hold 'r' - ir2 distance r (in approx cm)
ir2rmean = np.array([], dtype=np.int32)   # array to hold processed data ir2r-mean
ir2rstd = np.array([], dtype=np.int32)    # array to hold processed data ir2r-stdev
ir2h = np.array([], dtype=np.int32)       # array to hold 'h' - ir2 heading h (in +/- deg off-center)
ir2hmean = np.array([], dtype=np.int32)   # array to hold processed data ir2 heading mean
ir2hstd = np.array([], dtype=np.int32)    # array to hold processed data ir2 heading stdev

# IR calibration vars
beaconCali_mode = 0
ir1DistCaliK = 3.2373925234111067
ir1DistCaliOffset = 2.9145308391561286
ir2DistCaliK = 3.2099057918114617
ir2DistCaliOffset = -4.590962433759358



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
                print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        ir1DistVal = ir.distance(channel=1)
        if ir1DistVal == None:
            ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
        else:
            ir1DistVal = int((ir1DistCaliK * ir1DistVal) + ir1DistCaliOffset)
        ir1HeadVal = ir.heading(channel=1)
        if (ir1DistVal != prev_ir1DistVal):
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_ir1DistVal = ir1DistVal
            prev_ir1HeadVal = ir1HeadVal

        ir2DistVal = ir.distance(channel=2)
        if ir2DistVal == None:
            ir2DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
        else:
            ir2DistVal = int((ir2DistCaliK * ir2DistVal) + ir2DistCaliOffset)
        ir2HeadVal = ir.heading(channel=2)
        if (ir2DistVal != prev_ir2DistVal):
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  ir2DistVal = ", ir2DistVal, "  ir2HeadVal = ", ir2HeadVal, "  compassVal = ", compassVal, "  mR.position = ", mR.position)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  ir2DistVal = ", ir2DistVal, "  ir2HeadVal = ", ir2HeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_ir2DistVal = ir2DistVal
            prev_ir2HeadVal = ir2HeadVal

        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            if printVerbose > 0:
                #print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
                print("usDistCmVal = ", int(usDistCmVal), "  ir1DistVal = ", ir1DistVal, "  ir1HeadVal = ", ir1HeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
                print("ir2DistVal = ", ir2DistVal, "  ir2HeadVal = ", ir2HeadVal)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal

        ### halt if usDistCmVal < 5 
        if usDistCmVal < 5:    ### changed from '10' to '5' for US data accuracy testing
            if obstacleNear == False:
                print("Obstacle Detected! Forward motion stopped.")
                if spd > 0:
                    spd = 0
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
            if turnRatio > -1:
                turnRatio = turnRatio - 0.1
        if x == 97: # a key pushed (turn more to the Left)
            if turnRatio < 1:
                turnRatio = turnRatio + 0.1
        
        if x == 118: # v pushed - toggle verbosity on and off
            printVerbose *= -1
            print("toggled printVerbose mode (v)")



        if x == 103: # g pushed - graph the data
            plt.figure(1)
            plt.plot(ODt,USmean, label='USmean')
            # plt.plot(ODt,USstdX10, label='USstdX10')

            plt.plot(ODt,cmpMean, label='cmpMean')
            
            plt.plot(ODt,ir1rmean, label='ir1rmean')
            plt.plot(ODt,lpfir1rmean, label='lpfir1rmean')
            plt.plot(ODt,ir1hmeanX10, label='ir1hmeanX10')
            plt.plot(ODt,ir1rstdX10, label='ir1rstdX10')
            plt.plot(ODt,ir1hstdX10, label='ir1hstdX10')
            # plt.plot(ODt,ir1hmean, label='ir1hmean')
            plt.plot(ODt,lpfir1hmean, label='lpfir1hmean')

            plt.xlabel('ODt')
            plt.ylabel('sensor values')
            plt.title('sensor data V922b')      ### csv input files are V922-n
            plt.legend()
            # plt.show()

            plt.figure(2)
            plt.plot(ODt,USmean, label='USmean')
            # plt.plot(ODt,USstdX10, label='USstdX10')

            plt.plot(ODt,cmpMean, label='cmpMean')

            plt.plot(ODt,ir2rmean, label='ir2rmean')
            plt.plot(ODt,lpfir2rmean, label='lpfir2rmean')
            plt.plot(ODt,ir2hmeanX10, label='ir2hmeanX10')                        
            plt.plot(ODt,ir2rstdX10, label='ir2rstdX10')
            plt.plot(ODt,ir2hstdX10, label='ir2hstdX10')
            # plt.plot(ODt,ir2hmean, label='ir2hmean')
            plt.plot(ODt,lpfir2hmean, label='lpfir2hmean')

            plt.xlabel('ODt')
            plt.ylabel('sensor values')
            plt.title('sensor data V922b')      ### csv input files are V922-n
            plt.legend()

            plt.show()


        if x == 49: # 1 pushed - self-calibration routine for IR1 channel/beacon
            ir1DistVal = ir.distance(channel=1)
            if ir1DistVal == None:
                x == 0
                print("")
                print("beacon1 not found...ir1DistVal =   ", ir1DistVal)
                print("make sure beacon1 is on and in range")
                print("")
                time.sleep(3)
            else:
                sample_mode = 1
                beaconCali_mode = 1
                ir1DistCaliK = 1
                ir1DistCaliOffset = 0
                print("")
                print("sample_mode now set to...  ", sample_mode)
                print("and beaconCali_mode now set to ...  ", beaconCali_mode)
                print("")

        if x == 50: # 2 pushed - self-calibration routine for IR2 channel/beacon
            ir2DistVal = ir.distance(channel=2)
            if ir2DistVal == None:
                x == 0
                print("")
                print("beacon2 not found...ir2DistVal =   ", ir2DistVal)
                print("make sure beacon2 is on and in range")
                print("")
                time.sleep(3)
            else:
                sample_mode = 1
                beaconCali_mode = 2
                ir2DistCaliK = 1
                ir2DistCaliOffset = 0
                print("")
                print("sample_mode now set to...  ", sample_mode)
                print("and beaconCali_mode now set to ...  ", beaconCali_mode)
                print("")


        if x == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            # ensure beaconCali_mode set to 0 (not calibrating)
            beaconCali_mode = 0
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        
        
        ######################

        if sample_mode > 0:
            try:
                if beaconCali_mode == 0:
                    # setup for US data capture
                    # rototate to initial cmp angle 350 +/- 2
                    while compassVal < 348 or compassVal > 352:
                        compassVal = cmp.value(0)
                        if compassVal >= 125 and compassVal < 352:
                            # rotate cw
                            mL.on(7, brake=False)
                        if compassVal < 125 or compassVal > 352:
                            # rotate ccw
                            mL.on(-7, brake=False)
                    mL.on(0, brake=True)
                    time.sleep(0.5)

                    # rotate to angle i; stop and take US data samples
                    for i in range(6,360,6):
                        ## move motor to new cmps angle
                        while compassVal < i - 1 or compassVal > i + 1:
                            compassVal = cmp.value(0)
                            #print("compasVal = ", compassVal)
                            mL.on(3, brake=False)
                        mL.on(0, brake=True)
                        ###time.sleep(1)   # pause to let the shaking stop before collecting data
                        # init the data collection arrays for each angle/collection series
                        USr = np.array([], dtype=np.int32)
                        cmpDeg = np.array([], dtype=np.int32)
                        
                        # take 100 samples
                        for j in range(100):
                            usDistCmVal = us.distance_centimeters
                            compassVal = cmp.value(0)
                            USr = np.append(USr, usDistCmVal)
                            cmpDeg = np.append(cmpDeg, compassVal)
                            
                        print("")
                        print ("USr.size = ", USr.size)
                        print("min      max      mean      std")
                        print(np.min(USr), np.max(USr), np.mean(USr), np.std(USr))
                        print("")
                        print ("cmpDeg.size = ", cmpDeg.size)
                        print("min      max      mean      std")
                        print(np.min(cmpDeg), np.max(cmpDeg), np.mean(cmpDeg), np.std(cmpDeg))
                        print("")
                        print("")
                        
                        USmean = np.append(USmean, np.mean(USr))
                        USstd = np.append(USstd, np.std(USr))
                        cmpMean = np.append(cmpMean, np.mean(cmpDeg))

                        if USmean.size > 100000:
                            sample_mode = -1
                            print("")
                            print("")
                            print ("Exiting sample mode due to sample size too large...")
                            print ("USmean.size = ", USmean.size)
                            print("")
                
            # except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            #     mL.on(0, brake=False)
            #     mR.on(0, brake=False)
            #     sample_mode = -1

            # sample_mode = -1
            # print("Sampling complete. Returning to keyboard cmd mode.")

                ###############################
                ###############################
                ###############################                

                if beaconCali_mode == 1 or beaconCali_mode == 2:
                    # reset/initialize additional beacon cali arrays
                    USmean = np.array([], dtype=np.int32)
                    ir1rmeanCali = np.array([], dtype=np.int32)
                    ir2rmeanCali = np.array([], dtype=np.int32)
                    # calibrate IR distance senors using US dist as ref
                    print("")
                    print("Make sure robot is facing a flat wall or surface with no interference with US")
                    print("and that beacon1 is next to the wall, on, and facing the robot")
                    print("Start at 250cm US distance")
                    print("")
                    # set cycleCount
                    cycleCount = 24  # used during 'for i in range() below, so it iterates 24 times
                    time.sleep(5)


                # initialize another var to measure timing in various segments of the code
                tic = 0

                
                if beaconCali_mode == 0:
                    # getting beacon angle/distance data
                    print("")
                    print("getting beacon angle/distance data")
                    print("")
                    # set cycleCount
                    cycleCount = 2  # used during 'for i in range() below, so this iterates 1 time
                
                for i in range(1,cycleCount):
                    # for Cali cycles, iterate 25 times, moving 10cm closer to IR beacon each time
                    # center the IR heading (close to heading '0' as possible)
                    # then capture US and IR distance data, append to arrays
                    # for non-Cali cycles, iterate just once, gathering IR distance and angle (each beacon)

                    # setup for beacon cal and/or angle/distance data capture
                    if beaconCali_mode == 2:
                        b1lock = 1      # skip b1lock processing; we're doing ir2 cali
                    else:
                        b1lock = 0
                    b1IRdistMean = -1
                    b1IRhMean = 400
                    zeroir1hangle = 400
                    zeroir1hdist = 400
                    if beaconCali_mode == 1:
                        b2lock = 1      # skip b2lock processing; we're doing ir1 cali
                    else:
                        b2lock = 0
                    b2IRdistMean = -1
                    b2IRhMean = 400
                    zeroir2hangle = 400
                    zeroir2hdist = 400

                    # initialize timeout value for each beacon search
                    hailmarryTimeout = time.time()
                    
                    while not b1lock or not b2lock:
                        # init the data collection arrays for each collection series
                        USr = np.array([], dtype=np.int32)
                        cmpDeg = np.array([], dtype=np.int32)
                        ir1r = np.array([], dtype=np.int32)
                        ir1h = np.array([], dtype=np.int32)
                        ir2r = np.array([], dtype=np.int32)
                        ir2h = np.array([], dtype=np.int32)

                        # take 25 samples
                        tic = time.time()       # start the timer
                        for j in range(25):
                            usDistCmVal = us.distance_centimeters
                            ir1DistVal = ir.distance(channel=1)
                            ir2DistVal = ir.distance(channel=2)
                            if ir1DistVal == None:
                                ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                            else:
                                ir1DistVal = int((ir1DistCaliK * ir1DistVal) + ir1DistCaliOffset)
                            ir1HeadVal = ir.heading(channel=1)
                            if ir2DistVal == None:
                                ir2DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                            else:
                                ir2DistVal = int((ir2DistCaliK * ir2DistVal) + ir2DistCaliOffset)
                            ir2HeadVal = ir.heading(channel=2)
                            compassVal = cmp.value(0)
                            USr = np.append(USr, usDistCmVal)
                            cmpDeg = np.append(cmpDeg, compassVal)
                            ir1r = np.append(ir1r, ir1DistVal)
                            ir1h = np.append(ir1h, ir1HeadVal)
                            ir2r = np.append(ir2r, ir2DistVal)
                            ir2h = np.append(ir2h, ir2HeadVal)
                            time.sleep(0.01)
                        b1IRdistMean = np.mean(ir1r)
                        b1IRdistStdev = np.std(ir1r)
                        b1IRhMean = np.mean(ir1h)
                        b1IRhStdev = np.std(ir1h)
                        b2IRdistMean = np.mean(ir2r)
                        b2IRdistStdev = np.std(ir2r)
                        b2IRhMean = np.mean(ir2h)
                        b2IRhStdev = np.std(ir2h)
                        # show timer
                        print("")
                        print("time to take 25 samples, append np arrays, and gather stats:  ", time.time() - tic)
                        print("")

                        # show bxlock states
                        if not b1lock:
                            print("")
                            print("time, b1lock state = ", time.time() - hailmarryTimeout, b1lock)
                            print("b1IRdistMean, b1IRdistStdev = ", b1IRdistMean, b1IRdistStdev)
                            print("b1IRhMean, b1IRhStdev = ", b1IRhMean, b1IRhStdev)
                            print("compassVal = ", compassVal)
                            print("")
                            print("b2lock state = ", b2lock)
                            print("")
                        elif b1lock:
                            print("")
                            print("time, b2lock state = ", time.time() - hailmarryTimeout, b2lock)
                            print("b2IRdistMean, b2IRdistStdev = ", b2IRdistMean, b2IRdistStdev)
                            print("b2IRhMean, b2IRhStdev = ", b2IRhMean, b2IRhStdev)
                            print("compassVal = ", compassVal)
                            print("")
                            print("b2lock state = ", b2lock)
                            print("")


                        ######################

                        # if we don't have the beacon1 angle/distance yet, rotate to find those
                        if not b1lock:
                            if b1IRdistMean > 0 and b1IRhStdev < 3 and b1IRhMean < -20:
                                # move ccw med slow
                                mL.on(-3, brake=False)
                                print("move ccw med slow; b1IRdistMean, b1IRhMean, b1IRhStdev", b1IRdistMean, b1IRhMean, b1IRhStdev)
                            elif b1IRdistMean > 0 and b1IRhStdev < 3 and b1IRhMean < 0:
                                # move ccw slow
                                mL.on(-1, brake=False)
                                print("move ccw slow; b1IRdistMean, b1IRhMean, b1IRhStdev", b1IRdistMean, b1IRhMean, b1IRhStdev)
                            if b1IRdistMean > 0 and b1IRhStdev < 3 and b1IRhMean > 20:
                                # move cw med slow
                                mL.on(3, brake=False)
                                print("move cw med slow; b1IRdistMean, b1IRhMean, b1IRhStdev", b1IRdistMean, b1IRhMean, b1IRhStdev)
                            elif b1IRdistMean > 0 and b1IRhStdev < 3 and b1IRhMean > 0:
                                # move cw slow
                                mL.on(1, brake=False)
                                print("move cw slow; b1IRdistMean, b1IRhMean, b1IRhStdev", b1IRdistMean, b1IRhMean, b1IRhStdev)

                            # if we don't see a valid +25 or -25 heading, turn med fast until we do
                            # note that an ir distance of -1 means we don't even see the beacon
                            # note also that an ir distance > 300 is not trustworthy, so let's limit that
                            if (b1IRdistMean > 300 or b1IRdistMean == -1) and b1IRhMean == 0:
                                # move fast in cw direction until we pick up a beacon signal (or exceed search limit)
                                mL.on(7, brake=False)
                                print("move fast in cw direction; b1IRdistMean, b1IRhMean", b1IRdistMean, b1IRhMean)
                            
                            # stop when heading is between -0.7 and 0.7 AND ir distance is not -1
                            # you should now have a good lock on beacon1, so grab the data
                            if b1IRdistMean > 0 and b1IRdistMean < 300 and b1IRdistStdev < 3 and b1IRhMean >= -0.7 and b1IRhMean <= 0.7 and b1IRhStdev < 3:
                                mL.on(0, brake=True)
                                time.sleep(1)   ## let the shaking stop

                                # init the data collection arrays for each collection series
                                cmpDeg = np.array([], dtype=np.int32)
                                ir1r = np.array([], dtype=np.int32)

                                # take 25 samples
                                for j in range(25):
                                    usDistCmVal = us.distance_centimeters
                                    ir1DistVal = ir.distance(channel=1)
                                    if ir1DistVal == None:
                                        ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                                    else:
                                        ir1DistVal = int((ir1DistCaliK * ir1DistVal) + ir1DistCaliOffset)
                                    compassVal = cmp.value(0)
                                    cmpDeg = np.append(cmpDeg, compassVal)
                                    ir1r = np.append(ir1r, ir1DistVal)
                                    time.sleep(0.01)
                                    USr = np.append(USr, usDistCmVal)
                                
                                if beaconCali_mode == 1:
                                    # grab these data points for Cali arrays
                                    USmean = np.append(USmean, np.mean(USr))
                                    ir1rmeanCali = np.append(ir1rmeanCali, np.mean(ir1r))
                                
                                # grab the beacon1 final output    
                                zeroir1hangle = np.mean(cmpDeg)
                                zeroir1hdist = np.mean(ir1r)
                                print("zeroir1hangle = ", zeroir1hangle)
                                print("zeroir1hdist = ", zeroir1hdist)
                                print("")
                                # set b1lock true (1)
                                b1lock = 1
                                hailmarryTimeout = time.time()    # reset Timeout to setup for beacon2 search
                            
                            if  (time.time() - 60) > hailmarryTimeout:
                                mL.on(0, brake=True)
                                print("Searching for beacon1 too long...")
                                print("Proceeding with search for beacon2")
                                print("")
                                # set b1lock true (1)
                                b1lock = 1
                                # reset hailmarryTimeout to initial values
                                hailmarryTimeout = time.time()

                    
                        ######################

                        # if we DO have the beacon1 angle/distance, but not beacon2, rotate to find those
                        if b1lock and not b2lock:
                            if b2IRdistMean > 0 and b2IRhStdev < 3 and b2IRhMean < -20:
                                # move ccw med slow
                                mL.on(-3, brake=False)
                                print("move ccw med slow; b2IRdistMean, b2IRhMean, b2IRhStdev", b2IRdistMean, b2IRhMean, b2IRhStdev)
                            elif b2IRdistMean > 0 and b2IRhStdev < 3 and b2IRhMean < 0:
                                # move ccw slow
                                mL.on(-1, brake=False)
                                print("move ccw slow; b2IRdistMean, b2IRhMean, b2IRhStdev", b2IRdistMean, b2IRhMean, b2IRhStdev)
                            if b2IRdistMean > 0 and b2IRhStdev < 3 and b2IRhMean > 20:
                                # move cw med slow
                                mL.on(3, brake=False)
                                print("move cw med slow; b2IRdistMean, b2IRhMean, b2IRhStdev", b2IRdistMean, b2IRhMean, b2IRhStdev)
                            elif b2IRdistMean > 0 and b2IRhStdev < 3 and b2IRhMean > 0:
                                # move cw slow
                                mL.on(1, brake=False)
                                print("move cw slow; b2IRdistMean, b2IRhMean, b2IRhStdev", b2IRdistMean, b2IRhMean, b2IRhStdev)

                            # if we don't see a valid +25 or -25 heading, turn med fast until we do
                            # note that an ir distance of -1 means we don't even see the beacon
                            # note also that an ir distance > 300 is not trustworthy, so let's limit that
                            if (b2IRdistMean > 300 or b2IRdistMean == -1) and b2IRhMean == 0:
                                # move fast in cw direction until we pick up a beacon signal (or exceed search limit)
                                mL.on(7, brake=False)
                                print("move fast in cw direction; b2IRdistMean, b2IRhMean", b2IRdistMean, b2IRhMean)
                            
                            # stop when heading is between -0.7 and 0.7 AND ir distance is not -1
                            # you should now have a good lock on beacon2, so grab the data
                            if b2IRdistMean > 0 and b2IRdistMean < 300 and b2IRdistStdev < 3 and b2IRhMean >= -0.7 and b2IRhMean <= 0.7 and b2IRhStdev < 3:
                                mL.on(0, brake=True)
                                time.sleep(1)   ## let the shaking stop

                                # init the data collection arrays for each collection series
                                cmpDeg = np.array([], dtype=np.int32)
                                ir2r = np.array([], dtype=np.int32)

                                # take 25 samples
                                for j in range(25):
                                    usDistCmVal = us.distance_centimeters
                                    ir2DistVal = ir.distance(channel=2)
                                    if ir2DistVal == None:
                                        ir2DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                                    else:
                                        ir2DistVal = int((ir2DistCaliK * ir2DistVal) + ir2DistCaliOffset)
                                    compassVal = cmp.value(0)
                                    cmpDeg = np.append(cmpDeg, compassVal)
                                    ir2r = np.append(ir2r, ir2DistVal)
                                    time.sleep(0.01)
                                    USr = np.append(USr, usDistCmVal)
                                
                                if beaconCali_mode == 2:
                                    # grab these data points for Cali arrays
                                    USmean = np.append(USmean, np.mean(USr))
                                    ir2rmeanCali = np.append(ir2rmeanCali, np.mean(ir2r))
                                
                                # grab the beacon2 final output    
                                zeroir2hangle = np.mean(cmpDeg)
                                zeroir2hdist = np.mean(ir2r)
                                print("zeroir2hangle = ", zeroir2hangle)
                                print("zeroir2hdist = ", zeroir2hdist)
                                print("")
                                # set b2lock true (1)
                                b2lock = 1
                            
                            if  (time.time() - 60) > hailmarryTimeout:
                                mL.on(0, brake=True)
                                print("Searching for beacon2 too long...")
                                print("Concluding beacon search")
                                print("")
                                # set b2lock true (1)
                                b2lock = 1
                                # reset hailmarryTimeout to initial values
                                hailmarryTimeout = time.time()    # probably not needed here
                    
                    ###################
                    
                    if beaconCali_mode == 0:
                        print("")
                        print("")
                        print("zeroir1hangle = ", zeroir1hangle)
                        print("zeroir1hdist = ", zeroir1hdist)
                        print("")
                        print("zeroir2hangle = ", zeroir2hangle)
                        print("zeroir2hdist = ", zeroir2hdist)
                        print("")
                        print("")


                    if beaconCali_mode == 1 or beaconCali_mode == 2:
                        if np.mean(USr) > 20:
                            while usDistCmVal > (250 - (i * 10)) and usDistCmVal > 15:
                                usDistCmVal = us.distance_centimeters
                                # move forward slow
                                ### ??? use PID controller to lock heading on beacon while moving forward
                                mL.on(4, brake=False)
                                mR.on(4, brake=False)
                                time.sleep(0.1)
                            mL.on(0, brake=False)
                            mR.on(0, brake=False)
                    
                            # testing
                            print("")
                            print("i = ", i)
                            print("USmean: ", USmean)
                            print("ir1rmeanCali: ", ir1rmeanCali)
                            print("ir2rmeanCali: ", ir2rmeanCali)
                
                # obtain slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
                # save results to vars for each IR channel (i.e. different beacons)
                if beaconCali_mode == 1:
                    ir1DistCaliK, ir1DistCaliOffset, r_value, p_value, std_err = stats.linregress(ir1rmeanCali, USmean)
                    print("")
                    print("ir1DistCaliK = ", ir1DistCaliK)
                    print("ir1DistCaliOffset = ", ir1DistCaliOffset)
                    print("r_value: ", r_value)
                    print("p_value: ", p_value)
                    print("std_err: ", std_err)
                    print("")
                
                if beaconCali_mode == 2:
                    ir2DistCaliK, ir2DistCaliOffset, r_value, p_value, std_err = stats.linregress(ir2rmeanCali, USmean)
                    print("")
                    print("ir2DistCaliK = ", ir2DistCaliK)
                    print("ir2DistCaliOffset = ", ir2DistCaliOffset)
                    print("r_value: ", r_value)
                    print("p_value: ", p_value)
                    print("std_err: ", std_err)
                    print("")

                
            #### temp add for intermediate beacon angle/distance testing
            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                sample_mode = -1

            ######################################################
            #### Added world frame plotting/graphing code here ###
            ######################################################

            # rotate zeroir1hangle and zeroir2hangle 90 deg from compass to std graph
            # orientation and convert angles from degrees to rads
            rot_zeroir1hangle = radians((415 - zeroir1hangle) % 360)
            rot_zeroir2hangle = radians((415 - zeroir2hangle) % 360)
            print("zero ir h angle rotated 90 deg...")
            print("rot_zeroir1hangle = ", rot_zeroir1hangle)
            print("rot_zeroir2hangle = ", rot_zeroir2hangle)
            print("")


            ## calculate relative location (robot position is the origin)
            ir1x = zeroir1hdist * cos(rot_zeroir1hangle)
            ir1y = zeroir1hdist * sin(rot_zeroir1hangle)

            ir2x = zeroir2hdist * cos(rot_zeroir2hangle)
            ir2y = zeroir2hdist * sin(rot_zeroir2hangle)

            print("ir1x, ir1y (beacon1) = ", ir1x, ir1y)
            print("ir2x, ir2y (beacon2) = ", ir2x, ir2y)
            print("")
            print("")


            ## graph the relative position data
            plt.figure(1)

            plt.scatter(0,0, label='robot location', color='r', s=25, marker="o")
            plt.scatter(ir1x,ir1y, label='beacon1 location', color='k', s=25, marker="o")
            plt.scatter(ir2x,ir2y, label='beacon2 location', color='c', s=25, marker="o")
            plt.axis('equal')
            plt.xlabel('x-position')
            plt.ylabel('y-position')
            plt.title('IR beacons - robot frame relative position data V902b')
            plt.legend()
            # plt.show()


            ## convert from robot frame coords to world frame
            # beacon1 world frame coords
            wir1x = 114
            wir1y = 0

            # beacon2 world frame coords
            wir2x = 0
            wir2y = 381


            # robot world frame coords using beacon1 ref
            wBOTx1 = wir1x - ir1x
            wBOTy1 = wir1y - ir1y

            # robot world frame coords using beacon2 ref
            wBOTx2 = wir2x - ir2x
            wBOTy2 = wir2y - ir2y

            print("wir1x, wir1y (beacon1 world frame location) = ", wir1x, wir1y)
            print("wir2x, wir2y (beacon2 world frame location) = ", wir2x, wir2y)
            print("")
            print("wBOTx1, wBOTy1 (robot world frame location, beacon1 ref) = ", wBOTx1, wBOTy1)
            print("wBOTx2, wBOTy2 (robot world frame location, beacon2 ref) = ", wBOTx2, wBOTy2)
            print("")
            print("")


            ## world frame "room" configuration space (simple rectangle)

            # define the bounding box of the rectangular space - lower left (origin) = swCorner
            swCornerx = 0
            swCornery = 0

            neCornerx = 114
            neCornery = 381

            # North and South Wall boundaries
            southWallx = np.array([], dtype=np.int32)
            southWally = np.array([], dtype=np.int32)
            northWallx = np.array([], dtype=np.int32)
            northWally = np.array([], dtype=np.int32)
            for i in range(neCornerx + 1):
                southWallx = np.append(southWallx, i)
                southWally = np.append(southWally, swCornery)
                northWallx = np.append(northWallx, i)
                northWally = np.append(northWally, neCornery)

            # East and West Wall boundaries
            westWallx = np.array([], dtype=np.int32)
            westWally = np.array([], dtype=np.int32)
            eastWallx = np.array([], dtype=np.int32)
            eastWally = np.array([], dtype=np.int32)
            for i in range(neCornery + 1):
                westWallx = np.append(westWallx, swCornerx)
                westWally = np.append(westWally, i)
                eastWallx = np.append(eastWallx, neCornerx)
                eastWally = np.append(eastWally, i)
            
            # process the raw US and compass polar coord data into point cloud data
            USx = []
            USy = []
            for i in range(len(cmpMean)):
                # compass to std graph frame version
                thetaRad = radians((415 - cmpMean[i]) % 360)
                radius = USmean[i]
                # make the next few lines a result of wBOTx1/y1 and x2/y2 avg or best choice 
                newx = (radius * cos(thetaRad)) + wBOTx1
                #newx = (radius * cos(thetaRad)) + wBOTx2
                # print("newx = ", newx)
                newy =  (radius * sin(thetaRad)) + wBOTy1
                #newy =  (radius * sin(thetaRad)) + wBOTy2
                # print("newy = ", newy)
                # for testing
                # # cartesianCoords.append([i,i+1])
                USx.append([newx])
                USy.append([newy])


            ## graph the US point cloud data
            plt.figure(3)

            plt.scatter(USx,USy, label='US point cloud', color='r', s=25, marker="o")
            # plt.scatter(ir1x,ir1y, label='beacon1 location', color='k', s=25, marker="o")
            # plt.scatter(ir2x,ir2y, label='beacon2 location', color='c', s=25, marker="o")
            plt.axis('equal')
            plt.xlabel('x-position')
            plt.ylabel('y-position')
            plt.title('US & compass point cloud data V902b')
            plt.legend()
            # plt.show()


            ## graph the world frame "room" and position data
            plt.figure(2)

            plt.scatter(southWallx, southWally, label='southWall', color='y', s=10, marker=".")
            plt.scatter(northWallx, northWally, label='northWall', color='y', s=10, marker=".")
            plt.scatter(westWallx, westWally, label='westWall', color='y', s=10, marker=".")
            plt.scatter(eastWallx, eastWally, label='eastWall', color='y', s=10, marker=".")

            plt.scatter(USx, USy, label='US point cloud', color='r', s=25, marker="o")

            plt.scatter(wBOTx1, wBOTy1, label='robot location b1 ref', color='b', s=25, marker="o")
            plt.scatter(wBOTx2, wBOTy2, label='robot location b2 ref', color='m', s=25, marker="o")
            plt.scatter(wir1x,wir1y, label='beacon1 location', color='k', s=25, marker="o")
            plt.scatter(wir2x,wir2y, label='beacon2 location', color='c', s=25, marker="o")
            plt.axis('equal')
            plt.xlabel('x-position')
            plt.ylabel('y-position')
            plt.title('IR beacons - world frame position data V902b')
            plt.legend()
            plt.show()



            sample_mode = -1
            print("Sampling complete. Returning to keyboard cmd mode.")

                ######################
                ######################
                ######################

               

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
