#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 8/12/2020     Ron King    - blah...
#                           - this program will allow collection of US distance data, 360 deg rotation at 1 deg increments from a specific 
#                           place in a room. Numpy arrays and tools will be used to capture and analyze the data samples
#                           - generic autonomous driving mode features and code were removed along with other vestigial code
#                           - using IR sensor (distance mode in cm) and two channels with two beacons, which will enable
#                           triangulation of two landmark, and possibly some use of the distance data as well
#                           - also adding ability to plot the data in this same program
#                           - Note: irDistVal is now set to int(3.19 * ir.distance()) to approximate distance in cm
#                           - created V9.02b with the following changes/features:
#                           - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMdataAccurV9.02b as starting point
#                           - changed all vars and arrays tied to IR to IR1; then cloned/replicated those for IR2 to support
#                           use of two IR beacons and channels simultaneously
#
# 8/27/2020     Ron King    - updated this version to find beacon angle using "5, 0, -5 method" operating on numpy arrays
#                           ---> need to low-pass-filter the IR heading (polarCoordsir1hmean, polarCoordsir2hmean) values
#                                before using to extract the beacon angle
#                           ---> also need to tune the extraction threshholds some more...perhaps find a way to use
#                                stdev vals to weight the results: ex. 
'''
                            if abs(stdev*(target - new_measured) < abs(stdev*(target - prev_measured):
                                new_best = new_measured
'''
# 8/28/2020     Ron King    - implemented a 7 point rolling avg (n-3, n-2, n-1, n, n+1, n+2, n+3) on polarCoordsir1hmean
#                           array (polarCoordsir2hmean...you're next!)
# (test git commit)

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
ODt = np.array([], dtype=np.int32)                  # array to index each time/tick (x axis independant var)
# added IR1
polarCoordsir1r = np.array([], dtype=np.int32)       # array to hold 'r' - ir1 distance r (in approx cm)
polarCoordsir1rmean = np.array([], dtype=np.int32)   # array to hold processed data ir1r-mean
polarCoordsir1rstd = np.array([], dtype=np.int32)    # array to hold processed data ir1r-stdev
polarCoordsir1h = np.array([], dtype=np.int32)       # array to hold 'h' - ir1 heading h (in +/- deg off-center)
polarCoordsir1hmean = np.array([], dtype=np.int32)   # array to hold processed data ir1 heading mean
polarCoordsir1hstd = np.array([], dtype=np.int32)    # array to hold processed data ir1 heading stdev
# added IR2
polarCoordsir2r = np.array([], dtype=np.int32)       # array to hold 'r' - ir2 distance r (in approx cm)
polarCoordsir2rmean = np.array([], dtype=np.int32)   # array to hold processed data ir2r-mean
polarCoordsir2rstd = np.array([], dtype=np.int32)    # array to hold processed data ir2r-stdev
polarCoordsir2h = np.array([], dtype=np.int32)       # array to hold 'h' - ir2 heading h (in +/- deg off-center)
polarCoordsir2hmean = np.array([], dtype=np.int32)   # array to hold processed data ir2 heading mean
polarCoordsir2hstd = np.array([], dtype=np.int32)    # array to hold processed data ir2 heading stdev

# for better graphing
polarCoordsUSstdX10 = np.array([], dtype=np.int32)      # array to hold orig array vals * 10
polarCoordsir1rstdX10 = np.array([], dtype=np.int32)     # array to hold orig array vals * 10
polarCoordsir1hmeanX10 = np.array([], dtype=np.int32)    # array to hold orig array vals * 10
polarCoordsir1hstdX10 = np.array([], dtype=np.int32)     # array to hold orig array vals * 10
polarCoordsir2rstdX10 = np.array([], dtype=np.int32)     # array to hold orig array vals * 10
polarCoordsir2hmeanX10 = np.array([], dtype=np.int32)    # array to hold orig array vals * 10
polarCoordsir2hstdX10 = np.array([], dtype=np.int32)     # array to hold orig array vals * 10

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
            ir1DistVal = int(3.19 * ir1DistVal)
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
            ir2DistVal = int(3.19 * ir2DistVal)
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
            if polarCoordsUSmean.size < 1 or polarCoordsUSstd.size < 1 or polarCoordsir1rmean.size < 1 or polarCoordsir1rstd.size < 1 or polarCoordscmpMean.size < 1:
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
                print ("polarCoordsir1rmean.size = ", polarCoordsir1rmean.size)
                print("")
                print ("polarCoordsir1rmean = ", polarCoordsir1rmean)
                print("")
                print ("polarCoordsir1rstd.size = ", polarCoordsir1rstd.size)
                print("")
                print ("polarCoordsir1rstd = ", polarCoordsir1rstd)
                print("")
                print ("polarCoordsir1hmean.size = ", polarCoordsir1hmean.size)
                print("")
                print ("polarCoordsir1hmean = ", polarCoordsir1hmean)
                print("")
                print ("polarCoordsir1hstd.size = ", polarCoordsir1hstd.size)
                print("")
                print ("polarCoordsir1hstd = ", polarCoordsir1hstd)
                print("")
                print ("polarCoordsir2rmean.size = ", polarCoordsir2rmean.size)
                print("")
                print ("polarCoordsir2rmean = ", polarCoordsir2rmean)
                print("")
                print ("polarCoordsir2rstd.size = ", polarCoordsir2rstd.size)
                print("")
                print ("polarCoordsir2rstd = ", polarCoordsir2rstd)
                print("")
                print ("polarCoordsir2hmean.size = ", polarCoordsir2hmean.size)
                print("")
                print ("polarCoordsir2hmean = ", polarCoordsir2hmean)
                print("")
                print ("polarCoordsir2hstd.size = ", polarCoordsir2hstd.size)
                print("")
                print ("polarCoordsir2hstd = ", polarCoordsir2hstd)
                print("")
                print ("polarCoordscmpMean.size = ", polarCoordscmpMean.size)
                print("")
                print ("polarCoordscmpMean = ", polarCoordscmpMean)
                print("")

        if x == 102: # f pushed - save sample statistics to file
            print("")
            if polarCoordsUSmean.size < 1 or polarCoordsUSstd.size < 1 or polarCoordsir1rmean.size < 1 or polarCoordsir1rstd.size < 1 or polarCoordscmpMean.size < 1:
                print("no data at this time")
            else:
                print ("saving sample data to csv files")
                # save to csv files
                np.savetxt('polarCoordsUSmean-V912b-8.csv', polarCoordsUSmean, delimiter=',')
                np.savetxt('polarCoordsUSstd-V912b-8.csv', polarCoordsUSstd, delimiter=',')
                np.savetxt('polarCoordsir1rmean-V912b-8.csv', polarCoordsir1rmean, delimiter=',')
                np.savetxt('polarCoordsir1rstd-V912b-8.csv', polarCoordsir1rstd, delimiter=',')
                np.savetxt('polarCoordsir1hmean-V912b-8.csv', polarCoordsir1hmean, delimiter=',')
                np.savetxt('polarCoordsir1hstd-V912b-8.csv', polarCoordsir1hstd, delimiter=',')
                np.savetxt('polarCoordsir2rmean-V912b-8.csv', polarCoordsir2rmean, delimiter=',')
                np.savetxt('polarCoordsir2rstd-V912b-8.csv', polarCoordsir2rstd, delimiter=',')
                np.savetxt('polarCoordsir2hmean-V912b-8.csv', polarCoordsir2hmean, delimiter=',')
                np.savetxt('polarCoordsir2hstd-V912b-8.csv', polarCoordsir2hstd, delimiter=',')
                np.savetxt('polarCoordscmpMean-V912b-8.csv', polarCoordscmpMean, delimiter=',')
                print ("saves commplete")
                print("")

        if x == 108: # l pushed - loading sample data from csv files
            print("")
            print ("loading sample data from csv files")
            # load from csv files
            polarCoordsUSmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsUSmean-V912b-5.csv', delimiter=',')
            polarCoordsUSstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsUSstd-V912b-5.csv', delimiter=',')
            polarCoordsir1rmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir1rmean-V912b-5.csv', delimiter=',')
            polarCoordsir1rstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir1rstd-V912b-5.csv', delimiter=',')
            polarCoordsir1hmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir1hmean-V912b-5.csv', delimiter=',')
            polarCoordsir1hstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir1hstd-V912b-5.csv', delimiter=',')            
            polarCoordsir2rmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir2rmean-V912b-5.csv', delimiter=',')
            polarCoordsir2rstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir2rstd-V912b-5.csv', delimiter=',')
            polarCoordsir2hmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir2hmean-V912b-5.csv', delimiter=',')
            polarCoordsir2hstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsir2hstd-V912b-5.csv', delimiter=',')            
            polarCoordscmpMean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordscmpMean-V912b-5.csv', delimiter=',')
            print ("loads commplete")
            print("")
            # rescaling raw *std, polarCoordsir1hmean, and polarCoordsir2hmean values (multiply by 10) for better graphing
            print("rescaling raw *std, polarCoordsir1hmean, and polarCoordsir2hmean values (multiply by 10)...")
            print("")
            for i in range(0,len(polarCoordsUSstd)):
                polarCoordsUSstdX10 = np.append(polarCoordsUSstdX10, (10 * polarCoordsUSstd[i]))
                polarCoordsir1rstdX10 = np.append(polarCoordsir1rstdX10, (10 * polarCoordsir1rstd[i]))
                polarCoordsir1hmeanX10 = np.append(polarCoordsir1hmeanX10, (10 * polarCoordsir1hmean[i]))
                polarCoordsir1hstdX10 = np.append(polarCoordsir1hstdX10, (10 * polarCoordsir1hstd[i]))
                polarCoordsir2rstdX10 = np.append(polarCoordsir2rstdX10, (10 * polarCoordsir2rstd[i]))
                polarCoordsir2hmeanX10 = np.append(polarCoordsir2hmeanX10, (10 * polarCoordsir2hmean[i]))
                polarCoordsir2hstdX10 = np.append(polarCoordsir2hstdX10, (10 * polarCoordsir2hstd[i]))
            
            # calculating additional values
            print("creating ODt for use as x axis values for plotting/graphing...")
            print("")
            for i in range(0,len(polarCoordsUSmean)):
                ODt = np.append(ODt, i)
                #print("i = ", i)
            #print("ODt = ", ODt)
            print("")
            
            print("smoothing the ir heading mean array data...")
            print("")
            # 41 point rolling avg (n-3, n-2, n-1, n, n+1, n+2, n+3)
            # IR1
            lpfir1hmean = np.array([], dtype=np.float32) # initialize var used to store smoothed results
            for i in range(0,len(polarCoordsir1hmean)):
                taken = np.take(polarCoordsir1hmean,[i-20, i-19, i-18, i-17, i-16, i-15, i-14, i-13, i-12, i-11, i-10, i-9, i-8, i-7, i-6, i-5, i-4, i-3, i-2, i-1, i, i+1, i+2, i+3, i+4, i+5, i+6, i+7, i+8, i+9, i+10, i+11, i+12, i+13, i+14, i+15, i+16, i+17, i+18, i+19, i+20], mode='wrap')
                #print("taken = ", taken)
                k = 0
                for j in range(0,len(taken)):
                    k = k + taken[j]
                k = k / 21              ###  notice "21" not "41"; may need to rethink, retune
                # print("k = ", k)
                # print("i = ", i)
                lpfir1hmean = np.append(lpfir1hmean, k)
            print("")

            # IR2
            lpfir2hmean = np.array([], dtype=np.float32) # initialize var used to store smoothed results
            for i in range(0,len(polarCoordsir2hmean)):
                taken = np.take(polarCoordsir2hmean,[i-20, i-19, i-18, i-17, i-16, i-15, i-14, i-13, i-12, i-11, i-10, i-9, i-8, i-7, i-6, i-5, i-4, i-3, i-2, i-1, i, i+1, i+2, i+3, i+4, i+5, i+6, i+7, i+8, i+9, i+10, i+11, i+12, i+13, i+14, i+15, i+16, i+17, i+18, i+19, i+20], mode='wrap')
                #print("taken = ", taken)
                k = 0
                for j in range(0,len(taken)):
                    k = k + taken[j]
                k = k / 21              ###  notice "21" not "41"; may need to rethink, retune
                # print("k = ", k)
                # print("i = ", i)
                lpfir2hmean = np.append(lpfir2hmean, k)
            print("")

            print("finding beacon angles...")
            print("")
            plus5ir1hangle = 400        # initialize var which holds the +5 deg ir1 heading compass angle
            minus5ir1hangle = 400       # initialize var which holds the -5 deg ir1 heading compass angle
            zeroir1hangle = 400         # initialize var which holds the zero deg ir1 heading compass angle
            plus5ir2hangle = 400        # initialize var which holds the +5 deg ir2 heading compass angle
            minus5ir2hangle = 400       # initialize var which holds the -5 deg ir2 heading compass angle
            zeroir2hangle = 400         # initialize var which holds the zero deg ir2 heading compass angle

            new_plus5ir1hangle = 400        # initialize var which holds the +5 deg ir1 heading compass angle
            new_minus5ir1hangle = 400       # initialize var which holds the -5 deg ir1 heading compass angle
            new_zeroir1hangle = 400         # initialize var which holds the zero deg ir1 heading compass angle
            new_plus5ir2hangle = 400        # initialize var which holds the +5 deg ir2 heading compass angle
            new_minus5ir2hangle = 400       # initialize var which holds the -5 deg ir2 heading compass angle
            new_zeroir2hangle = 400         # initialize var which holds the zero deg ir2 heading compass angle

            for i in range(0,len(polarCoordsUSmean)):
                ## IR1
                if polarCoordsir1rmean[i] <= 255 and polarCoordsir1hstd[i] <= 3 and abs(polarCoordsir1hstd[i]) > 0.3:    # needs tuning
                    # find value closest to +5
                    if lpfir1hmean[i] > 3 and lpfir1hmean[i] < 9:    # needs tuning
                        new_plus5ir1hangle = polarCoordscmpMean[i]
                        if abs(5 - new_plus5ir1hangle) < abs(5 - plus5ir1hangle):
                            plus5ir1hangle = new_plus5ir1hangle
                            # print("lpfir1hmean[i] = ", lpfir1hmean[i])
                            # print("polarCoordsir1hstd[i] = ", polarCoordsir1hstd[i])
                            
                    # find value closest to 0
                    if lpfir1hmean[i] <= 3 and lpfir1hmean[i] >= -3:    # needs tuning
                        new_zeroir1hangle = polarCoordscmpMean[i]
                        if abs(0 - new_zeroir1hangle) < abs(0 - zeroir1hangle):
                            zeroir1hangle = new_zeroir1hangle
                            # print("lpfir1hmean[i] = ", lpfir1hmean[i])
                            # print("polarCoordsir1hstd[i] = ", polarCoordsir1hstd[i])
                            
                    # find value closest to -5
                    if lpfir1hmean[i] > -9 and lpfir1hmean[i] < -3:    # needs tuning
                        new_minus5ir1hangle = polarCoordscmpMean[i]
                        if abs(-5 - new_minus5ir1hangle) < abs(-5 - minus5ir1hangle):
                            minus5ir1hangle = new_minus5ir1hangle
                            # print("lpfir1hmean[i] = ", lpfir1hmean[i])
                            # print("polarCoordsir1hstd[i] = ", polarCoordsir1hstd[i])
                            

                ## IR2
                if polarCoordsir2rmean[i] <= 255 and polarCoordsir2hstd[i] <= 3 and abs(polarCoordsir2hstd[i]) > 0.3:    # needs tuning
                    # find value closest to +5
                    if lpfir2hmean[i] > 3 and lpfir2hmean[i] < 9:    # needs tuning
                        new_plus5ir2hangle = polarCoordscmpMean[i]
                        if abs(5 - new_plus5ir2hangle) < abs(5 - plus5ir2hangle):
                            plus5ir2hangle = new_plus5ir2hangle
                            # print("lpfir2hmean[i] = ", lpfir2hmean[i])
                            # print("polarCoordsir2hstd[i] = ", polarCoordsir2hstd[i])
                            
                    # find value closest to 0
                    if lpfir2hmean[i] <= 3 and lpfir2hmean[i] >= -3:    # needs tuning
                        new_zeroir2hangle = polarCoordscmpMean[i]
                        if abs(0 - new_zeroir2hangle) < abs(0 - zeroir2hangle):
                            zeroir2hangle = new_zeroir2hangle
                            # print("lpfir2hmean[i] = ", lpfir2hmean[i])
                            # print("polarCoordsir2hstd[i] = ", polarCoordsir2hstd[i])
                            
                    # find value closest to -5
                    if lpfir2hmean[i] > -9 and lpfir2hmean[i] < -3:    # needs tuning
                        new_minus5ir2hangle = polarCoordscmpMean[i]
                        if abs(-5 - new_minus5ir2hangle) < abs(-5 - minus5ir2hangle):
                            minus5ir2hangle = new_minus5ir2hangle
                            # print("lpfir2hmean[i] = ", lpfir2hmean[i])
                            # print("polarCoordsir2hstd[i] = ", polarCoordsir2hstd[i])
                            
            print("plus5ir1hangle = ", plus5ir1hangle)
            print("zeroir1hangle = ", zeroir1hangle)
            print("minus5ir1hangle = ", minus5ir1hangle)
            print("")
            print("plus5ir2hangle = ", plus5ir2hangle)
            print("zeroir2hangle = ", zeroir2hangle)
            print("minus5ir2hangle = ", minus5ir2hangle)
            print("")
            print("")

            print("values have been created")
            print("")


        if x == 114: # r pushed - reinitializing arrays
            polarCoordsUSr = np.array([], dtype=np.int32)
            polarCoordscmpDeg = np.array([], dtype=np.int32)
            polarCoordsUSmean = np.array([], dtype=np.int32)
            polarCoordsUSstd = np.array([], dtype=np.int32)
            polarCoordscmpMean = np.array([], dtype=np.int32)
            polarCoordsir1r = np.array([], dtype=np.int32)
            polarCoordsir1rmean = np.array([], dtype=np.int32)
            polarCoordsir1rstd = np.array([], dtype=np.int32)
            polarCoordsir1h = np.array([], dtype=np.int32)
            polarCoordsir1hmean = np.array([], dtype=np.int32)
            polarCoordsir1hstd = np.array([], dtype=np.int32)
            polarCoordsir2r = np.array([], dtype=np.int32)
            polarCoordsir2rmean = np.array([], dtype=np.int32)
            polarCoordsir2rstd = np.array([], dtype=np.int32)
            polarCoordsir2h = np.array([], dtype=np.int32)
            polarCoordsir2hmean = np.array([], dtype=np.int32)
            polarCoordsir2hstd = np.array([], dtype=np.int32)
            print("r pushed - reinitialized polarCoords* arrays")

        if x == 103: # g pushed - graph the (mostly) raw data
            plt.plot(ODt,polarCoordsUSmean, label='polarCoordsUSmean')
            # plt.plot(ODt,polarCoordsUSstdX10, label='polarCoordsUSstdX10')
            plt.plot(ODt,polarCoordscmpMean, label='polarCoordscmpMean')
            plt.plot(ODt,polarCoordsir1rmean, label='polarCoordsir1rmean')
            # plt.plot(ODt,polarCoordsir1hmeanX10, label='polarCoordsir1hmeanX10')
            # plt.plot(ODt,polarCoordsir1rstdX10, label='polarCoordsir1rstdX10')
            # plt.plot(ODt,polarCoordsir1hstdX10, label='polarCoordsir1hstdX10')
            plt.plot(ODt,polarCoordsir1hmean, label='polarCoordsir1hmean')
            plt.plot(ODt,lpfir1hmean, label='lpfir1hmean')
            plt.plot(ODt,polarCoordsir2rmean, label='polarCoordsir2rmean')
            # plt.plot(ODt,polarCoordsir2hmeanX10, label='polarCoordsir2hmeanX10')                        
            # plt.plot(ODt,polarCoordsir2rstdX10, label='polarCoordsir2rstdX10')
            # plt.plot(ODt,polarCoordsir2hstdX10, label='polarCoordsir2hstdX10')
            plt.plot(ODt,polarCoordsir2hmean, label='polarCoordsir2hmean')
            plt.plot(ODt,lpfir2hmean, label='lpfir2hmean')

            plt.xlabel('ODt')
            plt.ylabel('sensor values')
            #plt.ylabel('processed data')
            plt.title('sensor data V902b')
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
                    ###time.sleep(1)   # pause to let the shaking stop before collecting data
                    # init the data collection arrays for each angle/collection series
                    polarCoordsUSr = np.array([], dtype=np.int32)
                    polarCoordscmpDeg = np.array([], dtype=np.int32)
                    polarCoordsir1r = np.array([], dtype=np.int32)
                    polarCoordsir1h = np.array([], dtype=np.int32)
                    polarCoordsir2r = np.array([], dtype=np.int32)
                    polarCoordsir2h = np.array([], dtype=np.int32)
                    # take 100 samples
                    for j in range(100):
                        usDistCmVal = us.distance_centimeters
                        ir1DistVal = ir.distance(channel=1)
                        ir2DistVal = ir.distance(channel=2)
                        if ir1DistVal == None:
                            ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                        else:
                            ir1DistVal = int(3.19 * ir1DistVal)
                        ir1HeadVal = ir.heading(channel=1)
                        if ir2DistVal == None:
                            ir2DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                        else:
                            ir2DistVal = int(3.19 * ir2DistVal)
                        ir2HeadVal = ir.heading(channel=2)
                        compassVal = cmp.value(0)
                        polarCoordsUSr = np.append(polarCoordsUSr, usDistCmVal)
                        polarCoordscmpDeg = np.append(polarCoordscmpDeg, compassVal)
                        polarCoordsir1r = np.append(polarCoordsir1r, ir1DistVal)
                        polarCoordsir1h = np.append(polarCoordsir1h, ir1HeadVal)
                        polarCoordsir2r = np.append(polarCoordsir2r, ir2DistVal)
                        polarCoordsir2h = np.append(polarCoordsir2h, ir2HeadVal)
                        
                    print("")
                    print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsUSr), np.max(polarCoordsUSr), np.mean(polarCoordsUSr), np.std(polarCoordsUSr))
                    print("")
                    print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordscmpDeg), np.max(polarCoordscmpDeg), np.mean(polarCoordscmpDeg), np.std(polarCoordscmpDeg))
                    print("")
                    print ("polarCoordsir1r.size = ", polarCoordsir1r.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsir1r), np.max(polarCoordsir1r), np.mean(polarCoordsir1r), np.std(polarCoordsir1r))
                    print("")
                    print ("polarCoordsir1h.size = ", polarCoordsir1h.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsir1h), np.max(polarCoordsir1h), np.mean(polarCoordsir1h), np.std(polarCoordsir1h))
                    print("")
                    print ("polarCoordsir2r.size = ", polarCoordsir2r.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsir2r), np.max(polarCoordsir2r), np.mean(polarCoordsir2r), np.std(polarCoordsir2r))
                    print("")
                    print ("polarCoordsir2h.size = ", polarCoordsir2h.size)
                    print("min      max      mean      std")
                    print(np.min(polarCoordsir2h), np.max(polarCoordsir2h), np.mean(polarCoordsir2h), np.std(polarCoordsir2h))
                    print("")
                    
                    polarCoordsUSmean = np.append(polarCoordsUSmean, np.mean(polarCoordsUSr))
                    polarCoordsUSstd = np.append(polarCoordsUSstd, np.std(polarCoordsUSr))
                    polarCoordscmpMean = np.append(polarCoordscmpMean, np.mean(polarCoordscmpDeg))
                    polarCoordsir1rmean = np.append(polarCoordsir1rmean, np.mean(polarCoordsir1r))
                    polarCoordsir1rstd = np.append(polarCoordsir1rstd, np.std(polarCoordsir1r))
                    polarCoordsir1hmean = np.append(polarCoordsir1hmean, np.mean(polarCoordsir1h))
                    polarCoordsir1hstd = np.append(polarCoordsir1hstd, np.std(polarCoordsir1h))
                    polarCoordsir2rmean = np.append(polarCoordsir2rmean, np.mean(polarCoordsir2r))
                    polarCoordsir2rstd = np.append(polarCoordsir2rstd, np.std(polarCoordsir2r))
                    polarCoordsir2hmean = np.append(polarCoordsir2hmean, np.mean(polarCoordsir2h))
                    polarCoordsir2hstd = np.append(polarCoordsir2hstd, np.std(polarCoordsir2h))

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
