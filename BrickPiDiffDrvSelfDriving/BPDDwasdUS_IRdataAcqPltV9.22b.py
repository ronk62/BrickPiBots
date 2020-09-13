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
#                           - Note: irDistVal is currently set to int(3.19 * ir.distance()) to approximate distance in cm
#                           - updated this version to find beacon angle using live, rotation, seek routine

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
ODt = np.array([], dtype=np.int32)                  # array to index each time/tick (x axis independant var)
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

        if x == 112: # p pushed - print sample statistics to screen
            if USmean.size < 1 or USstd.size < 1 or ir1rmean.size < 1 or ir1rstd.size < 1 or cmpMean.size < 1:
                print("no data at this time")
            else:
                print ("USmean.size = ", USmean.size)
                print("")
                print ("USmean = ", USmean)
                print("")
                print ("USstd.size = ", USstd.size)
                print("")
                print ("USstd = ", USstd)
                print("")
                print ("ir1rmean.size = ", ir1rmean.size)
                print("")
                print ("ir1rmean = ", ir1rmean)
                print("")
                print ("ir1rstd.size = ", ir1rstd.size)
                print("")
                print ("ir1rstd = ", ir1rstd)
                print("")
                print ("ir1hmean.size = ", ir1hmean.size)
                print("")
                print ("ir1hmean = ", ir1hmean)
                print("")
                print ("ir1hstd.size = ", ir1hstd.size)
                print("")
                print ("ir1hstd = ", ir1hstd)
                print("")
                print ("ir2rmean.size = ", ir2rmean.size)
                print("")
                print ("ir2rmean = ", ir2rmean)
                print("")
                print ("ir2rstd.size = ", ir2rstd.size)
                print("")
                print ("ir2rstd = ", ir2rstd)
                print("")
                print ("ir2hmean.size = ", ir2hmean.size)
                print("")
                print ("ir2hmean = ", ir2hmean)
                print("")
                print ("ir2hstd.size = ", ir2hstd.size)
                print("")
                print ("ir2hstd = ", ir2hstd)
                print("")
                print ("cmpMean.size = ", cmpMean.size)
                print("")
                print ("cmpMean = ", cmpMean)
                print("")

        if x == 102: # f pushed - save sample statistics to file
            print("")
            if USmean.size < 1 or USstd.size < 1 or ir1rmean.size < 1 or ir1rstd.size < 1 or cmpMean.size < 1:
                print("no data at this time")
            else:
                print ("saving sample data to csv files")
                # save to csv files
                np.savetxt('USmean-V922b-1.csv', USmean, delimiter=',')
                np.savetxt('USstd-V922b-1.csv', USstd, delimiter=',')
                np.savetxt('ir1rmean-V922b-1.csv', ir1rmean, delimiter=',')
                np.savetxt('ir1rstd-V922b-1.csv', ir1rstd, delimiter=',')
                np.savetxt('ir1hmean-V922b-1.csv', ir1hmean, delimiter=',')
                np.savetxt('ir1hstd-V922b-1.csv', ir1hstd, delimiter=',')
                np.savetxt('ir2rmean-V922b-1.csv', ir2rmean, delimiter=',')
                np.savetxt('ir2rstd-V922b-1.csv', ir2rstd, delimiter=',')
                np.savetxt('ir2hmean-V922b-1.csv', ir2hmean, delimiter=',')
                np.savetxt('ir2hstd-V922b-1.csv', ir2hstd, delimiter=',')
                np.savetxt('cmpMean-V922b-1.csv', cmpMean, delimiter=',')
                print ("saves commplete")
                print("")

        if x == 108: # l pushed - loading sample data from csv files
            print("")
            print ("loading sample data from csv files")
            # load from csv files
            USmean = np.loadtxt('/home/robot/ev3dev2Projects/USmean-V922b-1.csv', delimiter=',')
            USstd = np.loadtxt('/home/robot/ev3dev2Projects/USstd-V922b-1.csv', delimiter=',')
            ir1rmean = np.loadtxt('/home/robot/ev3dev2Projects/ir1rmean-V922b-1.csv', delimiter=',')
            ir1rstd = np.loadtxt('/home/robot/ev3dev2Projects/ir1rstd-V922b-1.csv', delimiter=',')
            ir1hmean = np.loadtxt('/home/robot/ev3dev2Projects/ir1hmean-V922b-1.csv', delimiter=',')
            ir1hstd = np.loadtxt('/home/robot/ev3dev2Projects/ir1hstd-V922b-1.csv', delimiter=',')            
            ir2rmean = np.loadtxt('/home/robot/ev3dev2Projects/ir2rmean-V922b-1.csv', delimiter=',')
            ir2rstd = np.loadtxt('/home/robot/ev3dev2Projects/ir2rstd-V922b-1.csv', delimiter=',')
            ir2hmean = np.loadtxt('/home/robot/ev3dev2Projects/ir2hmean-V922b-1.csv', delimiter=',')
            ir2hstd = np.loadtxt('/home/robot/ev3dev2Projects/ir2hstd-V922b-1.csv', delimiter=',')            
            cmpMean = np.loadtxt('/home/robot/ev3dev2Projects/cmpMean-V922b-1.csv', delimiter=',')
            print ("loads commplete")
            print("")
                        
            # calculating additional values
            print("creating ODt for use as x axis values for plotting/graphing...")
            print("")
            for i in range(0,len(USmean)):
                ODt = np.append(ODt, i)
                #print("i = ", i)
            #print("ODt = ", ODt)
            print("")
            print("values have been created")
            print("")


        if x == 114: # r pushed - reinitializing arrays
            USr = np.array([], dtype=np.int32)
            cmpDeg = np.array([], dtype=np.int32)
            USmean = np.array([], dtype=np.int32)
            USstd = np.array([], dtype=np.int32)
            cmpMean = np.array([], dtype=np.int32)
            ir1r = np.array([], dtype=np.int32)
            ir1rmean = np.array([], dtype=np.int32)
            ir1rstd = np.array([], dtype=np.int32)
            ir1h = np.array([], dtype=np.int32)
            ir1hmean = np.array([], dtype=np.int32)
            ir1hstd = np.array([], dtype=np.int32)
            ir2r = np.array([], dtype=np.int32)
            ir2rmean = np.array([], dtype=np.int32)
            ir2rstd = np.array([], dtype=np.int32)
            ir2h = np.array([], dtype=np.int32)
            ir2hmean = np.array([], dtype=np.int32)
            ir2hstd = np.array([], dtype=np.int32)
            print("r pushed - reinitialized * arrays")


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


        if x == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        
        
        ######################

        if sample_mode > 0:
            try:
                # setup for beacon angle/distance data capture
                b1lock = 0
                b1IRdistMean = -1
                b1IRhMean = 400
                zeroir1hangle = 400
                zeroir1hdist = 400
                b2lock = 0
                b2IRdistMean = -1
                b2IRhMean = 400
                zeroir2hangle = 400
                zeroir2hdist = 400

                # initialize compass value for each beacon search
                hailmarryCmpVal = 180

                # use this to determine if you have searched too long for one beacon
                # increment each time to cirle 360 deg
                hailmarryincr = 0                

                # getting beacon angle/distance data
                print("")
                print("getting beacon angle/distance data")
                print("")
                while not b1lock or not b2lock:
                    # init the data collection arrays for each collection series
                    USr = np.array([], dtype=np.int32)
                    cmpDeg = np.array([], dtype=np.int32)
                    ir1r = np.array([], dtype=np.int32)
                    ir1h = np.array([], dtype=np.int32)
                    ir2r = np.array([], dtype=np.int32)
                    ir2h = np.array([], dtype=np.int32)

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
                        USr = np.append(USr, usDistCmVal)
                        cmpDeg = np.append(cmpDeg, compassVal)
                        ir1r = np.append(ir1r, ir1DistVal)
                        ir1h = np.append(ir1h, ir1HeadVal)
                        ir2r = np.append(ir2r, ir2DistVal)
                        ir2h = np.append(ir2h, ir2HeadVal)
                    b1IRdistMean = np.mean(ir1r)
                    b1IRhMean = np.mean(ir1h)
                    b2IRdistMean = np.mean(ir2r)
                    b2IRhMean = np.mean(ir2h)

                    # show bxlock states
                    print("")
                    print("b1lock state = ", b1lock)
                    print("")
                    print("b2lock state = ", b2lock)
                    print("")


                    ######################

                    # if we don't have the beacon1 angle/distance yet, rotate to find those
                    if not b1lock:
                        if b1IRhMean < -15:
                            # move ccw med fast
                            mL.on(-5, brake=False)
                        elif b1IRhMean < -1:
                            # move ccw slow
                            mL.on(-1, brake=False)

                        if b1IRhMean > 15:
                            # move cw med fast
                            mL.on(5, brake=False)
                        elif b1IRhMean > 1:
                            # move cw slow
                            mL.on(1, brake=False)

                        # if we don't see a valid +25 or -25 heading, turn med fast until we do
                        # note that an ir distance of -1 means we don't even see the beacon
                        # note also that an ir distance > 300 is not trustworthy, so let's limit that
                        if (b1IRdistMean > 300 or b1IRdistMean == -1) and b1IRhMean == 0:
                            # move fast either cw or ccw based on compass heading
                            if compassVal >= 125:
                                # rotate cw
                                mL.on(10, brake=False)
                            if compassVal < 125 or compassVal > 352:
                                # rotate ccw
                                mL.on(-10, brake=False)
                        
                        # stop when heading is between -1 and 1 AND ir distance is not -1
                        # you should now have a good lock on beacon1, so grab the data
                        if b1IRdistMean > 0 and b1IRhMean >= -1 and b1IRhMean <= 1:
                            mL.on(0, brake=True)
                            time.sleep(1)   ## let the shaking stop

                            # init the data collection arrays for each collection series
                            cmpDeg = np.array([], dtype=np.int32)
                            ir1r = np.array([], dtype=np.int32)

                            # take 100 samples
                            for j in range(100):
                                ir1DistVal = ir.distance(channel=1)
                                if ir1DistVal == None:
                                    ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                                else:
                                    ir1DistVal = int(3.19 * ir1DistVal)
                                compassVal = cmp.value(0)
                                cmpDeg = np.append(cmpDeg, compassVal)
                                ir1r = np.append(ir1r, ir1DistVal)
                            # grab the beacon1 final output    
                            zeroir1hangle = np.mean(cmpDeg)
                            zeroir1hdist = np.mean(ir1r)
                            print("zeroir1hangle = ", zeroir1hangle)
                            print("zeroir1hdist = ", zeroir1hdist)
                            print("")
                            # set b1lock true (1)
                            b1lock = 1
                        
                        if compassVal >= (hailmarryCmpVal - 1) and compassVal <= (hailmarryCmpVal + 1):
                            hailmarryincr += 1
                            if hailmarryincr > 2:
                                mL.on(0, brake=True)
                                print("Rotated searching for beacon1 too many times...")
                                print("hailmarryincr = ", hailmarryincr)
                                print("Proceeding with search for beacon2")
                                print("")
                                # set b1lock true (1)
                                b1lock = 1
                                # reset hailmarryCmpVal and hailmarryincr to initial values
                                hailmarryCmpVal = 180
                                hailmarryincr = 0

                
                    ######################

                    # if we DO have the beacon1 angle/distance, but not beacon2, rotate to find those
                    if b1lock and not b2lock:
                        if b2IRhMean < -15:
                            # move ccw med fast
                            mL.on(-5, brake=False)
                        elif b2IRhMean < -1:
                            # move ccw slow
                            mL.on(-1, brake=False)

                        if b2IRhMean > 15:
                            # move cw med fast
                            mL.on(5, brake=False)
                        elif b2IRhMean > 1:
                            # move cw slow
                            mL.on(1, brake=False)

                        # if we don't see a valid +25 or -25 heading, turn med fast until we do
                        # note that an ir distance of -1 means we don't even see the beacon
                        # note also that an ir distance > 300 is not trustworthy, so let's limit that
                        if (b2IRdistMean > 300 or b2IRdistMean == -1) and b2IRhMean == 0:
                            # move fast either cw or ccw based on compass heading
                            if compassVal >= 125:
                                # rotate cw
                                mL.on(10, brake=False)
                            if compassVal < 125 or compassVal > 352:
                                # rotate ccw
                                mL.on(-10, brake=False)
                        
                        # stop when heading is between -1 and 1 AND ir distance is not -1
                        # you should now have a good lock on beacon2, so grab the data
                        if b2IRdistMean > 0 and b2IRhMean >= -1 and b2IRhMean <= 1:
                            mL.on(0, brake=True)
                            time.sleep(1)   ## let the shaking stop

                            # init the data collection arrays for each collection series
                            cmpDeg = np.array([], dtype=np.int32)
                            ir2r = np.array([], dtype=np.int32)

                            # take 100 samples
                            for j in range(100):
                                ir2DistVal = ir.distance(channel=2)
                                if ir2DistVal == None:
                                    ir2DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
                                else:
                                    ir2DistVal = int(3.19 * ir2DistVal)
                                compassVal = cmp.value(0)
                                cmpDeg = np.append(cmpDeg, compassVal)
                                ir2r = np.append(ir2r, ir2DistVal)
                            # grab the beacon1 final output    
                            zeroir2hangle = np.mean(cmpDeg)
                            zeroir2hdist = np.mean(ir2r)
                            # set b2lock true (1)
                            b2lock = 1
                        
                        if compassVal >= (hailmarryCmpVal - 1) and compassVal <= (hailmarryCmpVal + 1):
                            hailmarryincr += 1
                            if hailmarryincr > 2:
                                mL.on(0, brake=True)
                                print("Rotated searching for beacon2 too many times...")
                                print("hailmarryincr = ", hailmarryincr)
                                print("Concluding beacon search")
                                print("")
                                # set b2lock true (1)
                                b2lock = 1
                                # reset hailmarryCmpVal and hailmarryincr to initial values
                                hailmarryCmpVal = 180
                                hailmarryincr = 0


                ###################
                
                print("")
                print("")
                print("zeroir1hangle = ", zeroir1hangle)
                print("zeroir1hdist = ", zeroir1hdist)
                print("")
                print("zeroir2hangle = ", zeroir2hangle)
                print("zeroir2hdist = ", zeroir2hdist)
                print("")
                print("")

            #### temp add for intermediate beacon angle/distance testing
            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                sample_mode = -1

            sample_mode = -1
            print("Sampling complete. Returning to keyboard cmd mode.")

                ######################
                ######################
                ######################


            #     # setup for US data capture
            #     # rototate to initial cmp angle 350 +/- 2
            #     compassVal = cmp.value(0)
            #     while compassVal < 348 or compassVal > 352:
            #         if compassVal >= 125:
            #             # rotate cw
            #             mL.on(1, brake=False)
            #         if compassVal < 125 or compassVal > 352:
            #             # rotate ccw
            #             mL.on(-1, brake=False)
            #     mL.on(0, brake=True)
            #     time.sleep(0.5)

            #     # take US data samples while rotating cw
            #     for i in range(360):
            #         ## move motor to new cmps angle
            #         while compassVal < i - 1 or compassVal > i + 1:
            #             compassVal = cmp.value(0)
            #             #print("compasVal = ", compassVal)
            #             mL.on(1, brake=False)
            #         mL.on(0, brake=True)
            #         ###time.sleep(1)   # pause to let the shaking stop before collecting data
            #         # init the data collection arrays for each angle/collection series
            #         USr = np.array([], dtype=np.int32)
            #         cmpDeg = np.array([], dtype=np.int32)
                    
            #         # take 100 samples
            #         for j in range(100):
            #             usDistCmVal = us.distance_centimeters
            #             ir1DistVal = ir.distance(channel=1)
            #             ir2DistVal = ir.distance(channel=2)
            #             if ir1DistVal == None:
            #                 ir1DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
            #             else:
            #                 ir1DistVal = int(3.19 * ir1DistVal)
            #             ir1HeadVal = ir.heading(channel=1)
            #             if ir2DistVal == None:
            #                 ir2DistVal = -1  ### set to -1 instead of None or numpy.savetxt and other Ops will complain
            #             else:
            #                 ir2DistVal = int(3.19 * ir2DistVal)
            #             ir2HeadVal = ir.heading(channel=2)
            #             compassVal = cmp.value(0)
            #             USr = np.append(USr, usDistCmVal)
            #             cmpDeg = np.append(cmpDeg, compassVal)
            #             ir1r = np.append(ir1r, ir1DistVal)
            #             ir1h = np.append(ir1h, ir1HeadVal)
            #             ir2r = np.append(ir2r, ir2DistVal)
            #             ir2h = np.append(ir2h, ir2HeadVal)
                        
            #         print("")
            #         print ("USr.size = ", USr.size)
            #         print("min      max      mean      std")
            #         print(np.min(USr), np.max(USr), np.mean(USr), np.std(USr))
            #         print("")
            #         print ("cmpDeg.size = ", cmpDeg.size)
            #         print("min      max      mean      std")
            #         print(np.min(cmpDeg), np.max(cmpDeg), np.mean(cmpDeg), np.std(cmpDeg))
            #         print("")
            #         print ("ir1r.size = ", ir1r.size)
            #         print("min      max      mean      std")
            #         print(np.min(ir1r), np.max(ir1r), np.mean(ir1r), np.std(ir1r))
            #         print("")
            #         print ("ir1h.size = ", ir1h.size)
            #         print("min      max      mean      std")
            #         print(np.min(ir1h), np.max(ir1h), np.mean(ir1h), np.std(ir1h))
            #         print("")
            #         print ("ir2r.size = ", ir2r.size)
            #         print("min      max      mean      std")
            #         print(np.min(ir2r), np.max(ir2r), np.mean(ir2r), np.std(ir2r))
            #         print("")
            #         print ("ir2h.size = ", ir2h.size)
            #         print("min      max      mean      std")
            #         print(np.min(ir2h), np.max(ir2h), np.mean(ir2h), np.std(ir2h))
            #         print("")
                    
            #         USmean = np.append(USmean, np.mean(USr))
            #         USstd = np.append(USstd, np.std(USr))
            #         cmpMean = np.append(cmpMean, np.mean(cmpDeg))
            #         ir1rmean = np.append(ir1rmean, np.mean(ir1r))
            #         ir1rstd = np.append(ir1rstd, np.std(ir1r))
            #         ir1hmean = np.append(ir1hmean, np.mean(ir1h))
            #         ir1hstd = np.append(ir1hstd, np.std(ir1h))
            #         ir2rmean = np.append(ir2rmean, np.mean(ir2r))
            #         ir2rstd = np.append(ir2rstd, np.std(ir2r))
            #         ir2hmean = np.append(ir2hmean, np.mean(ir2h))
            #         ir2hstd = np.append(ir2hstd, np.std(ir2h))

            #         if USmean.size > 100000:
            #             sample_mode = -1
            #             print("")
            #             print("")
            #             print ("Exiting sample mode due to sample size too large...")
            #             print ("USmean.size = ", USmean.size)
            #             print("")
                
            # except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            #     mL.on(0, brake=False)
            #     mR.on(0, brake=False)
            #     sample_mode = -1

            # sample_mode = -1
            # print("Sampling complete. Returning to keyboard cmd mode.")
               

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
