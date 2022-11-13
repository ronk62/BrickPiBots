#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 11/12/2022    Ron King    - from file 'BPDDwasdAprTag_US_PltV20210131'
#                           - introduction of MindSensors ToF Distance sensor
#                           - IR sensor is in proximity mode (no beacons)
#                           - collecting and graphing US, IR, T0F distance info, correlated with cmps angle
#                           - retained, but stubbed-out the April tag code in sample mode

#                           - DON'T Forget to start xming and export DISPLAY=10.0.0.9:0.0  (change IP addr as req'd)


""" 
This program provides SLAM testing with wasd keyboard remote control and data collection.

Motors

motor A = Left motor
motor B = Right motor

##############################################################
ref material for "rotate tag to align with world frame coords"
https://www.meccanismocomplesso.org/en/3d-rotations-and-euler-angles-in-python/

"Robotics 1 U1 (Kinematics) S5 (Homogeneous Transformation Matrix) P1 (HTM from Rotation Matrix)"

The main difference in the analysis is that Kinematics assumes a linkage between the
frames, while localization assumes tags placed in the base frame at a location and
rotation that is not rigidly linked to the base-frame. Likewise, the cam/bot is not rigidly
linked to the tags.

The result is that...
1) the relative tag rotation angles are not used for displacement calculations
2) the rotations and x,y displacements for tags are measured and entered as known data
3) the cam/bot matrix data comes from the processed vid capture and apriltag lib

ref. https://www.youtube.com/watch?v=fXewWpehAWw&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&index=17


Notations and code outline
--------------------------
H0 represents the base frame itself (with or without a displacement offset)
H1 represents the Apriltag
H2 represents the cam/bot

H1 is constructed from intuition due to initial condition simplicity and is for ref only

H0_1 is initially composed from R0_1 and d0_1 components using np.concatinate...
...then computed as the dot.product of H0 and H0_1

R0_1 and d0_1 are composed from angles, displacement values, and formulas

H0_2 is the cam/bot in the base frame and is the dot.product of H0_1 and the camInTagFrame


"""

import time, os, tty, sys, threading
import matplotlib.pyplot as plt
import numpy as np
import cv2
import apriltag
import math
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
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

ir.mode = 'IR-PROX'
irProxVal = 100
prev_irProxVal = 0



p4 = LegoPort(INPUT_4)

# https://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-buster/brickpi3.html#input-ports
p4.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-buster/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device 

dmesg -w

"""

# set device name and i2c address (in hex)
p4.set_device = 'ms-tof 0x01'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 4
ToF = Sensor(INPUT_4)

# allow for some time to load the new drivers
time.sleep(0.5)


ToF.mode = 'DIST'   # distance in mm

distmm = 0
prev_distmm = 0




# initialize apriltag Detector
detector = apriltag.Detector()


# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
sample_mode = -1        # start in sample mode off, '-1'; '1' means sample mode is enabled
obstacleNear = False    # there is an obstacle nearby - need to stop, turn, backup, etc (and go another way)
navPrint = 10           # print the Nav msgs every nth cycle
printVerbose = 1        # toggle verbose data/msg printing to terminal (-1 is 'disabled')
i = 0                   # "outer" iterator for dist point sampling
j = 0                   # "inner" iterator for dist point sampling

# SLAM vars
USr = np.array([], dtype=np.int32)       # array to hold 'r' - US distance r (in cm)
USmean = np.array([], dtype=np.int32)    # array to hold processed data US-mean
USstd = np.array([], dtype=np.int32)     # array to hold processed data US-stdev

IRr = np.array([], dtype=np.int32)       # array to hold 'r' - IR distance r (in approx cm)
IRmean = np.array([], dtype=np.int32)    # array to hold processed data IR-mean
IRstd = np.array([], dtype=np.int32)     # array to hold processed data IR-stdev

ToFr = np.array([], dtype=np.int32)       # array to hold 'r' - ToF distance r (in cm)
ToFmean = np.array([], dtype=np.int32)    # array to hold processed data ToF-mean
ToFstd = np.array([], dtype=np.int32)     # array to hold processed data ToF-stdev

cmpDeg = np.array([], dtype=np.int32)    # array to hold 'thetaDeg' - compass-angle  in degrees
cmpMean = np.array([], dtype=np.int32)   # array to hold processed data cmps-mean
ODt = np.array([], dtype=np.int32)       # array to index each time/tick (x axis independant var)


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

#np.set_printoptions(precision=2,floatmode='fixed')
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# vars for rotation matrices and calculations
R0_1 = np.array([], dtype=np.int32)
d0_1 = np.array([], dtype=np.int32)

H0 = np.array([], dtype=np.int32)
H1 = np.array([], dtype=np.int32)
H0_1 = np.array([], dtype=np.int32)
H0_2 = np.array([], dtype=np.int32)

# H0 at origin
H0 = [[1,0,0,0],
[0,1,0,0],
[0,0,1,0],
[0,0,0,1]]

# # H0 NOT at origin
# H0 = [[1,0,0,0],
# [0,1,0,400],
# [0,0,1,0],
# [0,0,0,1]]


# # H1 displaced from origin with no rotation - not used...for ref only
# H1 = [[1,0,0,0.05],
# [0,1,0,0],
# [0,0,1,0.05],
# [0,0,0,1]]

# Assign Eurler rotation angles
theta1Deg=135  # y rotation angle between H0 and H1 (positive values are cw when viewed top down in real world)

# convert angles from deg to radians
theta1Rad=math.radians(theta1Deg)

# Assign displacement values
ax1=0.9     # x displacement between H0 and H1 in meters
az1=0       # z displacement between H0 and H1 in meters
wAprTag1x = ax1 * 100     # scaling for graphs
wAprTag1y = az1 * 100     # scaling for graphs

### Define the rotation
# R0_1
R0_1 = [[np.cos(theta1Rad),0,np.sin(theta1Rad)],
[0,1,0],
[-np.sin(theta1Rad),0,np.cos(theta1Rad)]]


# Apply the displacement translations
d0_1 = [[ax1],[0],[az1]]

# setup vars to graph the unit vectors
# V0 location and rotation
x0 = H0[0][3]
y0 = H0[2][3]
vxx0 = [x0,x0 + H0[0][0]]
vxy0 = [y0,y0 + H0[2][0]]
vyx0 = [x0,x0 + H0[0][2]]
vyy0 = [y0,y0 + H0[2][2]]

print()
print("the x,y position of H0 is  ", x0,y0)
print()


H0_1 = np.concatenate((R0_1, d0_1), 1)
H0_1 = np.concatenate((H0_1, [[0,0,0,1]]), 0)

H0_1 = np.dot(H0, H0_1)

print()
print("H0_1 is  ")
print(np.matrix(H0_1))
print()

# extract the values from H0_1 for graphing
x1 = H0_1[0][3]
y1 = H0_1[2][3]
vxx1 = [x1,x1 + H0_1[0][0]]
vxy1 = [y1,y1 + H0_1[2][0]]
vyx1 = [x1,x1 + H0_1[0][2]]
vyy1 = [y1,y1 + H0_1[2][2]]

print()
print("the x,y position of H0_1 is  ", x1,y1)
print()

result = []


def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")

# use the below for detailed printing of bogus intermittent readings of '0'
badValCount = 0

### Main Loop
while (True):
    usDistCmVal = us.distance_centimeters
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if usDistCmVal == 0 and abs(usDistCmVal - prev_usDistCmVal) > 1:
        #badValCount += 1
        #print("BAD usDistCmVal = ", usDistCmVal, "badValCount = ", badValCount, "  prev_usDistCmVal (good value) = ", prev_usDistCmVal)   # print for testing; comment for more 'normal' use
        usDistCmVal = prev_usDistCmVal

    irProxVal = ir.proximity
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if irProxVal == 0 and abs(irProxVal - prev_irProxVal) > 1:
        #badValCount += 1
        #print("BAD irProxVal = ", irProxVal, "badValCount = ", badValCount, "  prev_irProxVal (good value) = ", prev_irProxVal)  # print for testing; comment for more 'normal' use
        irProxVal = prev_irProxVal

    distmm = ToF.value(0)

    compassVal = cmp.value(0)

    if usDistCmVal != prev_usDistCmVal or irProxVal != prev_irProxVal or distmm != prev_distmm or compassVal != prev_compassVal:
        if printVerbose > 0:
            print("usDistCmVal = ", int(usDistCmVal), "  irProxVal = ", irProxVal, " distmm = ", distmm, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
        prev_usDistCmVal = usDistCmVal
        prev_irProxVal = irProxVal
        prev_distmm = distmm
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
        print("toggled printVerbose mode (v); now set to...  ", printVerbose)


    if x == 105: # i pushed - toggle sample mode on and off
        sample_mode *= -1
        print("toggled sample_mode (i); now set to...  ", sample_mode)
    
    
    ######################

    ## some notes about odometry and robot wheelbase
    # from V501b-9.csv
    #      --> average of 18.452752525 ticks per cm
    #      --> average of 0.054192457 cm per tick
    # robot wheelbase = 14 cm (this is the radius of a pivot turn on one wheel)
    # 2 pi r = 88 cm (circumference, i.e. full robot rotation) = 1624 motor ticks
    # 4.5 ticks per degree of robot rotation
    # a 45 deg robot pivot turn = 202 motor ticks
    # a 30 deg robot pivot turn = 135 motor ticks
    # a 10 deg robot pivot turn = 45 motor ticks

    if sample_mode > 0:
        #### commenting Apriltag sample section for now - 11/12/2022
        '''
        # initialize PiCam, vars, arrays for image capture
        tagInCamFrame = np.array([[],[],[],[]], dtype=np.int32)
        camInTagFrame = np.array([[],[],[],[]], dtype=np.int32)
        CamZxDeg = 180
        cap = cv2.VideoCapture(0)
        time.sleep(1)
        result = []
        n = 12   # limit how many times we rotate 30 degrees looking for a tag
        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
        print("Capturing camera frame...")
        tic = time.time()       # for timing analysis
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = detector.detect(gray)
        while result == []:
            if n > 0:  # limit # of rotation iterations; 12 times (360 degs) should do it
                n = n -1  # decrement n
                
                # rotate robot 30 deg cw as we look for a tag
                mL.on_for_degrees(speed=14, degrees=135)
                time.sleep(1)

                # flush the camera frame buffer
                for i in range(7):
                    ret, frame = cap.read()

                np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
                print("Trying again...capturing camera frame...")
                # for timing analysis
                tic = time.time()
                ret, frame = cap.read()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                result = detector.detect(gray)
                # for initial testing/devopment
                print("dector result is ", result)
                print("")
                print("len of result is ", len(result))
                print(type(result))
                print("")
                print("delta t = ", time.time() - tic)
                print("")
            else:
                print("tag not found after wait time of ", time.time() - tic)
                print("")
                time.sleep(20)
                n = 12  # reset n

        ## center the cam on tag Zx zero (origin)
        while CamZxDeg < -1 or CamZxDeg > 1:
            tic = time.time()  # for timing analysis
            # capture new image frame from cam
            # flush the camera frame buffer
            for i in range(7):
                ret, frame = cap.read()

            np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
            print("centering the cam on tag Zx zero (origin)...capturing camera frame...")
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            result = detector.detect(gray)
            # for initial testing/devopment
            print("dector result is ", result)
            print("")
            print("len of result is ", len(result))
            print(type(result))
            print("")
            print("delta t = ", time.time() - tic)
            print("")

            # extract contents of results list
            for i, enum_result in enumerate(result):
                # print("i = ", i)
                # print("enum_result is... ")
                # print(enum_result.tostring())
                # print("")
                #result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
                result_pose = detector.detection_pose(enum_result,camera_params=(604.8851295863385, 606.0410127799453, 320.0, 240.0),tag_size=0.16, z_sign=1)
                # print("")
                # print("apriltag standard pose dector result is... ")
                # print(result_pose)
                # print("")

                for j, emum_result_pose in enumerate(result_pose):
                    if j == 0:
                        tagInCamFrame = emum_result_pose

                # Invert the frame perspective via matrix inversion
                # the x,y,z R and T vectors in this view show the camera/robot location relative to the tag
                camInTagFrame = np.linalg.inv(tagInCamFrame)

                # os.system("clear")
                print("")
                print("apriltag standard (tagInCamFrame) pose dector result is... ")
                print(np.matrix(tagInCamFrame))
                print("")
                
                # print camInTagFrame
                print("")
                print("inverted (camInTagFrame) pose dector result is... ")
                print(np.matrix(camInTagFrame))
                print("")

                # calculate and print cam/bot in base frame
                H0_2 = np.dot(H0_1, camInTagFrame)

                print("")
                print("cam/bot in base frame (H0_2) pose is... ")
                print(np.matrix(H0_2))
                print("")

                # extract the values for AprTag in Bot frame graphing
                AprTagx1 = tagInCamFrame[0][3]
                AprTagy1 = tagInCamFrame[2][3]                
                
                # extract the values from H0_2 for graphing
                x2 = H0_2[0][3]
                y2 = H0_2[2][3]
                vxx2 = [x2,x2 + H0_2[0][0]]
                vxy2 = [y2,y2 + H0_2[2][0]]
                vyx2 = [x2,x2 + H0_2[0][2]]
                vyy2 = [y2,y2 + H0_2[2][2]]

                # robot world frame coords using Apriltag ref
                wBOTx1 = x2 * 100
                wBOTy1 = y2 * 100
                print()
                print("the robot world frame x,y position (H0_2) is  ", wBOTx1,wBOTy1)
                print()
                print("")
                
                # calculate and print Robot heading (Z) Euler angle from Y axis rotation
                # use translational X and Z from tagInCamFrame, tag origin (centered) on X = 0 is the goal
                # angle = arctan * slope (slope = rise/run = deltaZ/deltaX)
                print("")
                print("Robot heading (Z) Euler angle (tagInCamFrame), from Y axis rotation (for Ref only, KODY KING!)... ")
                # arctan ((z1-z0) / (x1-x0))...x0, z0 is always 0,0 --> the origin of the cam
                x1 = tagInCamFrame[0][3]
                z1 = tagInCamFrame[2][3]
                CamZxDeg = (math.degrees(math.atan(z1/x1)))
                
                # offset from 90 deg, since the cam z is pointing at tag x
                if CamZxDeg >= 0:
                    CamZxDeg = CamZxDeg - 90 # subtract 90 if CamZxDeg is positive (or zero)
                else:
                    CamZxDeg = 90 + CamZxDeg # add to 90 if CamZxDeg is negative
                print(CamZxDeg, "  degrees")
                print("")
                print("Loop time = ", time.time() - tic)
                print("")

            ## rotate Bot/cam
            mL.on_for_degrees(speed=7, degrees=CamZxDeg * -4.5)
            time.sleep(1)

        

        cap.release()
        cv2.destroyAllWindows()
        '''

        #### hard-coding BOT world-frame, in place of Apriltag ref - 11/12/2022
        wBOTx1 = 114
        wBOTy1 = 40


        ######################

        ## setup for US data capture
        ## comment the following section for quick testing
        ## uncomment for normal operation
        print("rototate to initial cmp angle 350 +/- 2")
        print("")
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

        print("rotate to angle i; stop and take US data samples")
        print("")
        
        # rotate to angle i; stop and take US data samples
        #for i in range(compassVal - 1,compassVal + 1,1):  ### use for quick IR testing
        #for i in range(6,360,60):  ### use for quick, low-res testing
        #for i in range(6,360,6):  ### use for normal operation
        #for i in range(6,360,1):  ### use for slower, high-res testing
        for i in range(3,360,1):  ### active config
            ## move motor to new cmps angle
            while compassVal < i - 1 or compassVal > i + 1:
                compassVal = cmp.value(0)
                #print("compasVal = ", compassVal)
                mL.on(3, brake=False)
            mL.on(0, brake=True)
            ###time.sleep(1)   # pause to let the shaking stop before collecting data

            # init the data collection arrays for each angle/collection series
            USr = np.array([], dtype=np.int32)
            IRr = np.array([], dtype=np.int32)
            ToFr = np.array([], dtype=np.int32)
            cmpDeg = np.array([], dtype=np.int32)
            
            # take 100 samples
            for j in range(100):
                usDistCmVal = us.distance_centimeters
                # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
                if usDistCmVal == 0 and abs(usDistCmVal - prev_usDistCmVal) > 1:
                    #badValCount += 1
                    #print("BAD usDistCmVal = ", usDistCmVal, "badValCount = ", badValCount, "  prev_usDistCmVal (good value) = ", prev_usDistCmVal)   # print for testing; comment for more 'normal' use
                    usDistCmVal = prev_usDistCmVal

                irProxVal = ir.proximity
                # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
                if irProxVal == 0 and abs(irProxVal - prev_irProxVal) > 1:
                    #badValCount += 1
                    #print("BAD irProxVal = ", irProxVal, "badValCount = ", badValCount, "  prev_irProxVal (good value) = ", prev_irProxVal)  # print for testing; comment for more 'normal' use
                    irProxVal = prev_irProxVal

                distmm = ToF.value(0)
                if distmm > 3000:
                    distmm = 0
                distcm = distmm / 10
                
                compassVal = cmp.value(0)

                USr = np.append(USr, usDistCmVal)
                IRr = np.append(IRr, irProxVal)
                ToFr = np.append(ToFr, distcm)
                cmpDeg = np.append(cmpDeg, compassVal)
                
            print("")
            print ("USr.size = ", USr.size)
            print("min      max      mean      std")
            print(np.min(USr), np.max(USr), np.mean(USr), np.std(USr))
            print("")

            print("")
            print ("IRr.size = ", IRr.size)
            print("min      max      mean      std")
            print(np.min(IRr), np.max(IRr), np.mean(IRr), np.std(IRr))
            print("")
            
            print("")
            print ("ToFr.size = ", ToFr.size)
            print("min      max      mean      std")
            print(np.min(ToFr), np.max(ToFr), np.mean(ToFr), np.std(ToFr))
            print("")
            
            print ("cmpDeg.size = ", cmpDeg.size)
            print("min      max      mean      std")
            print(np.min(cmpDeg), np.max(cmpDeg), np.mean(cmpDeg), np.std(cmpDeg))
            print("")
            print("")
            
            USmean = np.append(USmean, np.mean(USr))
            USstd = np.append(USstd, np.std(USr))

            IRmean = np.append(IRmean, np.mean(IRr))
            IRstd = np.append(IRstd, np.std(IRr))
            
            ToFmean = np.append(ToFmean, np.mean(ToFr))
            ToFstd = np.append(ToFstd, np.std(ToFr))
            
            cmpMean = np.append(cmpMean, np.mean(cmpDeg))

            if USmean.size > 100000:
                sample_mode = -1
                print("")
                print("")
                print ("Exiting sample mode due to sample size too large...")
                print ("USmean.size = ", USmean.size)
                print("")


                
        ##################################################
        #### 'World frame' plotting/graphing code here ###
        ##################################################


        #### commenting Apriltag sample section for now - 11/12/2022
        '''
        ## graph the relative position data
        plt.figure(1)

        plt.scatter(0,0, label='robot location', color='r', s=25, marker="o")
        plt.scatter(AprTagx1,AprTagy1, label='tag location', color='k', s=25, marker="o")
        plt.axis('equal')
        plt.xlabel('x-position')
        plt.ylabel('y-position')
        plt.title('Apriltag location in robot frame - relative position data V20210131')
        plt.legend()
        # plt.show()


        print("wAprTag1x, wAprTag1y (AprTag1 world frame location) = ", wAprTag1x, wAprTag1y)
        print("")
        print("wBOTx1, wBOTy1 (robot world frame location, AprTag1 ref) = ", wBOTx1, wBOTy1)
        print("")
        print("")

        '''


        ## world frame "room" configuration space (simple rectangle)

        # define the bounding box of the rectangular space - lower left (origin) = swCorner
        swCornerx = 0
        swCornery = 0

        neCornerx = 200
        neCornery = 400

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
        # use Bot world frame (wBOTx1,wBOTy1) as plot ref
        USx1 = []
        USy1 = []
        for i in range(len(cmpMean)):
            # compass to std graph frame version
            thetaRad = math.radians((418 - cmpMean[i]) % 360)
            radius = USmean[i]
            newx = (radius * math.cos(thetaRad)) + wBOTx1
            # print("newx = ", newx)
            newy =  (radius * math.sin(thetaRad)) + wBOTy1
            # print("newy = ", newy)
            # for testing
            # # cartesianCoords.append([i,i+1])
            USx1.append([newx])
            USy1.append([newy])

        # process the raw IR and compass polar coord data into point cloud data
        # use Bot world frame (wBOTx1,wBOTy1) as plot ref
        IRx1 = []
        IRy1 = []
        for i in range(len(cmpMean)):
            # compass to std graph frame version
            thetaRad = math.radians((418 - cmpMean[i]) % 360)
            radius = IRmean[i]
            newx = (radius * math.cos(thetaRad)) + wBOTx1
            # print("newx = ", newx)
            newy =  (radius * math.sin(thetaRad)) + wBOTy1
            # print("newy = ", newy)
            # for testing
            # # cartesianCoords.append([i,i+1])
            IRx1.append([newx])
            IRy1.append([newy])

        # process the raw ToF and compass polar coord data into point cloud data
        # use Bot world frame (wBOTx1,wBOTy1) as plot ref
        ToFx1 = []
        ToFy1 = []
        for i in range(len(cmpMean)):
            # compass to std graph frame version
            thetaRad = math.radians((418 - cmpMean[i]) % 360)
            radius = ToFmean[i]
            newx = (radius * math.cos(thetaRad)) + wBOTx1
            # print("newx = ", newx)
            newy =  (radius * math.sin(thetaRad)) + wBOTy1
            # print("newy = ", newy)
            # for testing
            # # cartesianCoords.append([i,i+1])
            ToFx1.append([newx])
            ToFy1.append([newy])

        ## graph the US, IR, and ToF point cloud data
        plt.figure(3)

        plt.scatter(USx1,USy1, label='US point cloud', color='g', s=25, marker="o")
        plt.scatter(IRx1,IRy1, label='IR point cloud', color='r', s=25, marker="o")
        plt.scatter(ToFx1,ToFy1, label='ToF point cloud', color='b', s=25, marker="o")
        plt.axis('equal')
        plt.xlabel('x-position')
        plt.ylabel('y-position')
        plt.title('US, IR, ToF, & compass point cloud data V20221112')
        plt.legend()
        # plt.show()


        ## graph the world frame "room" and position data
        plt.figure(2)

        plt.scatter(southWallx, southWally, label='southWall', color='y', s=10, marker=".")
        plt.scatter(northWallx, northWally, label='northWall', color='y', s=10, marker=".")
        plt.scatter(westWallx, westWally, label='westWall', color='y', s=10, marker=".")
        plt.scatter(eastWallx, eastWally, label='eastWall', color='y', s=10, marker=".")

        plt.scatter(USx1, USy1, label='US point cloud', color='r', s=25, marker="o")
        # plt.scatter(USx2, USy2, label='US point cloud', color='g', s=25, marker="o")

        plt.scatter(wBOTx1, wBOTy1, label='robot location AprTag1 ref', color='b', s=25, marker="o")
        # commented Apriltag line below - 11/12/2022
        #plt.scatter(wAprTag1x,wAprTag1y, label='AprTag1 location', color='k', s=25, marker="o")
        plt.axis('equal')
        plt.xlabel('x-position')
        plt.ylabel('y-position')
        plt.title('AprTag1 - world frame position data V20221112')
        plt.legend()
        plt.show()



        sample_mode = -1
        print("Sampling complete. Returning to keyboard cmd mode.")
        # cleanup arrays and vars
        USmean = np.array([], dtype=np.int32)
        cmpMean = np.array([], dtype=np.int32)


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

    if navPrint == 10:
        if printVerbose > 0:
            print("NAV:  x, spd, turnRatio, sample_mode  ", x, spd, turnRatio, sample_mode)
        navPrint = 0
    navPrint += 1

    if obstacleNear:
        print("obstacleNear ", obstacleNear)

    x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
    time.sleep(0.2)
