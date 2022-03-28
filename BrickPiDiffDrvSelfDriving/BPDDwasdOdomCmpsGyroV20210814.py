#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 8/14/2021     Ron King    - used file BPDDwasdSLAMdataAccurV6.01b as starting point
#                           - this program will allow collection of motor encoder ticks, gyro, and compass
#                             angle to help refine dead reckoning routines.
#                           - Numpy arrays and tools will be used to capture and analyze the data samples
#                           - starting with circular routes - simple cw pivot turns with only the left motor
#                             running to capture and compare measurements from compass, gyro, motor-tics
#
# 8/29/2021     Ron King    - created functions for reading the sensors
#                           - added data collection and plotting routines
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
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import InfraredSensor, UltrasonicSensor, GyroSensor
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

irProxVal = 100
prev_irProxVal = 0



p4 = LegoPort(INPUT_4)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p4.mode = 'ev3-uart'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device 

dmesg -w

"""

# set device name and i2c address (in hex)
p4.set_device = 'lego-ev3-gyro'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 4
gyro = GyroSensor(INPUT_4)

# allow for some time to load the new drivers
time.sleep(0.5)

print("")
print("")
print("Initializing Gyro - Don't move the robot for a few seconds ")
print("")

#gyro.mode = 'GYRO-ANG'

gyro.mode = 'GYRO-FAS'

gyroVal = 0
prev_gyroVal = -1

time.sleep(2)     # give some time to stabilize gyro before reading values




# keyboard control setup and vars
tty.setcbreak(sys.stdin)
kx = 0   # set here to make global

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
# prev_mRposition = 0     # var to help detect when to save a new OD pair (motor pos, cmpVal) to array or print to screen
badValCount = 0
htCmpCal_mode = -1      # start in htCmpCal_mode off, '-1'; '1' means htCmpCal_mode is enabled


### SLAM vars ###
cmpDegVal = np.array([], dtype=np.int32)            # arrary to hold compass-angle  in degrees

# added Gyro
gyroDegVal = np.array([], dtype=np.int32)           # arrary to hold gyro-angle  in degrees

# added array motor encoder ticks for odometry
ODmLencoderVal = np.array([], dtype=np.int32)       # arrary to hold left motor encoder-count while moving
#ODmRencoderVal = np.array([], dtype=np.int32)       # arrary to hold right motor encoder-count while moving

# added array to hold calculated robot rotation deg based on motor ticks
ODmLrotDeg = np.array([], dtype=np.int32)           # arrary to hold left motor calculated dist (cm)

# added another array for compass odometry
ODcmpDeg = np.array([], dtype=np.int32)             # array to hold associated compass angle (degrees) for each motor tick


# for later expansion
# added array to hold calculated motor distance values 
#ODmLdistCm = np.array([], dtype=np.int32)           # arrary to hold left motor calculated dist (cm)
#ODmRdistCm = np.array([], dtype=np.int32)           # arrary to hold right motor calculated dist (cm)
#polarCoordsUSr = np.array([], dtype=np.int32)       # arrary to hold 'r' - US distance r (in cm)
#polarCoordsIRr = np.array([], dtype=np.int32)       # arrary to hold 'r' - IR distance r (in arb units)

# setup motors
mL = LargeMotor(OUTPUT_A)
time.sleep(0.5)

mL.reset()
time.sleep(2)
mLspd = 0
mL.stop_action = 'coast'
# mL.polarity = 'inversed'
mL.position = 0

ODmLrotDegVal = 0
 

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
        global kx  # Declare kx as global to force use of global 'x' in this function/thread
        kx = ord(sys.stdin.read(1))
        time.sleep(0.05)

def readEV3usDistCm():
    global usDistCmVal, prev_usDistCmVal
    usDistCmVal = us.distance_centimeters
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if usDistCmVal == 0 and abs(usDistCmVal - prev_usDistCmVal) > 1:
        usDistCmVal = prev_usDistCmVal
    if usDistCmVal != prev_usDistCmVal:
        # print("usDistCmVal = ", usDistCmVal)
        prev_usDistCmVal = usDistCmVal

def readHTcompass():
    global compassVal, prev_compassVal
    compassVal = cmp.value(0)
    if compassVal != prev_compassVal:
        # print("compassVal = ", compassVal)
        prev_compassVal = compassVal

def readEV3irProx():
    global irProxVal, prev_irProxVal
    irProxVal = ir.proximity
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if irProxVal == 0 and abs(irProxVal - prev_irProxVal) > 1:
        irProxVal = prev_irProxVal
    if irProxVal != prev_irProxVal:
        # print("irProxVal = ", irProxVal)
        prev_irProxVal = irProxVal

def readEV3gyro():
    global gyroVal, prev_gyroVal
    gyroVal = gyro.angle
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if gyroVal == 0 and abs(gyroVal - prev_gyroVal) > 1:
        gyroVal = prev_gyroVal
    if gyroVal != prev_gyroVal:
        # print("gyroVal = ", gyroVal)
        prev_gyroVal = gyroVal


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")
    print("")


### Main Loop
    while (True):
        # read the sensors
        readEV3usDistCm()
        readHTcompass()
        readEV3irProx()
        readEV3gyro()

        # print all sensor values (perhaps define a var 'senseChange' and set to 0 or 1)
        print("usDistCmVal = ", int(usDistCmVal), "  compassVal = ", compassVal, "  irProxVal = ", irProxVal, "  gyroVal = ", gyroVal) 

        if usDistCmVal < 5:    ### changed from '10' to '5' for US data accuracy testing
            if obstacleNear == False:
                print("Obstacle Detected! Forward motion stopped.")
                if spd > 0:
                    spd = 0


        if kx == 32 or kx == 120: # space or x key pushed
            spd = 0
            turnRatio = 0
        if kx == 119: # w key pushed
            if spd < 90:  # limit max frwd speed to 90
                spd = spd + 5
        if kx == 115: # s key pushed
            if spd > -30:  # limit max rvrs speed to -30
                spd = spd - 5
        if kx == 100: # d key pushed (turn more to the Right)
            if turnRatio > -1:
                turnRatio = turnRatio - 0.1
        if kx == 97: # a key pushed (turn more to the Left)
            if turnRatio < 1:
                turnRatio = turnRatio + 0.1
        
        if kx == 118: # v pushed - toggle verbosity on and off
            printVerbose *= -1
            print("toggled printVerbose mode (v)")


        if kx == 105: # i pushed - toggle sample mode on and off
            sample_mode *= -1
            print("toggled sample_mode (i); now set to...  ", sample_mode)
        

        if kx == 99: # c pushed - enter HT compass calibration routine
            htCmpCal_mode *= -1
            print("toggled HT compass calibration mode (c); now set to...  ", htCmpCal_mode)
        

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
            try:
                
                # Make sure motors are initially stopped
                mL.on(0, brake=True)
                mR.on(0, brake=True)
                
                # Make sure the robot is oriented (x,y and theta) as desired before continuing
                print("")
                print("Make sure the robot is oriented (x,y and theta) as desired before continuing")
                print("")
                time.sleep(0.3)

                if compassVal >= 2:
                    print("Compass value should be 0 for start of this test, but was found to be...")
                    print("compassVal = ", compassVal)
                    print("")
                    print("Exiting sample mode")
                    print("")
                    sample_mode = -1
                    print("toggled sample_mode (i); now set to...  ", sample_mode)
                    print("")
                    time.sleep(3)
                    break

                
                # init the data collection arrays for each sample collection
                cmpDegVal = np.array([], dtype=np.int32)
                gyroDegVal = np.array([], dtype=np.int32)
                ODmLencoderVal = np.array([], dtype=np.int32)
                ODmLrotDeg = np.array([], dtype=np.int32)
                ODcmpDeg = np.array([], dtype=np.int32)

                # set motor encoders to 0
                mL.position = 0
                prev_mLposition = 0
                mR.position = 0
                prev_mRposition = 0

                ### move motors, then take samples while stopped
                # for i in range(3,359,3):
                #     # rotate robot 3 deg cw
                #     print("rotating robot 3 deg cw, then pausing for 1 sec") 
                #     mL.on_for_degrees(speed=3, degrees= 3 * 4.5)

                # for quick testing
                for i in range(10,359,10):
                    # rotate robot 10 deg cw
                    print("rotating robot 10 deg cw, then pausing for 1 sec; i = ", i) 
                    mL.on_for_degrees(speed=3, degrees= 10 * 4.5)
                    
                    # pause to let robot top shaking
                    time.sleep(1)
                    
                    # read the sensors
                    readEV3usDistCm()
                    readHTcompass()
                    readEV3irProx()
                    readEV3gyro()
                    ODmLrotDegVal = mL.position / 4.5
                    print("usDistCmVal = ", int(usDistCmVal), "  compassVal = ", compassVal, "  irProxVal = ", irProxVal, "  gyroVal = ", gyroVal, "mL.position = ", mL.position) 

                    # append the arrays
                    cmpDegVal = np.append(cmpDegVal, compassVal)
                    gyroDegVal = np.append(gyroDegVal, gyroVal)
                    ODmLencoderVal = np.append(ODmLencoderVal, mL.position)
                    ODmLrotDeg = np.append(ODmLrotDeg, ODmLrotDegVal)
                    ODcmpDeg = np.append(ODcmpDeg, compassVal)
                    
                    if cmpDegVal.size > 1000:
                        sample_mode = -1
                        print("")
                        print("")
                        print ("Exiting sample mode due to sample size too large...")
                        print ("gyroDegVal.size = ", gyroDegVal.size)
                        print("")
                    
                    time.sleep(0.3)             
                
                
            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                sample_mode = -1


            # Make sure motors are stopped
            mL.on(0, brake=True)
            mR.on(0, brake=True)
            
            ### Graph the data
            print("")
            print("creating graphs...")
            print("")
            
            # plt.plot(cmpDegVal,ODmLencoderVal, label='ODmLencoderVal')
            # plt.plot(cmpDegVal,gyroDegVal, label='gyroDegVal')
            # plt.plot(cmpDegVal,ODmLrotDeg, label='ODmLrotDeg')
            # plt.xlabel('HTcompass-EV2Gyro-MotorTicks-Deg')
            # plt.ylabel('sensor data')
            # plt.title('Odometry data')
            # plt.legend()
            # plt.show()

            # plt.scatter(cmpDegVal,ODmLencoderVal, label='ODmLencoderVal', color='r', s=10, marker="o")
            # plt.scatter(cmpDegVal,gyroDegVal, label='gyroDegVal', color='b', s=10, marker="o")
            # plt.scatter(cmpDegVal,ODmLrotDeg, label='ODmLrotDeg', color='k', s=10, marker="o")
            plt.scatter(ODmLrotDeg,ODmLencoderVal, label='ODmLencoderVal', color='r', s=10, marker="o")
            plt.scatter(ODmLrotDeg,gyroDegVal, label='gyroDegVal', color='b', s=10, marker="o")
            plt.scatter(ODmLrotDeg,cmpDegVal, label='cmpDegVal', color='k', s=10, marker="o")
            plt.axis('equal')
            plt.xlabel('ODmLrotDeg vs ODmLencoderVal-EV2Gyro-HTcompass')
            plt.ylabel('sensor data')
            plt.title('Odometry data')
            plt.legend()
            plt.show()

            sample_mode = -1
            print("Sampling complete. Returning to keyboard cmd mode.")


        if htCmpCal_mode > 0:
            try:
                # Make sure motors are initially stopped
                mL.on(0, brake=True)
                mR.on(0, brake=True)
                
                # start compass cal
                cmp.command = 'BEGIN-CAL'
                time.sleep(0.5)

                # start 360 cw robot rotation
                mL.on_for_degrees(speed=3, degrees= 1700)

            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                cmp.command = 'END-CAL'
                htCmpCal_mode = -1

            # Make sure motors are both stopped
            mL.on(0, brake=True)
            mR.on(0, brake=True)

            # stop compass cal
            cmp.command = 'END-CAL'
            time.sleep(0.5)

            # return to main loop and keyboard cmd mode
            htCmpCal_mode = -1
            print("HT compass calibration routine complete. Returning to keyboard cmd mode.")


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

        if kx == 120: # x key means exit
            break
        
        if printVerbose > 0:
            print("NAV:  kx, spd, turnRatio  ", kx, spd, turnRatio)
            navPrint = 0
        if navPrint == 10:
            print("NAV:  kx, spd, turnRatio  ", kx, spd, turnRatio)
            navPrint = 0

        navPrint += 1

        kx = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key

        time.sleep(0.2)
