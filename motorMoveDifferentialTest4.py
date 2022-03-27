#!/usr/bin/env python3

# Date          Author      Change Notes
# 8/22/2021     Ron King    - created MyTire class and adjusted wheel_distance_mm
#                           - various other additions made while testing
#
# 3/6/2022      Ron King    - adapted to test use of gyro and compass to improve rotational accuracy
# 3/27/2022     Ron King    - add wasd to do manual moves; testing accuracy by comparing compass, gyro + odom

"""
Used to experiment with the MoveDifferential class
"""

import time, tty, sys, threading
import matplotlib.pyplot as plt
import numpy as np
import math
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank, MoveDifferential, SpeedRPM
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor.lego import Sensor  # for ht-compass

# from ev3dev2.wheel import EV3Tire
# replaced the line above with the 6 lines below to specify my own tire dimensions
from ev3dev2.wheel import Wheel

class MyTire(Wheel):
    def __init__(self):
        Wheel.__init__(self, 62.4, 20)

from math import pi
import logging


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

insideLoop_mode = -1

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


def readHTcompass():
    global compassVal, prev_compassVal
    compassVal = cmp.value(0)
    if compassVal != prev_compassVal:
        # print("compassVal = ", compassVal)
        prev_compassVal = compassVal

def readEV3gyro():
    global gyroVal, prev_gyroVal
    gyroVal = gyro.angle
    # work-around for bogus intermittent readings of '0' (possibly a f/w bug in BrickPi3)
    if gyroVal == 0 and abs(gyroVal - prev_gyroVal) > 1:
        gyroVal = prev_gyroVal
    if gyroVal != prev_gyroVal:
        # print("gyroVal = ", gyroVal)
        prev_gyroVal = gyroVal


def keyboardInput(name):
    while (True):
        global kx  # Declare kx as global to force use of global 'x' in this function/thread
        kx = ord(sys.stdin.read(1))
        time.sleep(0.05)



if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")
    print("")


# logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)5s: %(message)s")
log = logging.getLogger(__name__)

STUD_MM = 8
INCH_MM = 25.4

ONE_FOOT_CICLE_RADIUS_MM = (12 * INCH_MM) / 2
ONE_FOOT_CICLE_CIRCUMFERENCE_MM = 2 * pi * ONE_FOOT_CICLE_RADIUS_MM

# Testing with RileyRover
# http://www.damienkee.com/rileyrover-ev3-classroom-robot-design/
#
# The centers of the wheels are 16 studs apart but this is not the
# "effective" wheel seperation.  Test drives of circles with
# a diameter of 1-foot shows that the effective wheel seperation is
# closer to 16.3 studs. ndward has a writeup that goes into effective
# wheel seperation.
# https://sites.google.com/site/ev3basic/ev3-basic-programming/going-further/writerbot-v1/drawing-arcs
mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, MyTire, 16 * STUD_MM)

mdiff.gyro = GyroSensor()


try:
    # This goes crazy on brickpi3, does it do the same on ev3?
    # mdiff.on_for_distance(SpeedRPM(-25), 720, brake=False)
    # mdiff.on_for_distance(SpeedRPM(25), 720, brake=False)
    # mdiff.on_for_distance(SpeedRPM(25), 40, brake=False)

    # Test arc left/right turns
    # mdiff.on_arc_right(SpeedRPM(80), ONE_FOOT_CICLE_RADIUS_MM, ONE_FOOT_CICLE_CIRCUMFERENCE_MM / 4)
    # mdiff.on_arc_left(SpeedRPM(80), ONE_FOOT_CICLE_RADIUS_MM, ONE_FOOT_CICLE_CIRCUMFERENCE_MM)

    # Test turning in place
    # mdiff.turn_right(SpeedRPM(25), 180)
    # mdiff.turn_right(SpeedRPM(25), 3)
    # mdiff.turn_left(SpeedRPM(25), 180)
    # mdiff.turn_left(SpeedRPM(25), 2)

    # Test odometry
    mdiff.odometry_start()
    mdiff.odometry_coordinates_log()

    from ev3dev2.unit import DistanceFeet


    ### Main loop ###
    while (True):
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

        if kx == 105: # i pushed - toggle insideLoop_mode on and off
            insideLoop_mode *= -1
            print("toggled insideLoop_mode (i); now set to...  ", insideLoop_mode)

        # read the sensors
        readHTcompass()
        readEV3gyro()

        # print all sensor values (perhaps define a var 'senseChange' and set to 0 or 1)
        print("compassVal = ", compassVal, "  gyroVal = ", gyroVal, "  odom x,y,theta = ", mdiff.x_pos_mm, mdiff.y_pos_mm, math.degrees(mdiff.theta)) 

        # mdiff.turn_to_angle(SpeedRPM(25), 0)
        # mdiff.on_for_distance(SpeedRPM(25), DistanceFeet(2).mm)
        # mdiff.turn_right(SpeedRPM(25), 180)
        # mdiff.turn_left(SpeedRPM(30), 90)
        # mdiff.on_arc_left(SpeedRPM(80), ONE_FOOT_CICLE_RADIUS_MM, ONE_FOOT_CICLE_CIRCUMFERENCE_MM)

        # Drive in a quarter arc to the right then go back to where you started
        # log.info("turn on arc to the right")
        # mdiff.on_arc_right(SpeedRPM(25), ONE_FOOT_CICLE_RADIUS_MM, ONE_FOOT_CICLE_CIRCUMFERENCE_MM / 4)
        # mdiff.odometry_coordinates_log()
        # log.info("\n\n\n\n")
        # log.info("go back to (0, 0)")
        # mdiff.odometry_coordinates_log()
        # mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)
        # mdiff.turn_to_angle(SpeedRPM(25), 90)

        # Drive in a rectangle 10 times
        # for i in range(11):
        #     # mdiff.turn_to_angle(SpeedRPM(25), 120)
        #     mdiff.on_to_coordinates(SpeedRPM(25), 0, DistanceFeet(1).mm)
        #     mdiff.on_to_coordinates(SpeedRPM(25), DistanceFeet(2).mm, DistanceFeet(1).mm)
        #     mdiff.on_to_coordinates(SpeedRPM(25), DistanceFeet(2).mm, 0)
        #     mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)
        #     mdiff.turn_to_angle(SpeedRPM(25), 90)


        # enter this block to move forward after manual alignment via 'wasd'
        if insideLoop_mode > 0:
            kx = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
            
            # Make sure motors are initially stopped
            mL.on(0, brake=True)
            mR.on(0, brake=True)
            
            # move based on mdiff cmd
            mdiff.on_for_distance(SpeedRPM(25), 2000, block = False)
        
            while insideLoop_mode > 0:
                if kx == 105: # i pushed - toggle insideLoop_mode on and off
                    insideLoop_mode *= -1

                # read the sensors
                readHTcompass()
                readEV3gyro()

                # print all sensor values (perhaps define a var 'senseChange' and set to 0 or 1)
                print("compassVal = ", compassVal, "  gyroVal = ", gyroVal, "  odom x,y,theta = ", mdiff.x_pos_mm, mdiff.y_pos_mm, math.degrees(mdiff.theta)) 
                print("")
                time.sleep(0.5)

            # Make sure motors are stopped
            mL.on(0, brake=True)
            mR.on(0, brake=True)
            #insideLoop_mode = -1
            print("insideLoop_mode actions complete. Returning to keyboard cmd mode.")

        # # Drive between two points 10 times
        # # no gyro
        # for i in range(11):
        #     mdiff.turn_to_angle(SpeedRPM(5), 90)
        #     mdiff.on_to_coordinates(SpeedRPM(25), 0, 2500)
        #     mdiff.turn_to_angle(SpeedRPM(5), 270)
        #     mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)

        # Drive between two points 10 times
        # with gyro and gyro reset each leg - FAIL
        # for i in range(11):
        #     time.sleep(1)
        #     gyro.mode = 'GYRO-FAS'
        #     time.sleep(3)
        #     mdiff.turn_to_angle(SpeedRPM(5), 90, use_gyro=True)
        #     mdiff.on_to_coordinates(SpeedRPM(25), 0, 2500)
        #     time.sleep(1)
        #     gyro.mode = 'GYRO-FAS'
        #     time.sleep(3)
        #     mdiff.turn_to_angle(SpeedRPM(5), 270, use_gyro=True)
        #     mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)

        # Use odometry to drive to specific coordinates
        # mdiff.on_to_coordinates(SpeedRPM(25), 600, 300)
        # mdiff.on_to_coordinates(SpeedRPM(15), 0, -3000)
        # mdiff.turn_to_angle(SpeedRPM(5), 90)
        # mdiff.turn_to_angle(SpeedRPM(5), 0)

        # Now go back to where we started and rotate in place to 90 degrees
        # mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)
        # mdiff.turn_to_angle(SpeedRPM(25), 90)


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
            mdiff.off()
            break
        
        print("")
        print("NAV:  kx, spd, turnRatio  ", kx, spd, turnRatio)
        print("")

        kx = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key

        time.sleep(0.2)


except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    mdiff.off()
    print("")
    print("!!!!!!!!!!!!!!!!!!!!!!")
    print("")
    print("Stopping motors A and B")
    print("")
    print("!!!!!!!!!!!!!!!!!!!!!!")
    print("")



mdiff.odometry_coordinates_log()
mdiff.odometry_stop()
