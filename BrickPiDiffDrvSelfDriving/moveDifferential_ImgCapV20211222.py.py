#!/usr/bin/env python3

# Date          Author      Change Notes
#
# 8/22/2021     Ron King    - created MyTire class and adjusted wheel_distance_mm
#                           - various other additions made while testing
#
# 11/8/2021     Ron King    - adapted for use to accurately position for image captures
#
# 12/22/2021    Ron King    - created this new file from motorMoveDifferentialTest2.py to semi-automate the
#                             collection of images related to localization with ORB detect & BFMatcher
#                           - currently, this will only collect images with a changing y-value and constant
#                             x-value. Operator must align the x position prior to launch.
#
# 12/27/2021    Ron King    - discovered problems in data that stem from not clearing the image buffer
#


"""
Uses the MoveDifferential class
"""

from ev3dev2.motor import OUTPUT_A, OUTPUT_B, MoveDifferential, SpeedRPM
# from ev3dev2.wheel import EV3Tire
# replaced the line above with the 6 lines below to specify my own tire dimensions
from ev3dev2.wheel import Wheel

class MyTire(Wheel):
    def __init__(self):
        Wheel.__init__(self, 62.4, 20)

from math import pi
import logging
import time, os
import cv2
import numpy as np

# logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)5s: %(message)s")
log = logging.getLogger(__name__)

STUD_MM = 8
INCH_MM = 25.4

ONE_FOOT_CICLE_RADIUS_MM = (12 * INCH_MM) / 2
ONE_FOOT_CICLE_CIRCUMFERENCE_MM = 2 * pi * ONE_FOOT_CICLE_RADIUS_MM


mdiff = MoveDifferential(OUTPUT_A, OUTPUT_B, MyTire, 16 * STUD_MM)

# Image directory 
# directory = r'/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/refDataDayX100'
directory = r'/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData'
  
# Change the current directory  
# to specified directory  
os.chdir(directory)

# initialize the camera
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(-1)  # work-around for error "can't open camera by index"

# allow the camera to warmup
time.sleep(1)

time.sleep(8)  # delay for initial testing


try:

    # Use odometry
    mdiff.odometry_start()
    mdiff.odometry_coordinates_log()

    from ev3dev2.unit import DistanceFeet


    # Take a photo, then drive 10cm forward (incrementing y-position) 11 times
    for i in range(12):
        print("Take a photo...", i)  # for initial testing

        # flush the camera frame buffer
        for j in range(7):
            ret, frame = cap.read()
        
        # grab an image frame from the camera
        ret, frame = cap.read()

        # display the image on screen and wait for a keypress
        cv2.imshow("Image", frame)
        cv2.waitKey(2000)
        filename = str(i)+'cali.jpg'
        cv2.imwrite(filename,frame)

        # mdiff.on_to_coordinates(SpeedRPM(15), 0, (i*100))
        mdiff.on_for_distance(15, 100)
        # mdiff.turn_to_angle(SpeedRPM(5), 90)

        print("wait a few seconds after movement before taking another photo...")
        time.sleep(3)


    # Now go back to where we started and rotate in place to 90 degrees (facing orig forward direction)
    print("driving back to origin and rotating back to orig pose...")
    mdiff.on_to_coordinates(SpeedRPM(20), 0, 0)
    mdiff.turn_to_angle(SpeedRPM(5), 90)


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

cap.release()
cv2.destroyAllWindows()
