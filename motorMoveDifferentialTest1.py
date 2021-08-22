#!/usr/bin/env python3

# Date          Author      Change Notes
# 8/22/2021     Ron King    - created MyTire class and adjusted wheel_distance_mm
#                           - various other additions made while testing
#

"""
Used to experiment with the MoveDifferential class
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

# Drive in a rectangle
# mdiff.turn_to_angle(SpeedRPM(25), 120)
# mdiff.on_to_coordinates(SpeedRPM(25), 0, DistanceFeet(1).mm)
# mdiff.on_to_coordinates(SpeedRPM(25), DistanceFeet(2).mm, DistanceFeet(1).mm)
# mdiff.on_to_coordinates(SpeedRPM(25), DistanceFeet(2).mm, 0)
# mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)
# mdiff.turn_to_angle(SpeedRPM(25), 90)

# Use odometry to drive to specific coordinates
mdiff.on_to_coordinates(SpeedRPM(25), 600, 300)

# Now go back to where we started and rotate in place to 90 degrees
mdiff.on_to_coordinates(SpeedRPM(25), 0, 0)
mdiff.turn_to_angle(SpeedRPM(25), 90)

mdiff.odometry_coordinates_log()
mdiff.odometry_stop()