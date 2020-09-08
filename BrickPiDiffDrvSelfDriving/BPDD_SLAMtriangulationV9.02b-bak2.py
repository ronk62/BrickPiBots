#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 8/30/2020     Ron King    - initial attempt to calculate and plot relative position
#                           from IR beacon angles

import time, tty, sys
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, radians

'''
Sample data:

Location 1 --- data files V902b-1.csv

plus5ir1hangle =  39.0
zeroir1hangle =  45.0
minus5ir1hangle =  50.0

plus5ir2hangle ...not found
zeroir2hangle =  357.0
minus5ir2hangle =  1.86


Location 2 --- data files V902b-2.csv

plus5ir1hangle =  56.0
zeroir1hangle =  62.87
minus5ir1hangle =  73.0

plus5ir2hangle =  11.0
zeroir2hangle ...not found
minus5ir2hangle =  28.0


Location 3 --- data files V902b-3.csv

plus5ir1hangle ...not found
zeroir1hangle =  76.0
minus5ir1hangle =  80.9

plus5ir2hangle =  39.0
zeroir2hangle =  47.0
minus5ir2hangle =  49.0


Location 4 --- data files V902b-4.csv

plus5ir1hangle =  74.0
zeroir1hangle =  79.0
minus5ir1hangle =  82.0

plus5ir2hangle ...not found
zeroir2hangle =  43.0
minus5ir2hangle ...not found


# Helper info below...

Definitions based on std conventions:

  ASCII Art

                 A (robot)
                / \
              /     \
          b /         \ c
          /             \
        /                 \
      C (Beacon1)--------- B (Beacon2)
                  a

  vertices
  - Robot   Position = A
  - Beacon2 Position = B
  - Beacon1 Position = C

  angles
  - (Robot)   vertex A = ralpha
  - (Beacon2) vertex B = b2beta
  - (Beacon1) vertex C = b1gamma

  lines
  - (Robot-Beacon2)   A-B = c
  - (Robot-Beacon1)   A-C = b
  - (Beacon2-Beacon1) B-C = a

    a                    b                    c
---------      =     ---------      =     ---------      
sin(alpha)           sin(beta)            sin(gamma)

'''

## calculate lengths of triangle from known length and angles

# known compass angle for line (Beacon2-Beacon1) B-C; this is the ref 'y' axis
# using 325 deg for now
cmpDegB1B2 = 325

# length of the line between beacon 1 and 2
# using 125 cm for now
a = 125                             # (Beacon2-Beacon1) B-C = a

# # Location 1
# zeroir1hangle =  45.0
# zeroir2hangle =  357.0

# # Location 2
# zeroir1hangle =  62.87
# zeroir2hangle =  25.0

# # Location 3
# zeroir1hangle =  76.0
# zeroir2hangle =  47.0

# Location 4
zeroir1hangle =  79.0
zeroir2hangle =  43.0

# # Location made up (feasible, dummmy data)
# zeroir1hangle =  327.0
# zeroir2hangle =  326.0

# relative angles - calculate the offset (360 - cmpDegB1B2) and add to to each val
offsetDeg = 360 - cmpDegB1B2
rel_cmpDegB1B2 = 360
# use modulo 360 to deal with wrap-around
rel_zeroir1hangle = ((zeroir1hangle + offsetDeg) % 360)
rel_zeroir2hangle = ((zeroir2hangle + offsetDeg) % 360)
print("")
print("rel_zeroir1hangle = ", rel_zeroir1hangle)
print("rel_zeroir2hangle = ", rel_zeroir2hangle)

# angles at vertices
b1gamma = rel_cmpDegB1B2 - (rel_zeroir1hangle + 180)
b2beta  = rel_zeroir2hangle
ralpha = 180 - (b1gamma + b2beta)
print("")
print("b1gamma = ", b1gamma)
print("b2beta = ", b2beta)
print("ralpha = ", ralpha)
print("")

# convert angles from degrees to rads
rad_b1gamma = radians(b1gamma)
rad_b2beta  = radians(b2beta)
rad_ralpha  = radians(ralpha)
print("rad_b1gamma = ", rad_b1gamma)
print("rad_b2beta = ", rad_b2beta)
print("rad_ralpha = ", rad_ralpha)
print("")

# line lengths based on known quantities and law of sines
b = sin(rad_b2beta) * (125 / sin(rad_ralpha))
c = sin(rad_b1gamma) * (125 / sin(rad_ralpha))
print("c = ", c)
print("b = ", b)
print("a = ", a)
print("")

# rotate rel_zeroir1hangle and rel_zeroir2hangle 90 deg from compass to std graph
# orientation and convert angles from degrees to rads
rot_zeroir1hangle = radians((450 - rel_zeroir1hangle) % 360)
rot_zeroir2hangle  = radians((450 - rel_zeroir2hangle) % 360)
print("zero ir h angle rotated 90 deg...")
print("rot_zeroir1hangle = ", rot_zeroir1hangle)
print("rot_zeroir2hangle = ", rot_zeroir2hangle)
print("")


## triagulate relative location (robot position is the origin)
Cx = b * cos(rot_zeroir1hangle)
Cy = b * sin(rot_zeroir1hangle)

Bx = c * cos(rot_zeroir2hangle)
By = c * sin(rot_zeroir2hangle)

print("Cx, Cy (beacon1) = ", Cx, Cy)
print("Bx, By (beacon2) = ", Bx, By)
print("")
print("")


## graph the relative position data
plt.figure(1)
plt.scatter(0,0, label='robot location', color='r', s=25, marker="o")
plt.scatter(Cx,Cy, label='beacon1 location', color='k', s=25, marker="o")
plt.scatter(Bx,By, label='beacon2 location', color='c', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('IR beacons - robot frame relative position data V902b')
plt.legend()
# plt.show()


## convert from robot frame coords to world frame
# beacon1 world frame coords
wCx = 250
wCy = 50

# beacon2 world frame coords
wBx = 250
wBy = 175


# robot world frame coords using beacon1 ref
wAx1 = wCx - Cx
wAy1 = wCy - Cy

# robot world frame coords using beacon2 ref
wAx2 = wBx - Bx
wAy2 = wBy - By

print("wCx, wCy (beacon1 world frame location) = ", wCx, wCy)
print("wBx, wBy (beacon2 world frame location) = ", wBx, wBy)
print("")
print("wAx1, wAy1 (robot world frame location, beacon1 ref) = ", wAx1, wAy1)
print("wAx2, wAy2 (robot world frame location, beacon2 ref) = ", wAx2, wAy2)
print("")
print("")


## graph the world frame "room" and position data
plt.figure(2)
plt.scatter(wAx1, wAy1, label='robot location b1 ref', color='r', s=25, marker="o")
plt.scatter(wAx2, wAy2, label='robot location b2 ref', color='b', s=25, marker="o")
plt.scatter(wCx,wCy, label='beacon1 location', color='k', s=25, marker="o")
plt.scatter(wBx,wBy, label='beacon2 location', color='c', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('IR beacons - world frame position data V902b')
plt.legend()
plt.show()
