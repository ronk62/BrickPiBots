#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 8/30/2020     Ron King    - initial attempt to calculate and plot relative position
#                           from IR beacon angles (triangulation)
#
# 9/4/2020      Ron King    - load and plot the US point cloud data (compass data too)
#

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
#a = 125                             # (Beacon2-Beacon1) B-C = a
#a = 59                             # testing (Beacon2-Beacon1) B-C = a
a = 137                             # (Beacon2-Beacon1) B-C = a ...V902b-5,6,7

# # Location 1
# zeroir1hangle =  45.0
# zeroir2hangle =  357.0

# # Location 2
# zeroir1hangle =  62.87
# zeroir2hangle =  25.0

# # Location 3
# zeroir1hangle =  76.0
# zeroir2hangle =  47.0

# # Location 4
# zeroir1hangle =  79.0
# zeroir2hangle =  43.0

# # Location 5
# zeroir1hangle =  46.0
# zeroir2hangle =  354.0

# # Location 6
# zeroir1hangle =  83.0
# zeroir2hangle =  40.0

# # Location pseudo-6 hand-corrected for perfect 30-60-90 right-triangle
# zeroir1hangle =  100.0
# zeroir2hangle =  55.0

# # Location 7
# zeroir1hangle =  71.0
# zeroir2hangle =  13.0

# # Location 8
# # zeroir1hangle =  33.0 # original detected angle (using -5, 0, 5 method, as of 9/10/2020) low batteries - beacon1
# zeroir1hangle =  81.0 # corrected angle, using visual analysis of graphed IR data
# # beacon2 has near perfect angle and distance measurements and should be basis of r-theta positioning
# # a calculation and method to vote for preferred localization needs to be developed
# zeroir2hangle =  42.0

# # Location 9
# zeroir1hangle =  26.0      <---- more bogus data; this angle s/b close to 80
# zeroir2hangle =  40.0

## Locations 10, 11, 12 are all throw-away (most or all angles = '400')

# # Location 13
# zeroir1hangle =  90.0
# zeroir2hangle =  38.0

# # Location 14
# zeroir1hangle =  87.0
# zeroir2hangle =  42.0

# # Location 15
# zeroir1hangle =  85.0
# zeroir2hangle =  44.0


# # Location 22
# zeroir1hangle =  71.0
# zeroir2hangle =  16.0

# Location 24
zeroir1hangle =  46.0
zeroir2hangle =  0.0

# # Location made up (feasible, dummmy data)
# zeroir1hangle =  100.0
# zeroir2hangle =  55.0

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
b = sin(rad_b2beta) * (a / sin(rad_ralpha))
c = sin(rad_b1gamma) * (a / sin(rad_ralpha))
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
wCx = 137
wCy = 150

# beacon2 world frame coords
wBx = 137
wBy = wCy + a       # make this position relative to beacon1 y location and line 'a'


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


## world frame "room" configuration space (simple rectangle)

# define the bounding box of the rectangular space - lower left (origin) = swCorner
swCornerx = 0
swCornery = 0

neCornerx = 137
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


# load the US and compass point cloud data
print("")
print ("loading sample data from csv files")
# load from csv files
polarCoordsUSmean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsUSmean-V912b-23.csv', delimiter=',')
polarCoordsUSstd = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordsUSstd-V912b-23.csv', delimiter=',')
polarCoordscmpMean = np.loadtxt('/home/robot/ev3dev2Projects/polarCoordscmpMean-V912b-23.csv', delimiter=',')
print ("loads commplete")
print("")

# process the raw US and compass polar coord data into point cloud data
USx = []
USy = []
for i in range(len(polarCoordscmpMean)):
    # compass to std graph frame version with more skew for alignment with
    # cmpDegB1B2 (east wall)
    thetaRad = radians((415 - polarCoordscmpMean[i]) % 360)
    radius = polarCoordsUSmean[i]
    newx = (radius * cos(thetaRad)) + wAx1
    # print("newx = ", newx)
    newy =  (radius * sin(thetaRad)) + wAy1
    # print("newy = ", newy)
    # for testing
    # # cartesianCoords.append([i,i+1])
    USx.append([newx])
    USy.append([newy])


## graph the US point cloud data
plt.figure(3)

plt.scatter(USx,USy, label='US point cloud', color='r', s=25, marker="o")
# plt.scatter(Cx,Cy, label='beacon1 location', color='k', s=25, marker="o")
# plt.scatter(Bx,By, label='beacon2 location', color='c', s=25, marker="o")
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

# plt.scatter(wAx1, wAy1, label='robot location b1 ref', color='r', s=25, marker="o")
plt.scatter(wAx2, wAy2, label='robot location b2 ref', color='b', s=25, marker="o")
plt.scatter(wCx,wCy, label='beacon1 location', color='k', s=25, marker="o")
plt.scatter(wBx,wBy, label='beacon2 location', color='c', s=25, marker="o")
plt.axis('equal')
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('IR beacons - world frame position data V902b')
plt.legend()
plt.show()
