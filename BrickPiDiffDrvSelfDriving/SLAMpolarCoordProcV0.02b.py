#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 6/10/2020     Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMtestV0.01a.py as starting point
#                           - followed "Occupancy Grid Mapping" model, youtube, Bob Trenwith Robotics 5.3.2.3
#                           - created discretized cartesian model using previously captured polar coord data sets
#
# 6/11/2020     Ron King    - had issues getting the numpy implementation of Bresenham's line algorithm to work
#                           ...there were negative x,y values in the results that made no sense
#                           - switched to a purely python impelemtation from https://github.com/encukou/bresenham/blob/master/bresenham.py
#

import time, tty, sys, threading
import math
import numpy as np
from ev3dev2 import list_devices

# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
# searchSLAM1 = True      # are we exexcuting the initial SLAM circle?
# i = 0                   # "outer" iterator for dist point sampling
# j = 0                   # "inner" iterator for dist point sampling

# SLAM vars
# polarCoords = []      # list to hold US distance (cm) with index = approx compass-angle (deg)
'''
The below pre-captured polarCoords are used for testing and development
'''
# from polar coord data capture1c:  36 data points at multiples of 10 deg (compass +/- 3 deg) location: dwnstrs bathroom SW corner
polarCoords = [127.30000000000001, 134.3, 132.4, 135.3, 156.4, 156.60000000000002, 171.4, 166.4, 61.6, 57.400000000000006, 55.400000000000006, 54.6, 54.6, 54.6, 55.400000000000006, 58.1, 71.8, 47.400000000000006, 43.900000000000006, 42.5, 41.1, 40.300000000000004, 40.1, 40.300000000000004, 41.0, 42.5, 41.1, 38.300000000000004, 36.5, 35.7, 35.300000000000004, 35.300000000000004, 35.7, 37.6, 40.800000000000004, 45.6]
cartesianCoords = []        # list to hold converted data
discretizedCoords = []      # list to hold cartesian coords set in 10-by-10 grid
asciiMap = []               # visual model of the Occupancy Grid Map
minx = 0                    # helper var to remove negative x offset
miny = 0                    # helper var to remove negative y offset
#bresLine = 0,0             # var to hold results from bresenham function
newx = 0                    # var to hold x results from bresenham function
newy = 0                    # var to hold y results from bresenham function


"""Implementation of Bresenham's line drawing algorithm
See en.wikipedia.org/wiki/Bresenham's_line_algorithm
"""


def bresenham(x0, y0, x1, y1):
    """Yield integer coordinates on the line from (x0, y0) to (x1, y1).
    Input coordinates should be integers.
    The result will contain both the start and the end point.
    """
    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy



def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print("Ready for keyboard commands...")
    print("")


    # Initialize the asciiMap
    for i in range(10):
        asciiMap.append([2,2,2,2,2,2,2,2,2,2])
    for i in range(10):
        print(asciiMap[9 - i], "    ", (9 - i))
    print("")
    print(" 0  1  2  3  4  5  6  7  8  9")


    ### Main Loop
    while (True):
        if x == 120: # x key pushed - exit program
            break
        if x == 99: # c key pushed
            # convert compass to std graph frame and convert polar to cartesian
            print("convert polar to cartesian", time.time())
            for i in range(36):
                # compass frame version
                ##thetaRad = math.radians((i + 1) * 10)
                # compass to std graph frame version
                thetaRad = math.radians(450 - ((i + 1) * 10) % 360)
                radius = polarCoords[i]
                newx = radius * math.cos(thetaRad)
                # print("newx = ", newx)
                newy =  radius * math.sin(thetaRad)
                # print("newy = ", newy)
                # for testing
                # # cartesianCoords.append([i,i+1])
                cartesianCoords.append([newx,newy])
            print("Cartesians = ", cartesianCoords)
            print("")
            print("Ready for keyboard commands...")
        if x == 100: # d key pushed
            # discretize the data
            print("discretize the data", time.time())
            for i in range(36):
                discx = math.ceil(.04 * cartesianCoords[i][0])   # ceil(1/r * x)  ; r = 25
                # print("i, discx = ", i, discx)
                discy = math.ceil(.04 * cartesianCoords[i][1])   # ceil(1/r * y)  ; r = 25
                # print("i, discx, discy ", i, discx, discy)
                discretizedCoords.append([discx,discy])
            print("discretizedCoords = ", discretizedCoords)
            print("")
            # remove any negative offsets in x or y
            for i in range(len(discretizedCoords)):
                print(discretizedCoords[i][0])
                if discretizedCoords[i][0] < minx:
                    minx = discretizedCoords[i][0]
                print(discretizedCoords[i][1])
                if discretizedCoords[i][1] < miny:
                    miny = discretizedCoords[i][1]
                print("minx, miny", minx, miny)
            if minx < 0:
                for i in range(len(discretizedCoords)):
                    discretizedCoords[i][0] = discretizedCoords[i][0] - minx
            if miny < 0:
                for i in range(len(discretizedCoords)):
                    discretizedCoords[i][1] = discretizedCoords[i][1] - miny
            print("discretizedCoords offsets removed = ", discretizedCoords)
            print("")
            print("Ready for keyboard commands...")
        if x == 109: # m key pushed
            # apply discretizedCoords to Occupancy Grid Map (asciiMap)
            # x and y need to be associated as follows...
            # asciiMap[rows / y][cols / x] - reverse of what you might think intuitively!!
            print("apply discretizedCoords to Occupancy Grid Map", time.time())
            for i in range(len(discretizedCoords)):
                asciiMap[discretizedCoords[i][1]][discretizedCoords[i][0]] = 1
            asciiMap[abs(miny)][abs(minx)] = 8
            ## for testing
            # asciiMap[5][9] = 8
            for i in range(10):
                print(asciiMap[9 - i], "    ", (9 - i))
            print("")
            print(" 0  1  2  3  4  5  6  7  8  9")
            print("")
            print("Ready for keyboard commands...")
        if x == 98: # b key pushed
            # apply bresenhamline algo
            print("apply bresenhamline algo", time.time())
            # bresenham bx0, by0 = abs(minx), abs(miny)], which = 1,2 when using polar coord data capture1c 
            bx0 = abs(minx)
            by0 = abs(miny)
            ### for testing
            # bx1 = 6
            # by1 = 7
            for i in range(len(discretizedCoords)):
                bx1 = discretizedCoords[i][0]
                by1 = discretizedCoords[i][1]
                for newx, newy in bresenham(bx0, by0, bx1, by1):
                    print("newx, newy  ", newx, newy)
                    asciiMap[newy][newx] = 0
            for i in range(10):
                print(asciiMap[9 - i], "    ", (9 - i))
            print("")
            print("Ready for keyboard commands...")
        if x == 112: # p pushed
            # print the data to screen
            print("print the data to screen", time.time())
            print("")
            print("Ready for keyboard commands...")
    
            """ ### new SLAM routine; initial 360 deg data capture
            elif searchSLAM1:  # if we have not yet circled for initial SLAM, do so now
                ### comment this code block and uncomment the below section for stationary testing
                # set speed (slow)
                spd = 5
                # initiate a CW 360 deg circle while capturing SLAM data; progress in segments to ensure full circle
                turnRatio = -1
                if i == 0:
                    print("Initiating polarCoords capture... ", polarCoords, i)
                    # turn to compass val between 350 and 360 to initialize rotational position
                    if compassVal > (350) and compassVal < (360):
                        i = 1
                        j = 1
                elif i == 1:                # iterate through 350 deg
                    # turn to compass val j * 10 +/- 3
                    if compassVal > ((j * 10) -3) and compassVal < ((j * 10) +3):
                        polarCoords.append(usDistCmVal)
                        print("appended polarCoords...  ", polarCoords, j)
                        j = j + 1
                        if j == 36:          # iterate all but the final data point
                            i = 2           # update state to move to next/final data point collection
                elif i == 2:                # collect final data point and reset vars
                    # turn to compass val 355 +/-3
                    if compassVal > (352) and compassVal < (358):
                        polarCoords.append(usDistCmVal)
                        i = 0
                        j = 0
                        searchSLAM1 = False
                        print("SLAM1 data collection complete")
                        ### added for initial SLAM testing
                        # stop and return to human pilot mode
                        mL.on(0, brake=False)
                        mR.on(0, brake=False)
                        print("polarCoords:", polarCoords)
                        pilot_mode = 1  # set this back to human mode ('-1' means auto-pilot)
                else:
                    i = 0 """
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(0.2)

