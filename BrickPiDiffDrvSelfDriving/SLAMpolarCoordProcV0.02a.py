#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 6/10/2020      Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMtestV0.01a.py as starting point
#                            - followed "Occupancy Grid Mapping" model, youtube, Bob Trenwith Robotics 5.3.2.3
#                            - created discretized cartesian model using previously captured polar coord data sets
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
bresLineRet = np.array([])  # holds the line coords returned from function

"""
N-D Bresenham line algo
"""
import numpy as np
def _bresenhamline_nslope(slope):
    """
    Normalize slope for Bresenham's line algorithm.

    >>> s = np.array([[-2, -2, -2, 0]])
    >>> _bresenhamline_nslope(s)
    array([[-1., -1., -1.,  0.]])

    >>> s = np.array([[0, 0, 0, 0]])
    >>> _bresenhamline_nslope(s)
    array([[ 0.,  0.,  0.,  0.]])

    >>> s = np.array([[0, 0, 9, 0]])
    >>> _bresenhamline_nslope(s)
    array([[ 0.,  0.,  1.,  0.]])
    """
    scale = np.amax(np.abs(slope), axis=1).reshape(-1, 1)
    zeroslope = (scale == 0).all(1)
    scale[zeroslope] = np.ones(1)
    normalizedslope = np.array(slope, dtype=np.double) / scale
    normalizedslope[zeroslope] = np.zeros(slope[0].shape)
    return normalizedslope

def _bresenhamlines(start, end, max_iter):
    """
    Returns npts lines of length max_iter each. (npts x max_iter x dimension) 

    >>> s = np.array([[3, 1, 9, 0],[0, 0, 3, 0]])
    >>> _bresenhamlines(s, np.zeros(s.shape[1]), max_iter=-1)
    array([[[ 3,  1,  8,  0],
            [ 2,  1,  7,  0],
            [ 2,  1,  6,  0],
            [ 2,  1,  5,  0],
            [ 1,  0,  4,  0],
            [ 1,  0,  3,  0],
            [ 1,  0,  2,  0],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  0]],
    <BLANKLINE>
           [[ 0,  0,  2,  0],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  0],
            [ 0,  0, -1,  0],
            [ 0,  0, -2,  0],
            [ 0,  0, -3,  0],
            [ 0,  0, -4,  0],
            [ 0,  0, -5,  0],
            [ 0,  0, -6,  0]]])
    """
    if max_iter == -1:
        max_iter = np.amax(np.amax(np.abs(end - start), axis=1))
    npts, dim = start.shape
    nslope = _bresenhamline_nslope(end - start)

    # steps to iterate on
    stepseq = np.arange(1, max_iter + 1)
    stepmat = np.tile(stepseq, (dim, 1)).T

    # some hacks for broadcasting properly
    bline = start[:, np.newaxis, :] + nslope[:, np.newaxis, :] * stepmat

    # Approximate to nearest int
    return np.array(np.rint(bline), dtype=start.dtype)

def bresenhamline(start, end, max_iter=5):
    """
    Returns a list of points from (start, end] by ray tracing a line b/w the
    points.
    Parameters:
        start: An array of start points (number of points x dimension)
        end:   An end points (1 x dimension)
            or An array of end point corresponding to each start point
                (number of points x dimension)
        max_iter: Max points to traverse. if -1, maximum number of required
                  points are traversed

    Returns:
        linevox (n x dimension) A cumulative array of all points traversed by
        all the lines so far.

    >>> s = np.array([[3, 1, 9, 0],[0, 0, 3, 0]])
    >>> bresenhamline(s, np.zeros(s.shape[1]), max_iter=-1)
    array([[ 3,  1,  8,  0],
           [ 2,  1,  7,  0],
           [ 2,  1,  6,  0],
           [ 2,  1,  5,  0],
           [ 1,  0,  4,  0],
           [ 1,  0,  3,  0],
           [ 1,  0,  2,  0],
           [ 0,  0,  1,  0],
           [ 0,  0,  0,  0],
           [ 0,  0,  2,  0],
           [ 0,  0,  1,  0],
           [ 0,  0,  0,  0],
           [ 0,  0, -1,  0],
           [ 0,  0, -2,  0],
           [ 0,  0, -3,  0],
           [ 0,  0, -4,  0],
           [ 0,  0, -5,  0],
           [ 0,  0, -6,  0]])
    """
    # Return the points as a single array
    return _bresenhamlines(start, end, max_iter).reshape(-1, start.shape[-1])



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
            # asciiMap[rows / y][cols / x] - reverse of what you might do intuitively!!
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
            # bresLine = np.array([[6,7], [abs(minx), abs(miny)]])
            # print(bresLine.shape)
            # print(bresLine)
            # bresLineRet = bresenhamline(bresLine, np.zeros(bresLine.shape[1]), max_iter=-1)
            '''
            - stubbed 4 line above
            - testing below
            '''
            bresLineStart = np.array([[6,7],[abs(minx), abs(miny)]])
            bresLineEnd = np.array([[abs(minx), abs(miny)],[2, 1]])
            print("bresLineStart.shape", bresLineStart.shape)
            print("bresLineStart", bresLineStart)
            print("bresLineEnd.shape", bresLineEnd.shape)
            print("bresLineEnd", bresLineEnd)
            bresLineRet = bresenhamline(bresLineStart, bresLineEnd, max_iter=-1)
            print("bresLineRet", bresLineRet)
            bresLineRet = bresLineRet.tolist()
            print("bresLineRet as list ", bresLineRet)
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

