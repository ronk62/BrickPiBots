#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 9/10/2020     Ron King    - created this to visualize live sensor data from BrickPi3
#                           - initial code came from BPDDwasdSLAMdataAccurV9.12b.py
#                           - stripped out all code that saves or loads data from files
#                           - decided to leave the wasd and motor stuff intact for now
#                           - removed sampling and statists (min, max, mean, stddev) to speed performance

#                           - DON'T Forget to start xming and export DISPLAY=10.0.0.9:0.0  (change IP addr as req'd)


import time, tty, sys, threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from math import sin, cos, radians

yVal = np.array([], dtype=np.int32)
ODt = np.array([], dtype=np.int32)       # array to index each time/tick (x axis independant var)
i = 0                   # "outer" iterator for dist point sampling
j = 0                   # "inner" iterator for dist point sampling

# setup for live graphing
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

t = time.time()

def animate(i):
    #t = time.time()
    global yVal, ODt
    j = sin(i)
    i = i + 1

    #append the arrays
    ODt = np.append(ODt, i)
    yVal = np.append(yVal, j)

    ax1.clear()
    ax1.plot(ODt, yVal)
    print("time diff = ", time.time() - t)


ani = animation.FuncAnimation(fig, animate, interval=50)
#plt.show(block='False')        
plt.show()        
    
