#!/usr/bin/env python3
# 

'''
initial content from site: https://www.instructables.com/id/Automatic-Vision-Object-Tracking/

Added apriltag processing, based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
and https://github.com/swatbotics/apriltag

'''
#############
# 5/5/2021, Ron King,   variation of apriltag-101-LivePlot.py with added code to capture n samples per
#                       animation cycle and gather min, max, mean, std deviation stats

import numpy as np
import math
import cv2
import apriltag
import time, tty, sys, threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

global ii, iii
ii = 0
iii = 0

Xx = np.array([], dtype=np.int32)
Xy = np.array([], dtype=np.int32)
Xz = np.array([], dtype=np.int32)

Yx = np.array([], dtype=np.int32)
Yy = np.array([], dtype=np.int32)
Yz = np.array([], dtype=np.int32)

Zx = np.array([], dtype=np.int32)
Zy = np.array([], dtype=np.int32)
Zz = np.array([], dtype=np.int32)

Xt = np.array([], dtype=np.int32)
Yt = np.array([], dtype=np.int32)
Zt = np.array([], dtype=np.int32)

ODt = np.array([], dtype=np.int32)       # array to index each time/tick (x axis independant var)

### for Stats
ZxMin = np.array([], dtype=np.int32)
ZxMax = np.array([], dtype=np.int32)
ZxMean = np.array([], dtype=np.int32)
ZxStd = np.array([], dtype=np.int32)

XtMin = np.array([], dtype=np.int32)
XtMax = np.array([], dtype=np.int32)
XtMean = np.array([], dtype=np.int32)
XtStd = np.array([], dtype=np.int32)

ZtMin = np.array([], dtype=np.int32)
ZtMax = np.array([], dtype=np.int32)
ZtMean = np.array([], dtype=np.int32)
ZtStd = np.array([], dtype=np.int32)

ODtStats = np.array([], dtype=np.int32)       # array to index each sample group (x axis indep var for stats)


cap = cv2.VideoCapture(0)

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# setup for live graphing
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(4,1,1)
ax2 = fig.add_subplot(4,1,2)
ax3 = fig.add_subplot(4,1,3)
ax4 = fig.add_subplot(4,1,4)


def animate(i):
    global cap
    global Xx, Xy, Xz
    global Yx, Yy, Yz
    global Zx, Zy, Zz
    global Xt, Yt, Zt
    global ODt, ODtStats
    global ii, iii
    global ZxMin, ZxMax, ZxMean, ZxStd
    global XtMin, XtMax, XtMean, XtStd
    global ZtMin, ZtMax, ZtMean, ZtStd


    # flush the camera frame buffer
    for i in range(7):
        ret, frame = cap.read()

    tic = time.time()    # for timing analysis
    print("start sample loop", tic)
    print("")

    #### sample loop ####
    for i in range(25):
        ret, frame = cap.read()
        # frame = cv2.flip(frame, -1) # Flip camera vertically
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = apriltag.Detector()
        result = detector.detect(gray)
        # # for initial testing/devopment
        # print("dector result is ", result)
        # print("")
        # print("len of result is ", len(result))
        # print(type(result))
        # print("")
        
        ## extract contents of results list and append arrays
        for i, enum_result in enumerate(result):
            print("i = ", i)
            print("enum_result is... ")
            print(enum_result.tostring())
            print("")
            result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
            print("")
            print("pose dector result is... ")
            print(result_pose)
            print("")
            for j, emum_result_pose in enumerate(result_pose):
                if j == 0:
                    # column 1
                    Xx = np.append(Xx, emum_result_pose[0][0])
                    Xy = np.append(Xy, emum_result_pose[1][0])
                    Xz = np.append(Xz, emum_result_pose[2][0])
                    
                    # column 2
                    Yx = np.append(Yx, emum_result_pose[0][1])
                    Yy = np.append(Yy, emum_result_pose[1][1])
                    Yz = np.append(Yz, emum_result_pose[2][1])
                    
                    # column 3
                    Zx = np.append(Zx, emum_result_pose[0][2])
                    Zy = np.append(Zy, emum_result_pose[1][2])
                    Zz = np.append(Zz, emum_result_pose[2][2])
                    
                    # column 4
                    Xt = np.append(Xt, emum_result_pose[0][3])
                    Yt = np.append(Yt, emum_result_pose[1][3])
                    Zt = np.append(Zt, emum_result_pose[2][3])

                    # calculate and print Robot heading (Z) Euler angle from Y axis rotation
                    # column 3, row 1
                    print("")
                    print("Robot heading (Z) Euler angle (camInTagFrame), from Y axis rotation (for Ref only, KODY KING!)... ")
                    #ZxRad = math.radians(math.asin((camInTagFrame[0][2])))
                    ZxDeg = math.degrees(math.asin((emum_result_pose[0][2])))
                    print(ZxDeg, "  degrees")
                    print("")
                        
            #append the independant array then increment ii
            ODt = np.append(ODt, ii)
            ii = ii + 1

    # updates stats arrays
    # Stats column 1
    ZxMin = np.append(ZxMin, np.min(Zx))
    ZxMax = np.append(ZxMax, np.max(Zx))
    ZxMean = np.append(ZxMean, np.mean(Zx))

    # Stats column 2
    XtMin = np.append(XtMin, np.min(Xt))
    XtMax = np.append(XtMax, np.max(Xt))
    XtMean = np.append(XtMean, np.mean(Xt))

    # Stats column 3
    ZtMin = np.append(ZtMin, np.min(Zt))
    ZtMax = np.append(ZtMax, np.max(Zt))
    ZtMean = np.append(ZtMean, np.mean(Zt))

    # Stats column 4
    ZxStd = np.append(ZxStd, np.std(Zx))
    XtStd = np.append(XtStd, np.std(Xt))
    ZtStd = np.append(ZtStd, np.std(Zt))

    #append the Stats independant array then increment iii
    ODtStats = np.append(ODtStats, iii)
    iii = iii + 1

    print("Loop time = ", time.time() - tic)
    print("")

    ## uncomment this section to show video frames (warning:slow)
    # cv2.imshow('gray', gray) ### <--- this line causes an error and fails to show image, in this implementation
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


    ### Plot the data

    ## Uncomment this next section for raw data
    ax1.clear()
    # ax1.plot(ODt, USr)
    ax1.plot(ODt, Xx, label='Xx')
    ax1.plot(ODt, Xy, label='Xy')
    ax1.plot(ODt, Xz, label='Xz')
    ax1.legend()

    ax2.clear()
    # ax2.plot(ODt, USr)
    ax2.plot(ODt, Yx, label='Yx')
    ax2.plot(ODt, Yy, label='Yy')
    ax2.plot(ODt, Yz, label='Yz')    
    ax2.legend()

    ax3.clear()
    # ax3.plot(ODt, USr)
    ax3.plot(ODt, Zx, label='Zx')
    ax3.plot(ODt, Zy, label='Zy')
    ax3.plot(ODt, Zz, label='Zz')    
    ax3.legend()

    ax4.clear()
    ax4.plot(ODt, Xt, label='Xt')
    ax4.plot(ODt, Yt, label='Yt')
    ax4.plot(ODt, Zt, label='Zt')
    ax4.legend()

    ## Uncomment this next section for stats data
    # ax1.clear()
    # # ax1.plot(ODt, USr)
    # ax1.plot(ODt, Xx, label='Xx')
    # ax1.plot(ODt, Xy, label='Xy')
    # ax1.plot(ODt, Xz, label='Xz')
    # ax1.legend()

    # ax2.clear()
    # # ax2.plot(ODt, USr)
    # ax2.plot(ODt, Yx, label='Yx')
    # ax2.plot(ODt, Yy, label='Yy')
    # ax2.plot(ODt, Yz, label='Yz')    
    # ax2.legend()

    # ax3.clear()
    # # ax3.plot(ODt, USr)
    # ax3.plot(ODt, Zx, label='Zx')
    # ax3.plot(ODt, Zy, label='Zy')
    # ax3.plot(ODt, Zz, label='Zz')    
    # ax3.legend()

    # ax4.clear()
    # ax4.plot(ODt, Xt, label='Xt')
    # ax4.plot(ODt, Yt, label='Yt')
    # ax4.plot(ODt, Zt, label='Zt')
    # ax4.legend()


### main ###
while(True):
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

cap.release()
cv2.destroyAllWindows()


