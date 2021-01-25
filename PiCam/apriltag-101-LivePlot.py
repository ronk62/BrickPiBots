#!/usr/bin/env python3
# 

'''
initial content from site: https://www.instructables.com/id/Automatic-Vision-Object-Tracking/

Added apriltag processing, based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
and https://github.com/swatbotics/apriltag

'''

import numpy as np
import cv2
import apriltag
import time, tty, sys, threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

global ii
ii = 0

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

cap = cv2.VideoCapture(0)


# setup for live graphing
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)


def animate(i):
    global cap
    global Xx, Xy, Xz
    global Yx, Yy, Yz
    global Zx, Zy, Zz
    global Xt, Yt, Zt
    global ODt
    global ii

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
                    
   
    ## uncomment this section to show video frames (warning:slow)
    cv2.imshow('gray', gray) ### <--- this line causes an error and fails to show image, in this implementation
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    #append the independant array then increment ii
    ODt = np.append(ODt, ii)
    ii = ii + 1


    ax1.clear()
    # ax1.plot(ODt, USr)
    ax1.plot(ODt, Xx, label='Xx')
    ax1.plot(ODt, Xy, label='Xy')
    ax1.plot(ODt, Xz, label='Xz')
    ax1.plot(ODt, Yx, label='Yx')
    ax1.plot(ODt, Yy, label='Yy')
    ax1.plot(ODt, Yz, label='Yz')
    ax1.plot(ODt, Zx, label='Zx')
    ax1.plot(ODt, Zy, label='Zy')
    ax1.plot(ODt, Zz, label='Zz')
    
    ax1.legend()

    ax2.clear()
    ax2.plot(ODt, Xt, label='Xt')
    ax2.plot(ODt, Yt, label='Yt')
    ax2.plot(ODt, Zt, label='Zt')
    
    ax2.legend()

### main ###
while(True):
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

cap.release()
cv2.destroyAllWindows()


