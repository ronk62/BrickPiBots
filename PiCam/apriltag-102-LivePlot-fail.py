#!/usr/bin/env python3
# 

'''
initial content from site: https://www.instructables.com/id/Automatic-Vision-Object-Tracking/

Added apriltag processing, based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
and https://github.com/swatbotics/apriltag

'''

### 2/15/2021   - Added the below after capturing chessboard impages with the piCam and then running
#                 the calibration code, calibrate_camera.py

# all units below measured in pixels:
#   fx = 604.8851295863385
#   fy = 606.0410127799453
#   cx = 320.0
#   cy = 240.0

# pastable into Python:
#   fx, fy, cx, cy = (604.8851295863385, 606.0410127799453, 320.0, 240.0)


import numpy as np
import math
import cv2
import apriltag
import time, tty, sys, threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

global ii
ii = 0

tagInCamFrame = np.array([[],[],[],[]], dtype=np.int32)
camInTagFrame = np.array([[],[],[],[]], dtype=np.int32)
RotCamInTagFrame = np.array([[],[],[],[]], dtype=np.int32)

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
ax1 = fig.add_subplot(4,1,1)
ax2 = fig.add_subplot(4,1,2)
ax3 = fig.add_subplot(4,1,3)
ax4 = fig.add_subplot(4,1,4)


def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta), 0],
                   [ 0           , 1, 0, 0           ],
                   [-math.sin(theta), 0, math.cos(theta), 0],
                   [0, 0, 0, 1]])

# theta = math.pi/4   # 45 degrees ccw

theta = math.pi/2   # 90 degrees ccw

# theta = -0.279253     # -16 degrees

R = Ry(theta)


def animate(i):
    global cap
    global tagInCamFrame
    global camInTagFrame
    global RotCamInTagFrame
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
        # print("i = ", i)
        # print("enum_result is... ")
        # print(enum_result.tostring())
        # print("")
        #result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
        result_pose = detector.detection_pose(enum_result,camera_params=(604.8851295863385, 606.0410127799453, 320.0, 240.0),tag_size=0.16, z_sign=1)
        # print("")
        # print("pose dector result is... ")
        # print(result_pose)
        # print("")
        for j, emum_result_pose in enumerate(result_pose):
            if j == 0:
                tagInCamFrame = emum_result_pose

                # Invert the frame perspective via matrix inversion
                # the x,y,z R and T vectors in this view show the camera/robot location relative to the tag
                camInTagFrame = np.linalg.inv(tagInCamFrame)

                # Rotate the frame perspective via matrix rotation
                RotCamInTagFrame = R * camInTagFrame

                print("")
                print("apriltag standard (tagInCamFrame) pose dector result is... ")
                print(np.matrix(tagInCamFrame))
                print("")
                
                # print camInTagFrame
                print("")
                print("inverted (camInTagFrame) pose dector result is... ")
                print(np.matrix(camInTagFrame))
                print("")
                print("Rotated camInTagFrame pose dector result is... ")
                print(np.matrix(RotCamInTagFrame))
                print("")

                # calculate and print Robot heading (Z) Euler angle from Y axis rotation
                # column 3, row 1
                print("")
                print("Robot heading (Z) Euler angle (camInTagFrame), from Y axis rotation (for Ref only, KODY KING!)... ")
                #ZxRad = math.radians(math.asin((camInTagFrame[0][2])))
                ZxDeg = math.degrees(math.asin((camInTagFrame[0][2])))
                print(ZxDeg, "  degrees")
                print("")

                # column 1
                Xx = np.append(Xx, camInTagFrame[0][0])
                Xy = np.append(Xy, camInTagFrame[1][0])
                Xz = np.append(Xz, camInTagFrame[2][0])
                
                # column 2
                Yx = np.append(Yx, camInTagFrame[0][1])
                Yy = np.append(Yy, camInTagFrame[1][1])
                Yz = np.append(Yz, camInTagFrame[2][1])
                
                # column 3
                Zx = np.append(Zx, camInTagFrame[0][2])
                Zy = np.append(Zy, camInTagFrame[1][2])
                Zz = np.append(Zz, camInTagFrame[2][2])
                
                # column 4
                Xt = np.append(Xt, camInTagFrame[0][3])
                Yt = np.append(Yt, camInTagFrame[1][3])
                Zt = np.append(Zt, camInTagFrame[2][3])

        #append the independant array then increment ii
        ODt = np.append(ODt, ii)
        ii = ii + 1
   
    ## uncomment this section to show video frames (warning:slow)
    # cv2.imshow('gray', gray) ### <--- this line causes an error and fails to show image, in this implementation
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


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

### main ###
while(True):
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

cap.release()
cv2.destroyAllWindows()


