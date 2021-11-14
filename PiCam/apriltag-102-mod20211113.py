#!/usr/bin/env python3
# 

########  change log  ########
# 1/24/2021     - created from apriltag-101.py but added matrix math to invert the frame perspective
#               from camera->tag to tag->camera
#
# 1/31/2021     - added section to convert and show Robot heading (Z) in Euler angle for rotation in Y axis
#

### 2/15/2021   - Added the below after capturing chessboard impages with the piCam and then running
#                 the calibration code, calibrate_camera.py

# all units below measured in pixels:
#   fx = 604.8851295863385
#   fy = 606.0410127799453
#   cx = 320.0
#   cy = 240.0

# pastable into Python:
#   fx, fy, cx, cy = (604.8851295863385, 606.0410127799453, 320.0, 240.0)


# 3/13/2021     - added section to rotate tag to align with world frame coords
#

# 4/5/2021      - major updates to correct the approach for rotating the tag to align with world frame coords
#

# 5/28/2021     - typo correction and some clarifications/notes
#

'''
initial content from site: https://www.instructables.com/id/Automatic-Vision-Object-Tracking/

Added apriltag processing, based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
and https://github.com/swatbotics/apriltag

ref material for "rotate tag to align with world frame coords"
https://www.meccanismocomplesso.org/en/3d-rotations-and-euler-angles-in-python/

"Robotics 1 U1 (Kinematics) S5 (Homogeneous Transformation Matrix) P1 (HTM from Rotation Matrix)"

The main difference in the analysis is that Kinematics assumes a linkage between the
frames, while localization assumes tags placed in the base frame at a location and
rotation that is not rigidly linked to the base-frame. Likewise, the cam/bot is not rigidly
linked to the tags.

The result is that...
1) the relative tag rotation angles are not used for displacement calculations
2) the rotations and x,y displacements for tags are measured and entered as known data
3) the cam/bot matrix data comes from the processed vid capture and apriltag lib

ref. https://www.youtube.com/watch?v=fXewWpehAWw&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&index=17


Notations and code outline
--------------------------
H0 represents the base frame itself (with or without a displacement offset)
H1 represents the Apriltag
H2 represents the cam/bot

H1 is constructed from intuition due to initial condition simplicity and is for ref only

H0_1 is initially composed from R0_1 and d0_1 components using np.concatinate...
...then computed as the dot.product of H0 and H0_1

R0_1 and d0_1 are composed from angles, displacement values, and formulas

H0_2 is the cam/bot in the base frame and is the dot.product of H0_1 and the camInTagFrame


'''

import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
import apriltag
import time, os

#np.set_printoptions(precision=2,floatmode='fixed')
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# vars for rotation matrices and calculations
R0_1 = np.array([], dtype=np.int32)
d0_1 = np.array([], dtype=np.int32)

H0 = np.array([], dtype=np.int32)
H1 = np.array([], dtype=np.int32)
H0_1 = np.array([], dtype=np.int32)
H0_2 = np.array([], dtype=np.int32)

# H0 at origin
H0 = [[1,0,0,0],
[0,1,0,0],
[0,0,1,0],
[0,0,0,1]]

# # H0 NOT at origin
# H0 = [[1,0,0,0],
# [0,1,0,400],
# [0,0,1,0],
# [0,0,0,1]]


# # H1 displaced from origin with no rotation - not used...for ref only
# H1 = [[1,0,0,0.05],
# [0,1,0,0],
# [0,0,1,0.05],
# [0,0,0,1]]

# Assign Eurler rotation angles
theta1Deg=0  # y rotation angle between H0 and H1 (positive values are cw when viewed top down in real world)

# convert angles from deg to radians
theta1Rad=math.radians(theta1Deg)

# Assign displacement values
ax1=0     # x displacement between H0 and H1 in meters
az1=0       # z displacement between H0 and H1 in meters


### Define the rotation
# R0_1
R0_1 = [[np.cos(theta1Rad),0,np.sin(theta1Rad)],
[0,1,0],
[-np.sin(theta1Rad),0,np.cos(theta1Rad)]]


# Apply the displacement translations
d0_1 = [[ax1],[0],[az1]]

# setup vars to graph the unit vectors
# V0 location and rotation
x0 = H0[0][3]
y0 = H0[2][3]
vxx0 = [x0,x0 + H0[0][0]]
vxy0 = [y0,y0 + H0[2][0]]
vyx0 = [x0,x0 + H0[0][2]]
vyy0 = [y0,y0 + H0[2][2]]

print()
print("the x,y position of H0 is  ", x0,y0)
print()


H0_1 = np.concatenate((R0_1, d0_1), 1)
H0_1 = np.concatenate((H0_1, [[0,0,0,1]]), 0)

H0_1 = np.dot(H0, H0_1)

print()
print("H0_1 is  ")
print(np.matrix(H0_1))
print()

# extract the values from H0_1 for graphing
x1 = H0_1[0][3]
y1 = H0_1[2][3]
vxx1 = [x1,x1 + H0_1[0][0]]
vxy1 = [y1,y1 + H0_1[2][0]]
vyx1 = [x1,x1 + H0_1[0][2]]
vyy1 = [y1,y1 + H0_1[2][2]]

print()
print("the x,y position of H0_1 is  ", x1,y1)
print()

result = []

tagInCamFrame = np.array([[],[],[],[]], dtype=np.int32)
camInTagFrame = np.array([[],[],[],[]], dtype=np.int32)

cap = cv2.VideoCapture(0)

 
while(True):
    result = [] # clear list
    n = 12   # limit how many times we iterate looking for a tag
    i = 25   # load as many as 25 new frames to deal with some buffering in the pipeline
    # for timing analysis
    tic = time.time()
    while result == []:
        if i > 0:   # load as many as 25 new frames
            print("i = ", i)
            i = i - 1  # decrement i
            ret, frame = cap.read()
            # frame = cv2.flip(frame, -1) # Flip camera vertically
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detector = apriltag.Detector()
            result = detector.detect(gray)
            # for initial testing/devopment
            print("dector result is ", result)
            print("")
            print("len of result is ", len(result))
            print(type(result))
            print("")
        else:
            if n > 0:  # limit # of iterations; 12 times should do it
                print("n = ", n)
                n = n - 1  # decrement n
                i = 25     # reset i to 25
            else:
                print("tag not found")
                time.sleep(5)

    
    n = 12     # reset n to 12 each time we succeed in finding a tag

    ## extract contents of results list
    for i, enum_result in enumerate(result):
        # print("i = ", i)
        # print("enum_result is... ")
        # print(enum_result.tostring())
        # print("")
        ##result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
        result_pose = detector.detection_pose(enum_result,camera_params=(604.8851295863385, 606.0410127799453, 320.0, 240.0),tag_size=0.16, z_sign=1)
        # print("")
        # print("apriltag standard pose dector result is... ")
        # print(result_pose)
        # print("")

        for j, emum_result_pose in enumerate(result_pose):
            if j == 0:
                tagInCamFrame = emum_result_pose

                
        # Invert the frame perspective via matrix inversion
        # the x,y,z R and T vectors in this view show the camera/robot location relative to the tag
        camInTagFrame = np.linalg.inv(tagInCamFrame)

        # os.system("clear")
        print("")
        print("apriltag standard (tagInCamFrame) pose dector result is... ")
        print(np.matrix(tagInCamFrame))
        print("")
        
        # print camInTagFrame
        print("")
        print("inverted (camInTagFrame) pose dector result is... ")
        print(np.matrix(camInTagFrame))
        print("")

        # calculate and print cam/bot in base frame
        H0_2 = np.dot(H0_1, camInTagFrame)

        print("")
        print("cam/bot in base frame (H0_2) pose is... ")
        print(np.matrix(H0_2))
        print("")

        # extract the values from H0_2 for graphing
        x2 = H0_2[0][3]
        y2 = H0_2[2][3]
        vxx2 = [x2,x2 + H0_2[0][0]]
        vxy2 = [y2,y2 + H0_2[2][0]]
        vyx2 = [x2,x2 + H0_2[0][2]]
        vyy2 = [y2,y2 + H0_2[2][2]]

        print()
        print("the x,y position of H0_2 is  ", x2,y2)
        print()

        # calculate and print Robot heading (Z) Euler angle from Y axis rotation
        # column 3, row 1
        print("")
        print("Robot heading (Z) Euler angle (camInTagFrame), from Y axis rotation (for Ref only, KODY KING!)... ")
        #ZxRad = math.radians(math.asin((camInTagFrame[0][2])))
        ZxDeg = math.degrees(math.asin((camInTagFrame[0][2])))
        print(ZxDeg, "  degrees")
        print("")
        print("Loop time = ", time.time() - tic)


        ## graph the unit vectors
        fig, ax = plt.subplots()

        plt.scatter(x0,y0, label='base frame (v0) origin', color='r', s=25, marker="o")
        line1, = ax.plot(vxx0,vxy0, label='v0 x', lw=0.4, color='r', marker=">")
        line2, = ax.plot(vyx0,vyy0, label='v0 y', lw=0.4, color='b', marker="^")

        plt.scatter(x1,y1, label='apriltag (v1) origin', color='y', s=25, marker="o")
        line3, = ax.plot(vxx1,vxy1, label='v1 x', lw=0.4, color='y', marker="None")
        line4, = ax.plot(vyx1,vyy1, label='v1 y', lw=0.4, color='c', marker="None")

        plt.scatter(x2,y2, label='cam/bot (v2) origin', color='m', s=25, marker="o")
        line5, = ax.plot(vxx2,vxy2, label='v2 x', lw=0.4, color='m', marker="None")
        line6, = ax.plot(vyx2,vyy2, label='v2 y', lw=0.4, color='g', marker="None")

        plt.axis('equal')
        plt.xlabel('x-position')
        plt.ylabel('Z-position')
        plt.title('unit vectors')
        plt.legend()
        plt.show()

        print()



    
    # ## uncomment this section to show video frames (warning:slow)
    # cv2.imshow('gray', gray)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

cap.release()
cv2.destroyAllWindows()


