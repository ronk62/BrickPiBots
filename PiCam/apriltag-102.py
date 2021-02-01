#!/usr/bin/env python3
# 

########  change log  ########
# 1/24/2021 -   created from apriltag-101.py but added matrix math to invert the frame perspective
#               from camera->tag to tag->camera
#
# 1/31/2021 -   added section to convert and show Robot heading (Z) in Euler angle for rotation in Y axis
#

'''
initial content from site: https://www.instructables.com/id/Automatic-Vision-Object-Tracking/

Added apriltag processing, based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
and https://github.com/swatbotics/apriltag

'''

import numpy as np
import math
import cv2
import apriltag
import time, os

#np.set_printoptions(precision=2,floatmode='fixed')
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


tagInCamFrame = np.array([[],[],[],[]], dtype=np.int32)
camInTagFrame = np.array([[],[],[],[]], dtype=np.int32)

cap = cv2.VideoCapture(0)
 
while(True):
    # for timing analysis
    tic = time.time()
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
    
    ## extract contents of results list
    for i, enum_result in enumerate(result):
        # print("i = ", i)
        # print("enum_result is... ")
        # print(enum_result.tostring())
        # print("")
        result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
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

        os.system("clear")
        print("")
        print("apriltag standard (tagInCamFrame) pose dector result is... ")
        print(np.matrix(tagInCamFrame))
        print("")
        
        # calculate and print camInTagFrame
        print("")
        print("inverted (camInTagFrame) pose dector result is... ")
        print(np.matrix(camInTagFrame))
        print("")

        # calculate and print Robot heading (Z) Euler angle from Y axis rotation
        # column 3, row 1
        print("")
        print("Robot heading (Z) Euler angle (camInTagFrame), from Y axis rotation (for Ref only, KODY KING!)... ")
        #ZxRad = math.radians(math.asin((camInTagFrame[0][2])))
        ZxDeg = math.degrees(math.asin((camInTagFrame[0][2])))
        print(ZxDeg, "  degrees")
        print("")
        print("Loop time = ", time.time() - tic)

    
    ## uncomment this section to show video frames (warning:slow)
    cv2.imshow('gray', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


