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


cap = cv2.VideoCapture(0)
 
while(True):
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
        print("i = ", i)
        print("enum_result is... ")
        print(enum_result.tostring())
        print("")
        result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
        print("")
        print("pose dector result is... ")
        print(result_pose)
        print("")
    
    ## uncomment this section to show video frames (warning:slow)
    cv2.imshow('gray', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


