#!/usr/bin/env python3
# 

# import the necessary packages
import time, os
import cv2
import numpy as np

i = 0

# Image directory 
# directory = r'/home/robot/ev3dev2Projects/PiCam/calibrationImages/20210215b'
directory = r'/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData'
  
# Change the current directory  
# to specified directory  
os.chdir(directory)

# initialize the camera and grab a reference to the raw camera capture
cap = cv2.VideoCapture(0)

# allow the camera to warmup
time.sleep(2)

cap.release()
cv2.destroyAllWindows()

time.sleep(1)

while True:
    # need to re-init each interation?
    cap = cv2.VideoCapture(0)

    # allow the camera to warmup
    time.sleep(1)

    # grab an image frame from the camera
    ret, frame = cap.read()

    # display the image on screen and wait for a keypress
    cv2.imshow("Image", frame)
    cv2.waitKey(0)
    filename = str(i)+'cali.jpg'
    cv2.imwrite(filename,frame)

    # increment i
    i += 1

    cap.release()
    cv2.destroyAllWindows()

