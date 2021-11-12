#!/usr/bin/env python3
# 

# import the necessary packages
import time, os
import cv2
import numpy as np

i = 0


# initialize the camera and grab a reference to the raw camera capture
cap = cv2.VideoCapture(0)  # primary cam
# cap = cv2.VideoCapture(1)  # secondary cam

# cap.release()
# cv2.destroyAllWindows()

# allow the camera to warmup
time.sleep(2)

while True:

    # grab an image frame from the camera
    ret, frame = cap.read()

    cv2.imshow('Image',frame)
    cv2.waitKey(1)




