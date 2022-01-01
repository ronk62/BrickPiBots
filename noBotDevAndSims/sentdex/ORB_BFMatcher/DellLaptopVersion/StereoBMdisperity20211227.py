#!/usr/bin/env python3
# 

'''
content from site: 

https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html

'''

### 12/27/2021   - copy-pasted the basic code 
#

import numpy as np
import cv2
import matplotlib.pyplot as plt

from matplotlib import pyplot as plt
# imgL = cv.imread('tsukuba_l.png',0)
# imgR = cv.imread('tsukuba_r.png',0)

## read grayscale
imgL = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/imageLeft.jpg' ,0)
imgR = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/imageRight.jpg' ,0)

stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()

