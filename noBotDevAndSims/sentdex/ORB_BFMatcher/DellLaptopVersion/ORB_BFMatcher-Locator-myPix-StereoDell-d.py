#!/usr/bin/env python3
# 

# 12/29/2021    - version 'd' adds AprilTag102 from saved images to help understand and
#                 validate the point cloud data
#

#### camera calibration info
# all units below measured in pixels:
#   fx = 604.8851295863385
#   fy = 606.0410127799453
#   cx = 320.0
#   cy = 240.0

# pastable into Python:
#   fx, fy, cx, cy = (604.8851295863385, 606.0410127799453, 320.0, 240.0)

import numpy as np
import cv2
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import axes3d
from matplotlib import style

import math
import apriltag
import time, os


##########################  AprilTag setup code  ##############################

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


# H1 displaced from origin with no rotation - for ref only
H1 = [[1,0,0,0.05],
[0,1,0,0],
[0,0,1,0.05],
[0,0,0,1]]

# Assign Eurler rotation angles
theta1Deg=0  # y rotation angle between H0 and H1 (positive values are cw when viewed top down in real world)

# convert angles from deg to radians
theta1Rad=math.radians(theta1Deg)

# Assign displacement values
ax1=0     # x displacement between H0 and H1 in meters
az1=0     # z displacement between H0 and H1 in meters


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

##########################  end AprilTag setup code  ###############################

## read grayscale
imgObj1 = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData11by1manyAprTags20211228/4cali.jpg' ,0)
imgScene1 = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData11by1manyAprTags20211228/5cali.jpg' ,0)

imgObj2 = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData11by1manyAprTags20211228/5cali.jpg' ,0)
imgScene2 = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData11by1manyAprTags20211228/6cali.jpg' ,0)


print(imgObj1.shape)
print(imgObj1.size)
print(imgObj1.dtype)

# plt.imshow(imgObj1)
# plt.imshow(imgScene1)
# plt.show()

############################# more AprilTag stuff ##########################

detector = apriltag.Detector()
result = detector.detect(imgScene1)
# for initial testing/devopment
print("dector result is ", result)
print("")
print("len of result is ", len(result))
print(type(result))
print("")


## extract contents of results list
for i, enum_result in enumerate(result):
    # print("i = ", i)
    # print("enum_result is... ")
    # print(enum_result.tostring())
    # print("")
    ##result_pose = detector.detection_pose(enum_result,camera_params=(600,600,320,240),tag_size=0.16, z_sign=1)
    result_pose = detector.detection_pose(enum_result,camera_params=(604.8851295863385, 606.0410127799453, 320.0, 240.0),tag_size=0.16, z_sign=1)  ## PiCam calibrated
    # result_pose = detector.detection_pose(enum_result,camera_params=(1304.8295760258923, 1305.9220320234388, 634.5881929729601, 508.28074560736405),tag_size=0.16, z_sign=1)  ## DellLaptop
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


############################# end more AprilTag stuff ######################

orb = cv2.ORB_create(nfeatures=3000)

kp1, des1 = orb.detectAndCompute(imgObj1,None)
kp2, des2 = orb.detectAndCompute(imgScene1,None)

# for testing
print("len(kp1)", len(kp1))
print("len(kp2)", len(kp2))

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

matches = bf.match(des1,des2)
matches = sorted(matches, key = lambda x:x.distance)

print("matches... ", matches)
print()
print("len matches... ", len(matches))


# img3 = cv2.drawMatches(imgObj1,kp1,imgScene1,kp2,matches[:10],None, flags=2)
# plt.imshow(img3)
# plt.show()


#### modified stuff to make compatible with New stuff

img_object = imgObj1
img_scene = imgScene1

good_matches = matches[:15]   # most sparce
# good_matches = matches[:1000]   # moderately sparce
# good_matches = matches   # no points trimmed

keypoints_obj = kp1
keypoints_scene = kp2

# img_matches = cv2.drawMatches(imgObj1,kp1,imgScene1,kp2,matches[:10],None, flags=2)



#### New stuff from 'https://docs.opencv.org/3.4.15/d7/dff/tutorial_feature_homography.html'

#-- Draw matches
img_matches = np.empty((max(img_object.shape[0], img_scene.shape[0]), img_object.shape[1]+img_scene.shape[1], 3), dtype=np.uint8)
# cv2.drawMatches(img_object, keypoints_obj, img_scene, keypoints_scene, good_matches[:10],img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
cv2.drawMatches(img_object, keypoints_obj, img_scene, keypoints_scene, good_matches,img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

#-- Localize the object
obj = np.empty((len(good_matches),2), dtype=np.float32)
scene = np.empty((len(good_matches),2), dtype=np.float32)
for i in range(len(good_matches)):
    #-- Get the keypoints from the good matches
    obj[i,0] = keypoints_obj[good_matches[i].queryIdx].pt[0]
    obj[i,1] = keypoints_obj[good_matches[i].queryIdx].pt[1]
    scene[i,0] = keypoints_scene[good_matches[i].trainIdx].pt[0]
    scene[i,1] = keypoints_scene[good_matches[i].trainIdx].pt[1]
H, _ =  cv2.findHomography(obj, scene, cv2.RANSAC)

print("")
print("obj = ", obj)
print("")
print("scene = ", scene)
print("")
print("H = ", H)
print("")


#-- Get the corners from the image_1 ( the object to be "detected" )
obj_corners = np.empty((4,1,2), dtype=np.float32)
obj_corners[0,0,0] = 0
obj_corners[0,0,1] = 0
obj_corners[1,0,0] = img_object.shape[1]
obj_corners[1,0,1] = 0
obj_corners[2,0,0] = img_object.shape[1]
obj_corners[2,0,1] = img_object.shape[0]
obj_corners[3,0,0] = 0
obj_corners[3,0,1] = img_object.shape[0]
scene_corners = cv2.perspectiveTransform(obj_corners, H)

print("")
print("obj_corners = ", obj_corners)
print("")
print("scene_corners = ", scene_corners)
print("")

#-- Draw lines between the corners (the mapped object in the scene - image_2 )
cv2.line(img_matches, (int(scene_corners[0,0,0] + img_object.shape[1]), int(scene_corners[0,0,1])),\
    (int(scene_corners[1,0,0] + img_object.shape[1]), int(scene_corners[1,0,1])), (0,255,0), 4)
cv2.line(img_matches, (int(scene_corners[1,0,0] + img_object.shape[1]), int(scene_corners[1,0,1])),\
    (int(scene_corners[2,0,0] + img_object.shape[1]), int(scene_corners[2,0,1])), (0,0,255), 4)
cv2.line(img_matches, (int(scene_corners[2,0,0] + img_object.shape[1]), int(scene_corners[2,0,1])),\
    (int(scene_corners[3,0,0] + img_object.shape[1]), int(scene_corners[3,0,1])), (0,255,0), 4)
cv2.line(img_matches, (int(scene_corners[3,0,0] + img_object.shape[1]), int(scene_corners[3,0,1])),\
    (int(scene_corners[0,0,0] + img_object.shape[1]), int(scene_corners[0,0,1])), (255,0,0), 4)


# reprint len matches
print()
print("reprint len matches... ", len(matches))


# #-- Show detected matches
# cv2.imshow('Good Matches & Object detection', img_matches)
# cv2.waitKey()



### extract the data for graphing

## matched points...
# obj...
objX = np.array([], dtype=float)
objY = np.array([], dtype=float)
for i in range(len(obj)):
    x,y = obj[i]
    objX = np.append(objX, x)
    objY = np.append(objY, y)



## matched points...
# scene...
sceneX = np.array([], dtype=float)
sceneY = np.array([], dtype=float)
for i in range(len(scene)):
    x,y = scene[i]
    sceneX = np.append(sceneX, x)
    sceneY = np.append(sceneY, y)



### almost disperity math...almost
# xDiffs = objX - sceneX
# # yDiffs = objY - sceneY  # for test


### Disperity math for 3d points
'''
X'p = leftFrame-x - 320
X''p = rightFrame-x - 320

Y'p = leftFrame-x - 240
Y''p = rightFrame-x - 240

Xp = X'p * B / -(X''p - X'p)

Yp = ((Y'p + Y''p) / 2) * B / -(X''p - X'p)

Zp = c * B / -(X''p - X'p)

c = fx = focal-length of camera = 604.885129586338
'''
B = 10  # baseline = 10cm
c = 604.885129586338    # focal-length of camera
Xp = np.array([], dtype=float)
Yp = np.array([], dtype=float)
Zp = np.array([], dtype=float)


## no exclusions based on objY/sceneY distance
# distance = 5
# for i in range(len(objX)):
#     if abs(objY[i] - sceneY[i]) > distance:
#         print("abs(objY[i] - sceneY[i]) =  ", abs(objY[i] - sceneY[i]))
#     xobj = objX[i] - 320
#     xscene = sceneX[i] - 320
#     X = xobj * B / -(xscene - xobj)
#     Y = ((objY[i] + sceneY[i]) / 2) * B / -(xscene - xobj)
#     Z = c * B / -(xscene - xobj)
#     Xp = np.append(Xp, X)
#     Yp = np.append(Yp, Y)
#     Zp = np.append(Zp, Z)


## exclude elements based on objY/sceneY distance and Z distance values out of range
distance = 5   # initially set to 15
for i in range(len(objX)):
    if abs(objY[i] - sceneY[i]) < distance:
        xobj = objX[i] - 320
        xscene = sceneX[i] - 320
        yobj = objY[i] - 240
        yscene =  sceneY[i] - 240
        X = xobj * B / -(xscene - xobj)
        Y = ((yobj + yscene) / 2) * B / -(xscene - xobj)
        Y = -Y    ## try later to change from camera Y to cartesian Y 
        Z = c * B / -(xscene - xobj)
        if Z > 0 and Z <= 1200:
            Xp = np.append(Xp, X)
            Yp = np.append(Yp, Y)
            Zp = np.append(Zp, Z)
        else:
            print("Z =  ", Z)
    else:
        print("abs(objY[i] - sceneY[i]) =  ", abs(objY[i] - sceneY[i]))


### Graph the data
print("")
print("creating graphs...")
print("")

plt.figure(1)

plt.scatter(objX,objY, label='obj/left-frame x,y coords', color='g', s=10, marker="o")
# plt.scatter(sceneX,sceneY, label='scene x,y coords', color='r', s=10, marker="o")
plt.axis('equal')
plt.xlabel('camera x')
plt.ylabel('camera y')
plt.title('left frame - stereo data')
plt.legend()
# plt.show()


plt.figure(2)

# plt.scatter(objX,objY, label='obj x,y coords', color='g', s=10, marker="o")
plt.scatter(sceneX,sceneY, label='scene/right-frame x,y coords', color='r', s=10, marker="o")
plt.axis('equal')
plt.xlabel('camera x')
plt.ylabel('camera y')
plt.title('right frame - stereo data')
plt.legend()
# plt.show()


plt.figure(3)

plt.scatter(Xp,Zp, label='Xp,Zp', color='b', s=10, marker="o")
plt.axis('equal')
plt.xlabel('Xp')
plt.ylabel('Zp')
plt.title('X vs Z depth / disperity - stereo data')
plt.legend()
# plt.show()


style.use('ggplot')

fig = plt.figure(4)
ax1 = fig.add_subplot(111, projection='3d')

ax1.scatter(Xp, Zp, Yp, c='b', marker='o')
# ax1.scatter(Xp, Zp, Yp, c='b', marker='o')
# ax1.scatter(Xp, Yp, c='b', marker='o')


ax1.set_xlabel('x axis')
ax1.set_ylabel('depth (Z) axis')
ax1.set_zlabel('height y axis')

plt.show()