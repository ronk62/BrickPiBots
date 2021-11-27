# 11/24/2021    - created using ORB_BFMatcher-Locator-myPix-3 as starting point,
#                 but trimmed heavily to start with
#

import numpy as np
import cv2
import matplotlib.pyplot as plt

print(cv2.__version__)

def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

## read grayscale
# fixed pair of images for sanity testing code
# img1 = cv2.imread('/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/opencv-feature-matching-template.jpg' ,0)
# img2 = cv2.imread('/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/opencv-feature-matching-image.jpg' ,0)
#
# symlinked images for testing scenarios
img1 = cv2.imread('/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/mypic-template.jpg' ,0)    # query image/object
img2 = cv2.imread('/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/mypic.jpg' ,0)             # training image/scene

# print(img1.shape)
# print(img1.size)
# print(img1.dtype)


#### Blob detection - ref: https://circuitdigest.com/tutorial/image-segmentation-using-opencv

# Set up detector with default parameters
# detector = cv2.SimpleBlobDetector_create()
## or...

### Initialize parameters for tuning

#initialize parameter setting using cv2.SimpleBlobDetector
params=cv2.SimpleBlobDetector_Params()

# Set area filtering parameters
params.filterByArea=True
params.minArea=300
params.maxArea=10000000

# Set circularity filtering parameters
params.filterByCircularity=False
params.minCircularity=0.9

# Set convexity filtering parameter
params.filterByConvexity=False
params.minConvexity=0.2

# Set inertia filtering parameter
params.filterByInertia=True
params.minInertiaRatio=0.001
params.maxInertiaRatio=100

# Create detector with parameter
detector = cv2.SimpleBlobDetector_create(params)


# Detect blobs
keypoints = detector.detect(img1)

# Draw detected blobs as red circles
#cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensure the
#size of circle corresponds to the size of blob
blank = np.zeros((480,640))
blobs = cv2.drawKeypoints(img1,keypoints,blank,(0,255,255),cv2.DRAW_MATCHES_FLAGS_DEFAULT)

## Show keypoints
cv2.imshow('blobs',blobs)
cv2.waitKey(0)
cv2.destroyAllWindows()




'''
orb = cv2.ORB_create(nfeatures=3000)

kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

matches = bf.match(des1,des2)
matches = sorted(matches, key = lambda x:x.distance)

print("matches... ", matches)
print()
print("len matches... ", len(matches))


# img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)
# plt.imshow(img3)
# plt.show()


#### modified stuff to make compatible with New stuff

img_object = img1
img_scene = img2

good_matches = matches[:15]

keypoints_obj = kp1
keypoints_scene = kp2

# img_matches = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)

## for testing and possible scoring element (mDistSum)
mDistSum = 0
for m in good_matches:
    print(des2[m.trainIdx])
    print("#########")
    print(des1[m.queryIdx])
    print("###")
    print(m.distance)
    mDistSum = mDistSum + m.distance
    print("############################################################################")
print("mDistSum = ", mDistSum)
###

#### New stuff from 'https://docs.opencv.org/3.4.15/d7/dff/tutorial_feature_homography.html'

#-- Draw matches
img_matches = np.empty((max(img_object.shape[0], img_scene.shape[0]), img_object.shape[1]+img_scene.shape[1], 3), dtype=np.uint8)
cv2.drawMatches(img_object, keypoints_obj, img_scene, keypoints_scene, good_matches[:15],img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

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

# caculate and print scene_area

x = [0,0,0,0,0]
y = [0,0,0,0,0]

for i in range(4):
    x[i], y[i] = scene_corners[i][0]
    # experimental
    if x[i] < 0:
        x[i] = 0
    if y[i] < 0:
        y[i] = 0
    if x[i] > 640:
        x[i] = 640
    if y[i] > 480:
        y[i] = 480

x[4], y[4] = x[0], y[0] 

scene_corners_xarray = x[0],x[1],x[2],x[3],x[4]
scene_corners_yarray = y[0],y[1],y[2],y[3],y[4]

print(scene_corners_xarray) # for dev/test
print(scene_corners_yarray) # for dev/test

scene_area = PolyArea(scene_corners_xarray, scene_corners_yarray)

print("")
print("scene_area = ", scene_area)
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

# reprint reprint mDistSum
print()
print("reprint mDistSum = ", mDistSum)


#-- Show detected matches
cv2.imshow('Good Matches & Object detection', img_matches)
cv2.waitKey()

'''