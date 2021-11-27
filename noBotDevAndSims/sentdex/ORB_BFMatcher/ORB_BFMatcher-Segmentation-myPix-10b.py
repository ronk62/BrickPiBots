# 11/24/2021    - created using ORB_BFMatcher-Locator-myPix-3 as starting point,
#                 but trimmed heavily to start with
#

import numpy as np
import cv2
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import shape

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

print("np.shape(img1) = ",np.shape(img1))
# print(img1.size)
# print(img1.dtype)

### ORB detection, fresh sample image to be segmented
orb = cv2.ORB_create(nfeatures=3000)

kp1, des1 = orb.detectAndCompute(img1,None)

lenkp1 = len(kp1)
print("length of kp1 (count) = ", lenkp1)

# define k min, max, and calculate initial 'guess' based on kp count
kmin = 2
kmax = 5
k = int(lenkp1/100)
if k < kmin:
    k=kmin
if k > kmax:
    k=kmax

# ## hard-code k = 2 for testing
# k = 2

print("initial k value for kmeans = ", k)

## for dev and testing
# show kp on image
outImage = np.asarray(img1)
img1kp = cv2.drawKeypoints(img1, kp1, outImage, color=(255,0,0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
cv2.imshow('img1 with keypoints', img1kp)
cv2.waitKey(0)


## extract the x and y coordinates that belong to all the keypoints; assign to kp1array
kp1array = np.array([], dtype=np.float32)
XY = np.array([], dtype=np.float32)
for i in range(lenkp1):
    XY = kp1[i].pt[0], kp1[i].pt[1]
    kp1array = np.append(kp1array, [XY]) 

kp1array = np.reshape(kp1array, (lenkp1, 2))

print("np.shape(kp1array) = ",np.shape(kp1array))
print()

# convert to np.float32
kp1array = np.float32(kp1array)


'''
# queryIdx magic to get x,y from kp1
# use model akin to below...

X = np.random.randint(25,50,(25,2))
Y = np.random.randint(60,85,(25,2))
kp1array = np.vstack((X,Y))

# after vstack --> np.shape(kp1array) =  (50, 2)

# convert to np.float32
kp1array = np.float32(kp1array)



# this is the shape of A and B after doing this...

A = kp1array[labels.ravel()==0]
B = kp1array[labels.ravel()==1]

# ---> np.shape(A) =  (25, 2)
# ---> np.shape(B) =  (25, 2)

# ...so clearly a 2D array, with n, 2 is correctly sized/shaped for scatter plot and/or as kp overlay on img1

'''


## apply kmeans clustering

# define more thresholds
kdensitymin = 3


#### this is wrong for keypoint processing (in the tutorial, this was used to process all pixels)...
# # reshape img1 data and create array
# img1arrayk = img1.reshape(-1,1)
# img1arrayk = np.float32(img1arrayk)

# Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

# Set flags (Just to avoid line break in the code)
flags = cv2.KMEANS_RANDOM_CENTERS

# Apply KMeans
compactness,labels,centers = cv2.kmeans(kp1array,k,None,criteria,10,flags)

print()
print("compactness = ", compactness)
print()
print("labels = ", labels)
print()
print("centers = ", centers)
print()


# # assign letter labels based on cluster numeric label  <--- needed for column vector/histogram scenario
# A = img1arrayk[labels==0]
# B = img1arrayk[labels==1]
# if len(img1arrayk[labels==2]) > 0:
#     C = img1arrayk[labels==2]
# if len(img1arrayk[labels==3]) > 0:
#     D = img1arrayk[labels==3]
# if len(img1arrayk[labels==4]) > 0:
#     E = img1arrayk[labels==4]

# print()

# Create Visualizations (histogram, scatter plot, etc)
# Now plot 'A' in red, 'B' in blue, 'centers' in yellow
# plt.hist(A,256,[0,256],color = 'r')
# plt.hist(B,256,[0,256],color = 'b')
# if len(img1arrayk[labels==2]) > 0:
#     plt.hist(C,256,[0,256],color = 'g')
# if len(img1arrayk[labels==3]) > 0:
#     plt.hist(D,256,[0,256],color = 'c')
# if len(img1arrayk[labels==4]) > 0:
#     plt.hist(E,256,[0,256],color = 'k')
# plt.hist(centers,32,[0,256],color = 'y')
# plt.show()


## do the scatter plot
# Now separate the data and assign to A,B...E, Note the flatten()
A = kp1array[labels.ravel()==0]
B = kp1array[labels.ravel()==1]
C = kp1array[labels.ravel()==2]
D = kp1array[labels.ravel()==3]
E = kp1array[labels.ravel()==4]


print("np.shape(A) = ",np.shape(A))
print("np.shape(B) = ",np.shape(B))
print("np.shape(C) = ",np.shape(C))
print("np.shape(D) = ",np.shape(D))
print("np.shape(E) = ",np.shape(E))


# Plot the data
plt.scatter(A[:,0],A[:,1],color = 'r')
plt.scatter(B[:,0],B[:,1],c = 'b')
plt.scatter(C[:,0],C[:,1],color = 'g')
plt.scatter(D[:,0],D[:,1],color = 'c')
plt.scatter(E[:,0],E[:,1],color = 'k')
plt.scatter(centers[:,0],centers[:,1],s = 80,c = 'y', marker = 's')
plt.xlabel('x'),plt.ylabel('y')
plt.show()


# Now convert back into uint8, and make original image shape
# centers = np.uint8(centers)
# origImg1 = centers[labels.flatten()]
# origImg1 = origImg1.reshape((img1.shape))


# cv2.imshow('origImg1',origImg1)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

'''
fail - need to ref index of kp based on match with new segmeted kp set 'A'

## for dev and testing
# show segmented kp on image
outImage = np.asarray(img1)
img1kp = cv2.drawKeypoints(img1, A, outImage, color=(255,0,0), flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
cv2.imshow('img1 with segmented keypoints', img1kp)
cv2.waitKey(0)

'''

for i in range(len(A)):
    x,y = A[i]
    img1kp = cv2.circle(img1, (x,y), 2, (0,0,255),4)

for i in range(len(B)):
    x,y = B[i]
    img1kp = cv2.circle(img1, (x,y), 2, (255,0,0),4)

cv2.imshow('img1 with segmented keypoints', img1kp)
cv2.waitKey(0)


print()

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