'''
Some notes for future consideration:

Match test filters/qualifiers
- kp match count 
- area inside scene corners
- scene corner 'squareness' test
- scene bounding box rotation/orientation test (H=identity or whatever is expexted)
- ratio of kp match count to kp1 (obj) count
- remove outliers using the following...
-- average slope between matched kps (use absval)
-- points that fall outside scene corner bounding box

'''

import numpy as np
import cv2
import matplotlib.pyplot as plt

print(cv2.__version__)

## read grayscale
img1 = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/mypic-template.jpg' ,0)
img2 = cv2.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/sentdex/ORB_BFMatcher/mypic.jpg' ,0)

print(img1.shape)
print(img1.size)
print(img1.dtype)

# plt.imshow(img1)
# plt.imshow(img2)
# plt.show()

orb = cv2.ORB_create(nfeatures=3000)

kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# for testing
print("len(kp1)", len(kp1))
print("len(kp2)", len(kp2))

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

# good_matches = matches[:15]   # most sparce
# good_matches = matches[:1000]   # moderately sparce
good_matches = matches   # no points trimmed

keypoints_obj = kp1
keypoints_scene = kp2

# img_matches = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)



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

## keypoints
# # obj...
# objX = np.array([], dtype=np.float32)
# objY = np.array([], dtype=np.float32)
# for i in range(len(keypoints_obj)):
#     objX = np.append(objX, keypoints_obj[i].pt[0])
#     objY = np.append(objY, keypoints_obj[i].pt[1])


## matched points...
# scene...
sceneX = np.array([], dtype=float)
sceneY = np.array([], dtype=float)
for i in range(len(scene)):
    x,y = scene[i]
    sceneX = np.append(sceneX, x)
    sceneY = np.append(sceneY, y)

## keypoints
# # scene...
# sceneX = np.array([], dtype=np.float32)
# sceneY = np.array([], dtype=np.float32)
# for i in range(len(keypoints_scene)):
#     sceneX = np.append(sceneX, keypoints_scene[i].pt[0])
#     sceneY = np.append(sceneY, keypoints_scene[i].pt[1])


### almost disperity math...almost
# xDiffs = objX - sceneX
# # yDiffs = objY - sceneY  # for test


### Disperity math for 3d points
'''
X'p = abs(leftFrame-x - 320)
X''p = abs(rightFrame-x - 320)

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
# distance = 15
# for i in range(len(objX)):
#     X = objX[i] * B / -(sceneX[i] - objX[i])
#     #show Y elements outside range
#     if abs(objY[i] - sceneY[i]) > distance:
#         print("abs(objY[i] - sceneY[i]) =  ", abs(objY[i] - sceneY[i]))
#     Y = ((objY[i] + sceneY[i]) / 2) * B / -(sceneX[i] - objX[i])
#     Z = c * B / -(sceneX[i] - objX[i])
#     Xp = np.append(Xp, X)
#     Yp = np.append(Yp, Y)
#     Zp = np.append(Zp, Z)

## exclude elements based on objY/sceneY distance and negative val for X
distance = 5   # initially set to 15
for i in range(len(objX)):
    if abs(objY[i] - sceneY[i]) < distance:
        X = objX[i] * B / -(sceneX[i] - objX[i])
        if X >= 0:
            Y = ((objY[i] + sceneY[i]) / 2) * B / -(sceneX[i] - objX[i])
            Z = c * B / -(sceneX[i] - objX[i])
            Xp = np.append(Xp, X)
            Yp = np.append(Yp, Y)
            Zp = np.append(Zp, Z)
        else:
            print("X =  ", X)
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
plt.show()
