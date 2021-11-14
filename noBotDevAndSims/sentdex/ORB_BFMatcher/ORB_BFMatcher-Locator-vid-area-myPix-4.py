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
import time, os

#print(cv2.__version__)

debug = 1 # set to 1 or True for debug; 0 or False to turn debug off

def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

path = '/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData'

orb = cv2.ORB_create(nfeatures=3000)

### Import RefData Images
images = []
classnames = []
mylist = os.listdir(path)
print("Total Classes Detected = ", len(mylist))

for cl in mylist:
    imgCur = cv2.imread(f'{path}/{cl}',0)
    images.append(imgCur)
    classnames.append(os.path.splitext(cl)[0])
print("classnames = ", classnames)

def findDes(images):
    desList = []
    kpList = []
    n = 0
    for img in images:
        n = n + 1
        print("getting des for img n = ", n, time.time())
        kp, des = orb.detectAndCompute(img,None)
        # more details for image analysis
        if kp is None or des is None:
            kp = []
            des = []
        print("len(kp) = ", len(kp), "len(des) = ", len(des))
        desList.append(des)
        kpList.append(kp)
    return kpList, desList


def findID(img, kpList, desList, images, thresh = 7):      # chng to 7 for testing; typical thresh of 15 works well
    keypoints_obj, des2 = orb.detectAndCompute(img,None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matchList = []
    areaList = []
    finalVal = -1
    keypoints_scene = []
    img_scene = []
    des =[]
    # try:
    # for keypoints_scene, des in kpList, desList:
    for ii in range(len(desList)):
        keypoints_scene = kpList[ii]
        des  = desList[ii]
        img_scene = images[ii]
        # matches = bf.match(des,des2)
        matches = bf.match(des2,des)  # reversed des and des2 to match queryIdx and trainIdx below
        matches = sorted(matches, key = lambda x:x.distance)
        print("len(matches) = ", len(matches))
        matchList.append(len(matches)) # vestigial ?


        if len(matches) >= thresh:
            
            #### modified stuff to make compatible with New stuff
            img_object = img
            good_matches = matches
            ####

            if debug:
                #-- Draw matches
                img_matches = np.empty((max(img_object.shape[0], img_scene.shape[0]), img_object.shape[1]+img_scene.shape[1], 3), dtype=np.uint8)
                cv2.drawMatches(img_object, keypoints_obj, img_scene, keypoints_scene, good_matches[:3],img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

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

            if debug:
                #-- Draw lines between the corners (the mapped object in the scene - image_2 )
                cv2.line(img_matches, (int(scene_corners[0,0,0] + img_object.shape[1]), int(scene_corners[0,0,1])),\
                    (int(scene_corners[1,0,0] + img_object.shape[1]), int(scene_corners[1,0,1])), (0,255,0), 4)
                cv2.line(img_matches, (int(scene_corners[1,0,0] + img_object.shape[1]), int(scene_corners[1,0,1])),\
                    (int(scene_corners[2,0,0] + img_object.shape[1]), int(scene_corners[2,0,1])), (0,0,255), 4)
                cv2.line(img_matches, (int(scene_corners[2,0,0] + img_object.shape[1]), int(scene_corners[2,0,1])),\
                    (int(scene_corners[3,0,0] + img_object.shape[1]), int(scene_corners[3,0,1])), (0,255,0), 4)
                cv2.line(img_matches, (int(scene_corners[3,0,0] + img_object.shape[1]), int(scene_corners[3,0,1])),\
                    (int(scene_corners[0,0,0] + img_object.shape[1]), int(scene_corners[0,0,1])), (255,0,0), 4)
                
                #-- Show detected matches
                cv2.imshow('Good Matches & Object detection', img_matches)
                cv2.waitKey()


        else:
            scene_area = 0

        areaList.append(scene_area)
        ####

    # except:
    #     pass
    print("matchList... ", matchList)
    if len(matchList) != 0:
        if max(matchList) > thresh:
            # finalVal = matchList.index(max(matchList))
            finalVal = areaList.index(max(areaList))
            # print("max(matchList) = ", max(matchList))
            print("max(areaList) = ", max(areaList), "finalVal = ", finalVal)
        
    return finalVal


##### MAIN #####
print("Starting findDes routine", time.time())

kpList, desList = findDes(images)
print(len(desList))
# time.sleep(4)

cap = cv2.VideoCapture(0)


while True:
    tic = time.time()
    for i in range(8):
        ret, img2 = cap.read()
    imgOriginal = img2.copy()
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    # img2 = cv2.resize(img2, (1200, 640))

    id = findID(img2, kpList, desList, images)
    if id != -1:
        cv2.putText(imgOriginal,classnames[id], (50, 50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),3)

    cv2.imshow('Output',imgOriginal)
    if debug:
        cv2.waitKey()
    else:
        # cv2.waitKey(30)
        cv2.waitKey()
    fps = 1 / (time.time() - tic)
    print("fps = ", fps, "id = ", id, "classnames = ", classnames[id])

'''
## read grayscale
img1 = cv2.imread('/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/mypic-template.jpg' ,0)
img2 = cv2.imread('/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/mypic.jpg' ,0)

print(img1.shape)
print(img1.size)
print(img1.dtype)

# plt.imshow(img1)
# plt.imshow(img2)
# plt.show()


#kp1, des1 = orb.detectAndCompute(img1,None)
# kp2, des2 = orb.detectAndCompute(img2,None)

# bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# matches = bf.match(des1,des2)
# matches = sorted(matches, key = lambda x:x.distance)

print("matches... ", matches)
print()
print("len matches... ", len(matches))


# img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)
# plt.imshow(img3)
# plt.show()


#### modified stuff to make compatible with New stuff

img_object = img1
img_scene = img2

good_matches = matches

keypoints_obj = kp1
keypoints_scene = kp2

# img_matches = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None, flags=2)



#### New stuff from 'https://docs.opencv.org/3.4.15/d7/dff/tutorial_feature_homography.html'

#-- Draw matches
img_matches = np.empty((max(img_object.shape[0], img_scene.shape[0]), img_object.shape[1]+img_scene.shape[1], 3), dtype=np.uint8)
cv2.drawMatches(img_object, keypoints_obj, img_scene, keypoints_scene, good_matches[:80],img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

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


#-- Show detected matches
cv2.imshow('Good Matches & Object detection', img_matches)
cv2.waitKey()

'''
