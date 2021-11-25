#!/usr/bin/env python3
# 

# Date          Author      Change Notes      
# 11/21/2021    Ron King    - created from ORB_BFMatcher-Locator-vid-score-myPix-4
#                           - this model uses manually extracted/cropped image sections with prominent
#                             landmarks selected for future triagulation
#                           - swapped Obj and Scene for kp and des processing since the scoring mechanism
#                             is based on finding the landmark in the scene
#
# 11/24/2021    Ron King    - test code to help understand the anatomy of DMatch
#                           - ref: https://docs.opencv.org/4.2.0/dc/dc3/tutorial_py_matcher.html 
#                           - section ("What is this Matcher Object?")
#

#                           Dont forget to rediect X and start xming as needed

#######  SOME EARLY NOTES ABOUT THIS TECHNIQUE  #######
## * - scoring does not match desired ranking effect, based on debug observations
## * - Homography perhaps does not make sense in this application, since the goal is not reprojection match
## * - most likely, need to wrap a bounding box around matching key points in the scene and find the 'x' middle
## * - then rotate bot so that the 'x' middle in centered in the impage (aligned with x middle pixel ~320)
## * - at this time, use the compass and gyro to get angles for this landmark
## * - repeat for landmark #2, then do triangulation with law of sines
#
#
#######  IDEAS FOR NEW PIPELINE AND TESTING  #######
## * - normalize scene and obj points so that you are comparing like-values for pixel x, y
## * - use the avg x and avg y for 'centriod', or send to kmeans clustering to get centroids (plural)
## * - use a LiveStats approach to gather avg, min, max, stdev for x, and y from a given location
## * --- do the above for scene x,y pixels (before normalizing) to start with to see if stable
## * --- repeat the above from ever-further locations if values are predictable from same location
## * - for scoring, use normalized scene and obj bounding boxes for area and least squares formula
## * - if a single landmark can be repeatably found, try to get a second and then triagulate
## * - see if delta y (max - min) can be used as a relative distance indicator
## * - see if delta x (centroids) from one frame to another can be used to measure distance to landmark
## * - perhaps go back to normal obj/scene image collection without cropping the obj
## * --- use kmeans clusters to collect a group of kps and their coords to use a landmarks
## * --- this would allow for automated landmark extraction and use

#######  Update: IDEAS FOR NEW PIPELINE AND TESTING  #######
## * - Segmentation: k-means clustering
## * --- use Blob detection at first to evaluate scene image
## * --- find 'useful blobs' (larger size? some means of rough density measurement?)
## * --- use use count of useful blobs to determin 'k' for k-means clustering
## * - BoVW: measure the usefulness of desciptors for raw use as words in the dictionary
## * --- using a very small set of qry and train images (1 to start with), meausre some statistics
## * ---- gather and plot total count of descriptor per cycle
## * ---- gather and plot count of unique descriptors over time (show count per cycle)
## * --- repeat the above with a growing set of qry and train images
## * - Explore scoring systems:
## * --- revisit Homography with Area and least-squares, perhaps compare like-clusters
## * ----- look at training image extracted cluster and compare to qry image after  warpPerspective
## * ----- ref: https://www.youtube.com/watch?v=cA8K8dl-E6k
## * --- look into BoVW histogram scoring, using descriptors as words
## * --- if word count (decriptor count) becomes unmanagable (size, cpu-cost), look at subset per cluster


'''
Some notes for future consideration:

Match test filters/qualifiers
- kp match count  (this turned out to be not the best as certain scenes have few kps)
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

from numpy.lib.function_base import average

#print(cv2.__version__)

# set debug level
debug = 2 # set to 0 for no debug, 1 for lite debug, or 2 for heavy debug

def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

path = '/home/robot/ev3dev2Projects/noBotDevAndSims/sentdex/ORB_BFMatcher/refData'

orb = cv2.ORB_create(nfeatures=3000)
## for testing
# orb = cv2.ORB_create(nfeatures=30)

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
    keypoints_scene, des2 = orb.detectAndCompute(img,None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matchList = []
    areaList = []
    scoreList = []
    finalVal = -1
    keypoints_obj = []
    img_object = []
    des =[]
    # try:
    # for keypoints_obj, des in kpList, desList:
    for ii in range(len(desList)):
        keypoints_obj = kpList[ii]
        des  = desList[ii]
        img_object = images[ii]
        ## for testing
        # print("des (obj) = ", des)
        print("min/max/avg/len des (obj) = ", np.min(des), np.max(des), np.average(des), len(des))
        # print("des2 (scene) = ", des2)
        print("min/max/avg/len des2 (scene) = ", np.min(des2), np.max(des2), np.average(des2), len(des2))
        # print()
        # matches = bf.match(des,des2)
        matches = bf.match(des2,des)  # reversed des and des2 to match queryIdx and trainIdx below
        matches = sorted(matches, key = lambda x:x.distance)
        print("len(matches) = ", len(matches))
        ## for testing
        # for m in matches:
        #     print(des[m.trainIdx])

        matchList.append(len(matches)) # vestigial ?

        if len(matches) >= thresh:
            
            #### modified stuff to make compatible with New stuff
            img_scene  = img
            good_matches = matches[:15]
            print("good_matches = ", good_matches)
            print("")
            
            ## for testing and possible scoring element (mDistSum)
            mDistSum = 0
            for m in good_matches:
                print(des[m.trainIdx])
                print("#########")
                print(des2[m.queryIdx])
                print("###")
                print(m.distance)
                mDistSum = mDistSum + m.distance
                print("############################################################################")
            print("mDistSum = ", mDistSum)
            ###
            
            if debug > 0:
                #-- Draw matches
                img_matches = np.empty((max(img_scene .shape[0], img_object.shape[0]), img_scene .shape[1]+img_object.shape[1], 3), dtype=np.uint8)
                # cv2.drawMatches(img_scene , keypoints_scene, img_object, keypoints_obj, good_matches[:3],img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                cv2.drawMatches(img_scene , keypoints_scene, img_object, keypoints_obj, good_matches[:10],img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

            #-- Localize the object
            scene = np.empty((len(good_matches),2), dtype=np.float32)
            scenex = np.empty((len(good_matches),1), dtype=np.float32)
            sceney = np.empty((len(good_matches),1), dtype=np.float32)
            obj = np.empty((len(good_matches),2), dtype=np.float32)
            for i in range(len(good_matches)):
                #-- Get the keypoints from the good matches
                scene[i,0] = keypoints_scene[good_matches[i].queryIdx].pt[0]
                scenex[i] = scene[i,0]
                scene[i,1] = keypoints_scene[good_matches[i].queryIdx].pt[1]
                sceney[i] = scene[i,1]
                obj[i,0] = keypoints_obj[good_matches[i].trainIdx].pt[0]
                obj[i,1] = keypoints_obj[good_matches[i].trainIdx].pt[1]

            print("")
            print("scene = ", scene)
            print("")

            print("scenex = ", scenex)
            print("")
            print("sceney = ", sceney)
            print("")

            print("min, max, stdev, avg scenex = ", np.min(scenex), np.max(scenex), np.std(scenex), np.average(scenex))
            print("")

            print("min, max, stdev, avg sceney = ", np.min(sceney), np.max(sceney), np.std(sceney), np.average(sceney))
            print("")


            print("obj = ", obj)
            print("")


            H, _ =  cv2.findHomography(scene, obj, cv2.RANSAC)

            print("H = ", H)
            print("")


            #-- Get the corners from the image_1 ( the object to be "detected" )
            scene_corners = np.empty((4,1,2), dtype=np.float32)
            scene_corners[0,0,0] = 0
            scene_corners[0,0,1] = 0
            scene_corners[1,0,0] = img_scene .shape[1]
            scene_corners[1,0,1] = 0
            scene_corners[2,0,0] = img_scene .shape[1]
            scene_corners[2,0,1] = img_scene .shape[0]
            scene_corners[3,0,0] = 0
            scene_corners[3,0,1] = img_scene .shape[0]
            obj_corners = cv2.perspectiveTransform(scene_corners, H)

            print("")
            print("scene_corners = ", scene_corners)
            print("")
            print("obj_corners = ", obj_corners)
            print("")

            # caculate and print sum of diff (scene_corners - obj_corners) squared
            sumOfSqrs = 0
            ox = [0,0,0,0]
            oy = [0,0,0,0]
            sx = [0,0,0,0]
            sy = [0,0,0,0]
            for i in range(4):
                ox[i], oy[i] = scene_corners[i][0]
                sx[i], sy[i] = obj_corners[i][0]
                sumOfSqrs = sumOfSqrs + (ox[i] - (sx[i]))**2 + (oy[i] - (sy[i]))**2
            print("sumOfSqrs = ", sumOfSqrs)


            # caculate and print scene_area
            x = [0,0,0,0,0]
            y = [0,0,0,0,0]

            for i in range(4):
                x[i], y[i] = obj_corners[i][0]
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

            obj_corners_xarray = x[0],x[1],x[2],x[3],x[4]
            obj_corners_yarray = y[0],y[1],y[2],y[3],y[4]

            print(obj_corners_xarray) # for dev/test
            print(obj_corners_yarray) # for dev/test

            scene_area = PolyArea(obj_corners_xarray, obj_corners_yarray)

            print("")
            print("scene_area = ", scene_area)
            print("")

            # calculate score based on scene_area / 1 + (sumOfSqrs)
            score = scene_area / (1 + sumOfSqrs)
            print("score = ", score)
            print("")

            if debug > 0:
                #-- Draw lines between the corners (the mapped object in the scene - image_2 )
                cv2.line(img_matches, (int(obj_corners[0,0,0] + img_scene .shape[1]), int(obj_corners[0,0,1])),\
                    (int(obj_corners[1,0,0] + img_scene .shape[1]), int(obj_corners[1,0,1])), (0,255,0), 4)
                cv2.line(img_matches, (int(obj_corners[1,0,0] + img_scene .shape[1]), int(obj_corners[1,0,1])),\
                    (int(obj_corners[2,0,0] + img_scene .shape[1]), int(obj_corners[2,0,1])), (0,0,255), 4)
                cv2.line(img_matches, (int(obj_corners[2,0,0] + img_scene .shape[1]), int(obj_corners[2,0,1])),\
                    (int(obj_corners[3,0,0] + img_scene .shape[1]), int(obj_corners[3,0,1])), (0,255,0), 4)
                cv2.line(img_matches, (int(obj_corners[3,0,0] + img_scene .shape[1]), int(obj_corners[3,0,1])),\
                    (int(obj_corners[0,0,0] + img_scene .shape[1]), int(obj_corners[0,0,1])), (255,0,0), 4)
                
                if debug == 2 or scene_area > 150000:
                    #-- Show detected matches
                    cv2.imshow('Good Matches & Object detection', img_matches)
                    cv2.waitKey()


        else:
            scene_area = 0
            score = 0

        areaList.append(scene_area)
        scoreList.append(score)
        ####

    # except:
    #     pass
    print("matchList... ", matchList)
    if len(matchList) != 0:
        if max(matchList) > thresh:
            # finalVal = matchList.index(max(matchList))
            # finalVal = areaList.index(max(areaList))
            finalVal = scoreList.index(max(scoreList))
            # print("max(matchList) = ", max(matchList))
            # print("max(areaList) = ", max(areaList), "finalVal = ", finalVal)
            print("max(scoreList) = ", max(scoreList), "finalVal = ", finalVal)
        
    return finalVal, max(scoreList)


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

    id, finalScore = findID(img2, kpList, desList, images)
    if id != -1:
        cv2.putText(imgOriginal,classnames[id], (50, 50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),3)
        cv2.putText(imgOriginal,str(finalScore), (50, 85),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),3)

    cv2.imshow('Output',imgOriginal)
    if debug > 0:
        cv2.waitKey()
    else:
        # cv2.waitKey(30)
        cv2.waitKey(2000)
    fps = 1 / (time.time() - tic)
    print("fps = ", fps, "id = ", id, "classnames = ", classnames[id])

