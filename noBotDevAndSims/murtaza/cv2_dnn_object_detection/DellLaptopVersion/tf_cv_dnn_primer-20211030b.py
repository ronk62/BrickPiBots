'''
ref:

https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API




https://www.youtube.com/watch?v=BFDkTTeBgCQ&list=WL&index=2&t=17s

https://www.fypsolutions.com/opencv-python/ssdlite-mobilenet-object-detection-with-opencv-dnn/


https://github.com/murtazahassan/OpenCV-Python-Tutorials-and-Projects


https://cocodataset.org/#home


'''


import cv2 as cv


def getClassLabel(class_id, classes):
    for key,value in classes.items():
        if class_id == key:
            return value


COCO_labels = { 0: 'background',
    1: '"person"', 2: 'bicycle', 3: 'car', 4: 'motorcycle',
    5: 'airplane', 6: 'bus', 7: 'train', 8: 'truck', 9: 'boat',
    10: 'traffic light', 11: 'fire hydrant', 13: 'stop sign', 14: 'parking meter',
    15: 'zebra', 16: 'bird', 17: 'cat', 18: 'dog',19: 'horse',20: 'sheep',21: 'cow',22: 'elephant',
    23: 'bear', 24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella',
    31: 'handbag', 32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis',
    36: 'snowboard', 37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
    41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle', 
    46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon', 51: 'bowl', 52: 'banana',
    53: 'apple', 54: 'sandwich', 55: 'orange', 56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza',
    60: 'donut', 61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed', 
    67: 'dining table',70: 'toilet', 72: 'tv', 73: 'laptop',
    74: 'mouse', 75: 'remote', 76: 'keyboard', 78: 'microwave', 79: 'oven', 80: 'toaster', 81: 'sink',
    82: 'refrigerator',84: 'book', 85: 'clock', 86: 'vase', 87: 'scissors',
    88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush' }


cvNet = cv.dnn.readNetFromTensorflow('/home/ronk/Documents/pythonProjects/noBotDevAndSims/murtaza/cv2_dnn_object_detection/frozen_inference_graph.pb', '/home/ronk/Documents/pythonProjects/noBotDevAndSims/murtaza/cv2_dnn_object_detection/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

img = cv.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/murtaza/cv2_dnn_object_detection/example.jpg')
rows = img.shape[0]
cols = img.shape[1]
cvNet.setInput(cv.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
cvOut = cvNet.forward()

for detection in cvOut[0,0,:,:]:
    score = float(detection[2])
    if score > 0.3:
        class_id = detection[1]
        class_label = getClassLabel(class_id,COCO_labels)
        left = detection[3] * cols
        top = detection[4] * rows
        right = detection[5] * cols
        bottom = detection[6] * rows
        # print("")
        cv.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (23, 230, 210), thickness=2)
        cv.putText(img,class_label ,(int(left),int(top)+25),cv.FONT_HERSHEY_SIMPLEX,1,(255, 0, 255),3,cv.LINE_AA)
        print(str(str(class_id) + " " + str(detection[2])  + " " + class_label))


cv.imshow('img', img)
cv.waitKey()