'''
ref:

https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API




https://www.youtube.com/watch?v=BFDkTTeBgCQ&list=WL&index=2&t=17s

https://www.fypsolutions.com/opencv-python/ssdlite-mobilenet-object-detection-with-opencv-dnn/


https://github.com/murtazahassan/OpenCV-Python-Tutorials-and-Projects


https://cocodataset.org/#home


'''


import cv2 as cv

cvNet = cv.dnn.readNetFromTensorflow('/home/ronk/Documents/pythonProjects/noBotDevAndSims/murtaza/cv2_dnn_object_detection/frozen_inference_graph.pb', '/home/ronk/Documents/pythonProjects/noBotDevAndSims/murtaza/cv2_dnn_object_detection/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

img = cv.imread('/home/ronk/Documents/pythonProjects/noBotDevAndSims/murtaza/cv2_dnn_object_detection/example.jpg')
rows = img.shape[0]
cols = img.shape[1]
cvNet.setInput(cv.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))
cvOut = cvNet.forward()

for detection in cvOut[0,0,:,:]:
    score = float(detection[2])
    if score > 0.3:
        left = detection[3] * cols
        top = detection[4] * rows
        right = detection[5] * cols
        bottom = detection[6] * rows
        cv.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (23, 230, 210), thickness=2)

cv.imshow('img', img)
cv.waitKey()