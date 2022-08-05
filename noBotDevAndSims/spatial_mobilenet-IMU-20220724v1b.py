#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 7/24/2022     Ron King    - merged OAK-D IMU Rotation Vector code into spacial_mobilenet code
#
# 8/4/2022                  - updated to show live sensor data plots (matplotlib + animate)


#                           - DON'T Forget to start xming and export DISPLAY=10.0.0.9:0.0  (change IP addr as req'd)


from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import math
# import matplotlib.pyplot as plt
import matplotlib.animation as animation
# from matplotlib import style


'''
Spatial detection adjusted by IMU rotation vector, normalized to North and gravity

Adapted from Spatial detection network demo.
    Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
'''

# flag to enable/disable show images
showImages = False

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('../models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnBlobPath = sys.argv[1]

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

## Define Cameras sources and outputs, etc
# sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)


## Define IMU sources and outputs, etc
# sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")

# enable ROTATION_VECTOR at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(1)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)


# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # init some vars
    roll_xMean = 0
    yaw_zMean = 0
    pitch_yMean = 0
    roll_xStdev = 0
    yaw_zStdev = 0
    pitch_yStdev = 0

    ### Quaternion to Euler angles conversion (alternative conv methods below)
    ## optional pre-fab code from https://discuss.luxonis.com/d/397-oak-d-bno085-imu-coordinate-frame/4
    def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.degrees(math.atan2(t0, t1))
    

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.degrees(math.asin(t2))
    

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.degrees(math.atan2(t3, t4))
    
        return pitch_y, yaw_z, roll_x


    # basic rotations (Euler to rotation matrix)
    def Rx(theta):
        return np.matrix([[ 1, 0           , 0           ],
                        [ 0, math.cos(theta),-math.sin(theta)],
                        [ 0, math.sin(theta), math.cos(theta)]])
  
    def Ry(theta):
        return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                        [ 0           , 1, 0           ],
                        [-math.sin(theta), 0, math.cos(theta)]])
    
    def Rz(theta):
        return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                        [ math.sin(theta), math.cos(theta) , 0 ],
                        [ 0           , 0            , 1 ]])


    # ref https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
    def quaternion_multiply(quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
    

    #ref https://github.com/KodyJKing/quaternion_to_euler/blob/main/main.py
    def quat_multiply(q1, q2):
        a1, b1, c1, d1 = q1
        a2, b2, c2, d2 = q2
        return np.array([
            a1*a2 - b1*b2 - c1*c2 - d1*d2,
            a1*b2 + b1*a2 + c1*d2 - d1*c2,
            a1*c2 - b1*d2 + c1*a2 + d1*b2,
            a1*d2 + b1*c2 - c1*b2 + d1*a2
    ])

    # more ref (for testing) 
    # https://www.wolframalpha.com/input?i=quaternion+-Sin%5BPi%5D%2B3i%2B4j%2B3k+multiplied+by+-1j%2B3.9i%2B4-3k

    
    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000


    def animate(i):
        # Declare vars as global to force use of global in this function
        global roll_x, yaw_z, pitch_y
        # global roll_xMean, yaw_zMean, pitch_yMean
        # global roll_xStdev, yaw_zStdev, pitch_yStdev

        # # init the data collection arrays for each collection series
        # roll_xArr = np.array([], dtype=np.int32)
        # yaw_zArr = np.array([], dtype=np.int32)
        # pitch_yArr = np.array([], dtype=np.int32)


        ## Cam main section
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = inPreview.getCvFrame()

        depthFrame = depth.getFrame() # depthFrame values are in millimeters

        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        detections = inDet.detections
        if len(detections) != 0:
            boundingBoxMapping = xoutBoundingBoxDepthMapping.get()
            roiDatas = boundingBoxMapping.getConfigData()

            for roiData in roiDatas:
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), 255, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

            ## testing - print detections data
            #print("detections data...  ", detections)
            for detection in detections:
                label = labelMap[detection.label]
                print("detection label, labelnum, x, y, z, confidence ...  ", label, detection.label, (int(detection.spatialCoordinates.x)), (int(detection.spatialCoordinates.y)), (int(detection.spatialCoordinates.z)), detection.confidence * 100)
        temperature = device.getChipTemperature().average
        print("avg temp:  ", temperature)


        ## IMU section
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            rVvalues = imuPacket.rotationVector

            rvTs = rVvalues.timestamp.get()
            if baseTs is None:
                baseTs = rvTs
            rvTs = rvTs - baseTs

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            print(f"Rotation vector timestamp: {tsF.format(timeDeltaToMilliS(rvTs))} ms")
            print(f"Quaternion: i: {imuF.format(rVvalues.i)} j: {imuF.format(rVvalues.j)} "
                f"k: {imuF.format(rVvalues.k)} real: {imuF.format(rVvalues.real)}")
            print(f"Accuracy (rad): {imuF.format(rVvalues.rotationVectorAccuracy)}")


            ### Quaternions to Euler

            # orig assignments
            w = rVvalues.real
            x = rVvalues.i
            y = rVvalues.j
            z = rVvalues.k

            # represent quaternions as vectors
            qV0 = np.array([w, x, y, z], dtype=np.float16)
            qV1 = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float16)
            
            ## quaternion multiplication - rotation to adjust for sensor/cam orientation
            # # alt 1 (Ron)
            # qV2 = quaternion_multiply(qV0, qV1)

            # alt 2 (Kody)
            qV2 = quat_multiply(qV0, qV1)

            # extract w, z, y, z as converted (cw, cx, cy, cz)
            cw, cx, cy, cz = qV2
            # testing
            print("cw, cx, cy, cz = ", cw, cx, cy, cz)
            print("")
            
            # # conv to uler_from_quaternion using function
            # pitch_y, yaw_z, roll_x = euler_from_quaternion(w, x, y, z)

            # conv to uler_from_quaternion using function
            # 7/4/2022 note: pitch and roll seem to be transposed; further testing needed
            # pitch_y, yaw_z, roll_x = euler_from_quaternion(cw, cx, cy, cz)  # orig order, seems wrong
            roll_x, yaw_z, pitch_y = euler_from_quaternion(cw, cx, cy, cz)  # swapped pitch and roll

            print("RPY [deg]: roll_x: {0:.7f}, pitch_y: {1:.7f}, yaw_z: {2:.7f}".format(roll_x, pitch_y, yaw_z))

            ### conv to matrix form and 
            # remove negative (0 +/- 180) angles
            if roll_x < 0:
                roll_x = 360 + roll_x
            if pitch_y < 0:
                pitch_y = 360 + pitch_y
            if yaw_z < 0:
                yaw_z = 360 + yaw_z
            
            # create idetntity matrix
            R_I = np.eye(3)
            
            ## rotate around x by (minus) -roll_x # suspected problems negated roll_x
            # convert to radians
            theta1Rad=math.radians(-roll_x)
            # ## rotate around x by roll_x # removed negation
            # # convert to radians
            # theta1Rad=math.radians(roll_x)
            Rq_01 = Rx(theta1Rad)
            Rq_02 = np.dot(R_I, Rq_01)
            # print()
            # print("Rq_02 is  ")
            # print(np.matrix(Rq_02))
            # print()
            
            ## rotate around y by pitch_y
            # convert to radians
            theta1Rad=math.radians(pitch_y)
            Rq_03 = Ry(theta1Rad)
            Rq_04 = np.dot(Rq_02, Rq_03)
            # print()
            # print("Rq_04 is  ")
            # print(np.matrix(Rq_04))
            # print()
            
            ## rotate around z by yaw_z
            # convert to radians
            theta1Rad=math.radians(yaw_z)
            Rq_05 = Rz(theta1Rad)
            Rq_06 = np.dot(Rq_04, Rq_05)
            print()
            print("Rq_06 is  ")
            print(np.matrix(Rq_06))
            print()

            # time.sleep(0.05)


        ## Cam optional section
        # If show showImages True, proceed
        if showImages == True:
            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]
            for detection in detections:
                # Denormalize bounding box
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                try:
                    label = labelMap[detection.label]
                except:
                    label = detection.label
                cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)

            cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
            cv2.imshow("depth", depthFrameColor)
            cv2.imshow("preview", frame)

            if cv2.waitKey(1) == ord('q'):
                exit


        ## setup vars to graph the unit vectors
        # extract column vectors from Rq_06 matrix
        x0 = 0
        y0 = 0
        vxx0 = [x0,x0 + Rq_06[0][0]]
        vxz0 = [y0,y0 + Rq_06[2][0]]
        vzx0 = [x0,x0 + Rq_06[0][2]]
        vzz0 = [y0,y0 + Rq_06[2][2]]

        vyx0 = [x0,x0 + Rq_06[0][1]]
        vyy0 = [y0,y0 + Rq_06[1][1]]        
        
        # graph the IMU Rotation unit vectors
        fig, ax = plt.subplots()

        plt.scatter(x0,y0, label='cam/bot (v0) origin', color='r', s=25, marker="o")
        line1, = ax.plot(vxx0,vxz0, label='v0 x', lw=0.4, color='r', marker=">")
        line2, = ax.plot(vyx0,vyy0, label='v0 z (bot forward/out from cam lens)', lw=0.4, color='b', marker="^")

        # plt.scatter(x0,y0, label='cam/bot (v0) origin', color='m', s=25, marker="o")
        # line3, = ax.plot(vxx0,vxz0, label='v0 x', lw=0.4, color='m', marker=">")
        # line4, = ax.plot(vyx0,vyy0, label='v0 y (vertical)', lw=0.4, color='g', marker="^")

        plt.axis('equal')
        plt.xlabel('x-position')
        plt.ylabel('Z-position')
        plt.title('unit vectors')
        plt.legend()

        time.sleep(0.001)

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        xoutBoundingBoxDepthMapping = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        startTime = time.monotonic()
        counter = 0
        fps = 0

        # Output queue for imu bulk packets
        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        baseTs = None


        while True:
            try:
                    ani = animation.FuncAnimation(fig, animate, interval=100)
                    plt.show()        
                    
            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                exit