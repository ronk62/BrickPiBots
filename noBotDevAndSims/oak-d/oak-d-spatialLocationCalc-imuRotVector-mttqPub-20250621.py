#!/usr/bin/env python3

# Date          Author      Change Notes
# 6/21/2025     Ron King    - from files 'spatial_location_calculator-noGUI.py' and 'oak-d-imuRotVector-mttqPub-20240418'
#                           - merged the two into one as a pre-step to merging in a third file,
#                             'oak-d-spatial_mobilenet-mttqPub-20240504.py', which should supply the required data to
#                             capture the shape of a room (within limits)
#                           - on 6/21/2025, testing showed problems with keyboard inputs; changing to
#                             'imuQueue.tryGet()  # non-blocking call' didn't help, so pick-up t/shooting here
#                           - also, I did not complete the 'publish spatialLocationCalcData data over mttq' section; the
#                             data needs to be assigned to variables (prior to the print statements) and the function call
#                             needs those variables substituted for the hard-coded dummy values I placed there temporarily
# 6/22/2025                 - fixed the keyboard control entry bug; this was caused by a redundant use of var name 'x'
#                           - also renamed var, 'yawCCWincr' to 'yawCWincr' (the intent was to indicate clockwise, not counterCW)


import cv2
import depthai as dai
import time, tty, sys, threading
import math
import numpy as np

import oak_d_mttqPub_spatialLocationCalc_imuRotVector_20250621 as mttqPub

# keyboard control setup and vars
tty.setcbreak(sys.stdin)
kbx = 0   # set here to make global

def keyboardInput(name):
    while (True):
        global kbx  # Declare kbx as global to force use of global 'kbx' in this function/thread
        kbx = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")

stepSize = 0.05

newConfig = False

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs for spatialLocationCalculator
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

lrcheck = False
subpixel = False

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(lrcheck)
stereo.setSubpixel(subpixel)

# Config
topLeft = dai.Point2f(0.4, 0.4)
bottomRight = dai.Point2f(0.6, 0.6)

config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 100
config.depthThresholds.upperThreshold = 10000
config.roi = dai.Rect(topLeft, bottomRight)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)
spatialLocationCalculator.initialConfig.addROI(config)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)


# Define sources and outputs for IMU Sensor (imuRotVector)
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
imu.setMaxBatchReports(10)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)


# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    color = (255, 255, 255)

    print("Use Arrow keys to move ROI!")
    
    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None

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
            a1*d2 + b1*c2 - c1*b2 + d1*a2])


    # more ref (for testing) 
    # https://www.wolframalpha.com/input?i=quaternion+-Sin%5BPi%5D%2B3i%2B4j%2B3k+multiplied+by+-1j%2B3.9i%2B4-3k


    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000


    while True:
        
        #kbx = 65 # testing, comment or delete later!

        # Get Spatial Results
        inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

        depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        spatialData = spatialCalcQueue.get().getSpatialLocations()
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            depthMin = depthData.depthMin
            depthMax = depthData.depthMax

            fontType = cv2.FONT_HERSHEY_TRIPLEX
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
            cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, 255)
            print("spatialLocationCalc X: ", int(depthData.spatialCoordinates.x), "mm")
            spatialLocationCalcX = int(depthData.spatialCoordinates.x)
            cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, 255)
            print("spatialLocationCalc Y: ", int(depthData.spatialCoordinates.y), "mm")
            spatialLocationCalcY = int(depthData.spatialCoordinates.y)
            cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, 255)
            print("spatialLocationCalc Z: ", int(depthData.spatialCoordinates.z), "mm")
            spatialLocationCalcZ = int(depthData.spatialCoordinates.z)
            print()
            print()

        # publish spatialLocationCalcData data over mttq
        mttqPub.publish_spatialLocationCalc_data(spatialLocationCalcX, spatialLocationCalcY, spatialLocationCalcZ)

        # Show the frame
        #cv2.imshow("depth", depthFrameColor)

        # Keyboard, ROI adjustment, etc
        if kbx == 65: # up-arrow key pushed
            if topLeft.y - stepSize >= 0:
                topLeft.y -= stepSize
                bottomRight.y -= stepSize
                newConfig = True

        elif kbx == 68: # left-arrow key pushed key pushed
            if topLeft.x - stepSize >= 0:
                topLeft.x -= stepSize
                bottomRight.x -= stepSize
                newConfig = True

        elif kbx == 66: # down-arrow key pushed
            if bottomRight.y + stepSize <= 1:
                topLeft.y += stepSize
                bottomRight.y += stepSize
                newConfig = True

        elif kbx == 67: # right-arrow key pushed key pushed
            if bottomRight.x + stepSize <= 1:
                topLeft.x += stepSize
                bottomRight.x += stepSize
                newConfig = True

        kbx = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key

        if newConfig:
            print("topLeft.x", topLeft.x, "topLeft.y", topLeft.y, "bottomRight.x", bottomRight.x, "bottomRight.y", bottomRight.y)
            config.roi = dai.Rect(topLeft, bottomRight)
            config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.AVERAGE
            cfg = dai.SpatialLocationCalculatorConfig()
            cfg.addROI(config)
            spatialCalcConfigInQueue.send(cfg)
            newConfig = False



        # Get IMU Results
        #imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
        # ----
        '''
        # --- optional approach if blocking method causes issues (per chatGPT) ---
        '''
        imuData = imuQueue.tryGet()  # non-blocking call
        if imuData:
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
                # 7/4/2022 note: pitch and roll were transposed; further testing confirmed this
                # pitch_y, yaw_z, roll_x = euler_from_quaternion(cw, cx, cy, cz)  # orig order, seems wrong
                roll_x, yaw_z, pitch_y = euler_from_quaternion(cw, cx, cy, cz)  # swapped pitch and roll
                print("RPY [0 +/-180 deg]: roll_x: {0:.7f}, pitch_y: {1:.7f}, yaw_z: {2:.7f}".format(roll_x, pitch_y, yaw_z))

                # remove negative (0 +/- 180) angles
                if roll_x < 0:
                    roll_x = 360 + roll_x
                if pitch_y < 0:
                    pitch_y = 360 + pitch_y
                if yaw_z < 0:
                    yaw_z = 360 + yaw_z
                print("RPY [0-359 deg]: roll_x: {0:.7f}, pitch_y: {1:.7f}, yaw_z: {2:.7f}".format(roll_x, pitch_y, yaw_z))

                # show yaw(z) in std compass CW increasing form
                yawCWincr = 360 - yaw_z
                print("RPY [yawCWincr deg]: roll_x: {0:.7f}, pitch_y: {1:.7f}, yawCWincr: {2:.7f}".format(roll_x, pitch_y, yawCWincr))

                # publish imuData over mttq
                mttqPub.publish_imuRotVector_data(roll_x, pitch_y, yaw_z)

                ### conv to matrix form
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
        
        time.sleep(0.5)
