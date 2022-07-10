#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math
import numpy as np

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
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

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:

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

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None
    while True:
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

            time.sleep(0.5)


        if cv2.waitKey(1) == ord('q'):
            break
