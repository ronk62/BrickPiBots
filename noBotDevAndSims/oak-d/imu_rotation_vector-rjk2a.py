#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math
import numpy as np

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
                        [ 0, m.cos(theta),-m.sin(theta)],
                        [ 0, m.sin(theta), m.cos(theta)]])
  
    def Ry(theta):
        return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                        [ 0           , 1, 0           ],
                        [-m.sin(theta), 0, m.cos(theta)]])
    
    def Rz(theta):
        return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                        [ m.sin(theta), m.cos(theta) , 0 ],
                        [ 0           , 0            , 1 ]])


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

            # w = {imuF.format(rVvalues.real)}
            # x = {imuF.format(rVvalues.i)}
            # y = {imuF.format(rVvalues.j)}
            # z = {imuF.format(rVvalues.k)}

            w = rVvalues.real
            x = rVvalues.i
            y = rVvalues.j
            z = rVvalues.k

            # Quaternion to Euler angles conversion

            ## orig math from realsense/t265 code (failed here)
            # pitch =  -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi
            # roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi
            # yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi
            
            # print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))

            ## New math from wikipedia, https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
            
            '''
            code below written in C
            
            // roll (x-axis rotation)
            double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (q.w * q.y - q.z * q.x);
            if (std::abs(sinp) >= 1)
                angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                angles.pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);
            
            '''

            # conv using function
            pitch_y, yaw_z, roll_x = euler_from_quaternion(w, x, y, z)

            print("RPY [deg]: roll_x: {0:.7f}, pitch_y: {1:.7f}, yaw_z: {2:.7f}".format(pitch_y, yaw_z, roll_x))

            # conv to matrix form


            time.sleep(0.5)


        if cv2.waitKey(1) == ord('q'):
            break
