#!/usr/bin/env python3

# raw/unchanged from koonpl, ref. https://discuss.luxonis.com/d/397-oak-d-bno085-imu-coordinate-frame/4
# with the exception of possible commented lines near 96-99

import cv2
import depthai as dai
import time
import math

import numpy as np

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

def quaternion_to_euler_angle(qr, qi, qj, qk):
    jsqr = qj * qj
    isqr = qi * qi
    ksqr = qk * qk
    rsqr = qr * qr

    # Yaw  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));    t0 = 2.0 * (qi * qj + qk * qr)
    t0 = 2.0 * (qi * qj + qk * qr)
    t1 = isqr - jsqr - ksqr + rsqr
    yaw = np.degrees(np.arctan2(t0, t1))

    # Pitch  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    t2 = -2.0 * (qi * qk - qj * qr)/(isqr + jsqr + ksqr + rsqr)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.degrees(np.arcsin(t2))  
    
    # Z  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));   
    t3 = +2.0 * (qj * qk + qi * qr)
    t4 = 1 - 2 * (-isqr -jsqr + ksqr + rsqr)
    #Roll
    roll = np.degrees(np.arctan2(t3, t4))

    return pitch, yaw, roll 

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
imuOut = pipeline.create(dai.node.XLinkOut)

imuOut.setStreamName("imu")

# enable ROTATION_VECTOR at 400 hz rate ARVR_STABILIZED_GAME_ROTATION_VECTOR
imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 10)
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

# Link plugins IMU -> XLINK
imu.out.link(imuOut.input)

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:

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
            pitch, yaw, roll = euler_from_quaternion(
                rVvalues.real, rVvalues.i, rVvalues.j, rVvalues.k)
            # pitch, yaw, roll = quaternion_to_euler_angle(
            #     rVvalues.real, rVvalues.i, rVvalues.j, rVvalues.k)

            rvTs = rVvalues.timestamp.get()
            if baseTs is None:
                baseTs = rvTs
            rvTs = rvTs - baseTs

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            # print(f"Rotation vector timestamp: {tsF.format(timeDeltaToMilliS(rvTs))} ms")
            # print(f"Quaternion: i: {imuF.format(rVvalues.i)} j: {imuF.format(rVvalues.j)} "
            #     f"k: {imuF.format(rVvalues.k)} real: {imuF.format(rVvalues.real)}")
            print(f"Euler: pitch: {imuF.format(pitch)} yaw: {imuF.format(yaw)} "
                f"roll: {imuF.format(roll)} Accuracy (rad): {imuF.format(rVvalues.rotationVectorAccuracy)}")
            # print(f"Accuracy (rad): {imuF.format(rVvalues.rotationVectorAccuracy)}")
        if cv2.waitKey(1) == ord('q'):
            break