#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")

# enable at 500 hz rate
# imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW], 200)
# imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER], 200)
imu.enableIMUSensor([dai.IMUSensor.GRAVITY], 200)

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

    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None
    while True:
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter

            acceleroTs = acceleroValues.timestamp.get()
            if baseTs is None:
                baseTs = acceleroTs
            acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            total = math.sqrt(math.pow(acceleroValues.x, 2) + math.pow(acceleroValues.y, 2) + math.pow(acceleroValues.z, 2))
            print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
            print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
            print(f"Total: {total}")

        if cv2.waitKey(1) == ord('q'):
            break