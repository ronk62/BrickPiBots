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

# enable ACCELEROMETER_RAW at 500 hz rate
# imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
# # enable GYROSCOPE_RAW at 400 hz rate
# imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
# enable MAGNETOMETER_xxxx at 50 hz rate (recommended value for calibration by HillCrest Labs)
# _RAW
# imu.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_RAW, 50)
# magUnits = "uTesla"
# _UNCALIBRATED
# imu.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_UNCALIBRATED, 50)
# magUnits = "uTesla"
# _CALIBRATED
imu.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_CALIBRATED, 50)
magUnits = "calibrated"
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
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
            # acceleroValues = imuPacket.acceleroMeter
            # gyroValues = imuPacket.gyroscope
            magmeterValues = imuPacket.magneticField

            # acceleroTs = acceleroValues.timestamp.get()
            # gyroTs = gyroValues.timestamp.get()
            magTs = magmeterValues.timestamp.get()
            # if baseTs is None:
            #     baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
            # acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
            # gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            # print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
            # print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
            # print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
            # print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")

            ##  Note that "relHeading" is +/- 180deg relative to North
             
            # ## sensor/camera facing up (camera on heatsing facing ceiling)
            # ### resulting orientation - usb cable is the "heading"
            # relHeading = math.atan2(magmeterValues.y, magmeterValues.x) * (180 / math.pi)

            ## sensor/camera facing forward (camera mounted in robot)
            ### resulting orientation - camera 'z' is the "heading"
            relHeading = math.atan2(magmeterValues.y, magmeterValues.z) * (180 / math.pi)

            # relHeading = math.degrees(relHeading)  ### not needed, results are already in deg
            
            # calculate compassHeading for user convenience
            if relHeading < 0:
                compassHeading = 360 + relHeading
            else:
                compassHeading = relHeading
            
            print(f"Magnetometer: x: {imuF.format(magmeterValues.x)} y: {imuF.format(magmeterValues.y)} z: {imuF.format(magmeterValues.z)} Accuracy: {imuF.format(magmeterValues.accuracy.value)} ")
            print(f"relHeading: {imuF.format(relHeading)} compassHeading: {imuF.format(compassHeading)} ")



        if cv2.waitKey(1) == ord('q'):
            break
