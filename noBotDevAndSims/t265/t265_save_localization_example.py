#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import time

lmap = np.array([], dtype=np.int32) # arrary to hold localization map for import/export

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# setup device and pose_sensor
device = cfg.resolve(pipe).get_device()
pose_sensor = device.first_pose_sensor()

# export localization map
lmap = pose_sensor.export_localization_map()
print("lmap = ", lmap)
print("")

# save lmap to csv file
print ("saving exported localization map to csv file")
np.savetxt('t265localiztionmap-1.csv', lmap, delimiter=',')
print ("save commplete")
print("")

# # get static node
# result, translation, rotation = pose_sensor.get_static_node("name")
# print("result = ",result)
# print("translation = ",translation)
# print("rotation = ",rotation)
# print("")

# Start streaming with requested config
pipe.start(cfg)

try:
    for _ in range(50):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            print("Frame #{}".format(pose.frame_number))
            print("Position: {}".format(data.translation))
            print("Velocity: {}".format(data.velocity))
            print("Acceleration: {}\n".format(data.acceleration))

        time.sleep(1)

finally:
    pipe.stop()

