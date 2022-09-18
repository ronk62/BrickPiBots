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

### use as needed for option discovery
print("supported_options  ", pose_sensor.get_supported_options())
print("option option.enable_mapping  ", pose_sensor.get_option(rs.option.enable_mapping))
print("option option.enable_relocalization  ", pose_sensor.get_option(rs.option.enable_relocalization))
print("option option.option.enable_pose_jumping  ", pose_sensor.get_option(rs.option.enable_pose_jumping))
print("option option.enable_map_preservation  ", pose_sensor.get_option(rs.option.enable_map_preservation))

# enable_map_preservation 
pose_sensor.set_option(rs.option.enable_map_preservation, 1)
print("option option.enable_map_preservation  ", pose_sensor.get_option(rs.option.enable_map_preservation))

# load lmap from csv file
print ("loading exported localization map from csv file")
lmap = np.loadtxt('t265localiztionmap-1.csv', delimiter=',')
print ("load commplete")
print("lmap = ", lmap)
print("size lmap = ", np.size(lmap))
print("")

# convert lmap from np.array float to int
lmap = lmap.astype(int)

# # convert lmap from np.array to python list  ### not needed due to 'lmap = lmap.astype(int)'
# lmap = np.ndarray.tolist(lmap)

# import localization map
pose_sensor.import_localization_map(lmap)
print("")


# get static node
result, translation, rotation = pose_sensor.get_static_node("name")
print("result = ",result)
print("translation = ",translation)
print("rotation = ",rotation)
print("")

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

    # get static node
    print("--------------------------------------------------------")
    result, translation, rotation = pose_sensor.get_static_node("name")
    print("result = ",result)
    print("translation = ",translation)
    print("rotation = ",rotation)
    print("")
        

finally:
    pipe.stop()

