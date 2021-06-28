#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
#import pyrealsense2 as rs
import pyrealsense2.pyrealsense2 as rs  # req'd for RPi3b and compiled installation

import time

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

try:
    while(1):
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
        time.sleep(0.5)

finally:
    pipe.stop()
