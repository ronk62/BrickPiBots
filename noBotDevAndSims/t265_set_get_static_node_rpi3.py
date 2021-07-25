#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

#
# Date          Author      Change Notes
# 7/24/2021     Ron King    - adapted from example for rpi3
#                           - added set/get static node routines and
#                             exception handling for wait_for_frames timeouts
#

# First import the library
import pyrealsense2.pyrealsense2 as rs  # req'd for RPi3b and compiled installation
import time

# vars
static_node_set = 0

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

device = cfg.resolve(pipe).get_device()
pose_sensor = device.first_pose_sensor()

# enable_map_preservation 
pose_sensor.set_option(rs.option.enable_map_preservation, 1)

# result = pose_sensor.remove_static_node("snode1")
# print("result set from remove_static_node...")
# print("result = ",result)
# time.sleep(2)

# Start streaming with requested config
pipe.start(cfg)

try:
    while(True):
        
        try:
            # Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames()

        except RuntimeError:
            print("wait for frames error")

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            # print("Frame #{}".format(pose.frame_number))
            # print("Position: {}".format(data.translation))
            print("Position is ", data.translation)
            # print("Velocity: {}".format(data.velocity))
            # print("Acceleration: {}\n".format(data.acceleration))

            ### confidence info
            trackerConfidence = data.tracker_confidence
            print("pose trackerConfidence is ", trackerConfidence)

            
            # set static node 'snode1' if conditions are met
            if trackerConfidence == 3 and static_node_set == 0 and data.translation.x >= 0.3  and data.translation.z <= -0.3:
                print("")
                print("setting static node 'snode1'...")
                print("")
                # pipe.stop()
                pose_sensor.set_static_node("snode1", data.translation, data.rotation)
                static_node_set = 1
                # pipe.start(cfg)

            # # stop pipe, get static node, restart pipe
            # pipe.stop()
            result, translation, rotation = pose_sensor.get_static_node("snode1")
            print("result set from updated get_static_node...")
            print("result = ",result)
            print("translation = ",translation)
            print("rotation = ",rotation)
            print("")
            # pipe.start(cfg)

        time.sleep(1)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    #finally:
    pipe.stop()

