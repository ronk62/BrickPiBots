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
# 7/25/2021     Ron King    - condensed static node to SxT, SzT, Syaw
#                           - print output in nice, formatted columns
#

# First import the library
import pyrealsense2.pyrealsense2 as rs  # req'd for RPi3b and compiled installation
import time
import math as m

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


### Main Loop

while(True):
    
    for i in range(13):

        try:
            # read the sensor - we really only need x, z, and yaw for this
                    
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
                # print("Position is ", data.translation)
                
                ### confidence info
                trackerConfidence = data.tracker_confidence
                # print("pose trackerConfidence is ", trackerConfidence)

                xT = data.translation.x
                zT = data.translation.z

                w = data.rotation.w
                x = -data.rotation.z
                y = data.rotation.x
                z = -data.rotation.y

                # pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
                # roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
                yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;


                # set static node 'snode1' if conditions are met
                if trackerConfidence == 3 and static_node_set == 0 and data.translation.x >= 0.3  and data.translation.z <= -0.3:
                    print("")
                    print("setting static node 'snode1'...")
                    print("")
                    pose_sensor.set_static_node("snode1", data.translation, data.rotation)
                    static_node_set = 1

                # get static node
                result, translation, rotation = pose_sensor.get_static_node("snode1")
                # print("result set from updated get_static_node...")
                # print("result = ",result)
                # print("translation static x, z = ", translation.x, translation.z)
                # print("rotation = ",rotation)
                # print("")

                SxT = translation.x
                SzT = translation.z

                Sw = rotation.w
                Sx = -rotation.z
                Sy = rotation.x
                Sz = -rotation.y

                Syaw   =  m.atan2(2.0 * (Sw*Sz + Sx*Sy), Sw*Sw + Sx*Sx - Sy*Sy - Sz*Sz) * 180.0 / m.pi;



                # print formatted columns of data for easy consumption
                if i == 0:
                    print()
                    print("xT  |  zT  |  yaw  |  Confid  |  S result  |  SxT  |  SzT  |  Syaw")
                    print("----   ----   -----   ------     --------     ------  -----   ----")
                print(round(xT,2), " ", round(zT,2),  " ", round(yaw,2),  "   ", trackerConfidence,  "       ", result,  "     ", round(SxT,2), "   ", round(SzT,2), " ", round(Syaw,2))

            time.sleep(0.5)

        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            pipe.stop()