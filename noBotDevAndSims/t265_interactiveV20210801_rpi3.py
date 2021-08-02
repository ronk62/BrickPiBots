#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

#
# Date          Author      Change Notes
# 8/1/2021      Ron King    - adapted from t265_set_get_static_node_rpi3.py
#                           - added interactive controls for saving/loading maps, starting/stopping the
#                             pipeline, setting static node 1
#
#                           - below is a view of the control keys

'''

save map = 'm'
load map = 'l'
set static node 1 = '1'
start/stop pipeline (toggle) = 'i'

'''


# First import the library
import pyrealsense2.pyrealsense2 as rs  # req'd for RPi3b and compiled installation
import time, sys, os, tty, threading
import math as m
import numpy as np

# vars
sample_mode = -1

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

# keyboard control setup and vars
tty.setcbreak(sys.stdin)
kx = 0   # set here to make global

def keyboardInput(name):
    while (True):
        global kx  # Declare kx as global to force use of global 'kx' in this function/thread
        kx = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")


### Main Loop

while(True):

    if kx == 105: # i pushed - toggle sample mode on and off
        sample_mode *= -1
        kx = 0 # debounce
        print("toggled sample_mode (i); now set to...  ", sample_mode)

    if kx == 108: # l (lower case L) pushed - load map
        print("loading saved map...  ")
        # load lmap from csv file
        print ("loading exported localization map from csv file")
        lmap = np.loadtxt('t265_dining_room_map1.csv', delimiter=',')
        
        # convert lmap from np.array float to int
        lmap = lmap.astype(int)
        
        # import localization map
        pose_sensor.import_localization_map(lmap)
        print("")
        print ("load commplete")
        print("lmap = ", lmap)
        print("size lmap = ", np.size(lmap))
        print("")


    if kx == 109: # m pushed - save map
        # export localization map
        lmap = pose_sensor.export_localization_map()
        print("lmap = ", lmap)
        print("")

        # save lmap to csv file
        print ("saving exported localization map to csv file")
        np.savetxt('t265_dining_room_map1.csv', lmap, delimiter=',')
        print ("save commplete")
        print("")

    if kx == 114: # r pushed - reset hardware
        print("resetting t265 hardware device...  ")
        device.hardware_reset()
        time.sleep(3)

        # # Declare RealSense pipeline, encapsulating the actual device and sensors
        # pipe = rs.pipeline()

        # # Build config object and request pose data
        # cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        device = cfg.resolve(pipe).get_device()
        pose_sensor = device.first_pose_sensor()

        # enable_map_preservation 
        pose_sensor.set_option(rs.option.enable_map_preservation, 1)

        print ("t265 hardware device is reset")


    if(sample_mode > 0): # Start streaming with requested config
        pipe.start(cfg)
        print("pipeline started...  ")


    kx = 0 # debounce
    
    while(sample_mode > 0):  ## pipeline active, start pipe and do active pipe stuff

        for i in range(13):

            try:   # read the sensor - we really only need x, z, and yaw for this
                
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

                if kx == 49: # 1 (one) pushed - set static node 1, if trackerConfidence is high
                    if trackerConfidence == 3:
                        print("")
                        print("setting static node 'snode1'...")
                        print("")
                        pose_sensor.set_static_node("snode1", data.translation, data.rotation)

                if kx == 105: # i pushed - toggle sample mode on and off
                    sample_mode *= -1
                    pipe.stop()
                    print("pipeline stopped; toggled sample_mode (i); now set to...  ", sample_mode)
                    break

                if kx == 109: # m pushed - save map
                    # export localization map
                    lmap = pose_sensor.export_localization_map()
                    print("lmap = ", lmap)
                    print("")

                    # save lmap to csv file
                    print ("saving exported localization map to csv file")
                    np.savetxt('t265_dining_room_map1.csv', lmap, delimiter=',')
                    print ("save commplete")
                    print("")

            except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
                pipe.stop()

            kx = 0 # debounce
            time.sleep(0.5)

    
    kx = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
    time.sleep(0.2)

