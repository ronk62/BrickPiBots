#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 7/18/2021     Ron King    - created this to print live text data from the Intel RealSense T265
#                           tracking camera to terminal for accuracy and repeatability testing
#

'''
Example code for relocalization notifications... 
...from https://github.com/IntelRealSense/librealsense/issues/5843


def callback_function(notif):
    if notif.get_category() is rs.notification_category.pose_relocalization:
        print("Relocalization has happened!")

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
device = cfg.resolve(pipe).get_device()
pose_sensor = device.first_pose_sensor()
pose_sensor.set_notifications_callback(callback_function)
pipe.start(cfg)

'''


import time, datetime, tty, sys, threading
from matplotlib.colors import to_rgb
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import math as m
from numpy.lib.function_base import blackman

# import the pyrealsense library
import pyrealsense2.pyrealsense2 as rs  # req'd for RPi3b and compiled installation


# relocalization notifications
def callback_function(notif):
    print("callback function invoked", datetime())
    # if notif.get_category() is rs.notification_category.pose_relocalization:
    if notif.get_category() == rs.notification_category.pose_relocalization:
        print("Relocalization has happened!")

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# additions along with relocalization notifications
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

'''FAIL
### use as needed for notification info discovery
print("rs.notification.category  ", pose_sensor.notification.get_category(rs.notification))
'''

# relocalization notifications callback code
pose_sensor.set_notifications_callback(callback_function)


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
                data = pose.get_pose_data()
                # # Print some of the pose data to the terminal
                # print("Frame #{}".format(pose.frame_number))
                # print("Position: {}".format(data.translation))
                # print("Velocity: {}".format(data.velocity))
                # print("Acceleration: {}\n".format(data.acceleration))
                
                ### confidence info
                ''' ignore mapperConfidence - it is never not 0 (zero)
                # mapperConfidence = data.mapper_confidence
                # print("pose mapperConfidence is ", mapperConfidence)
                '''
                trackerConfidence = data.tracker_confidence
                # print("pose trackerConfidence is ", trackerConfidence)

                xT = data.translation.x
                zT = data.translation.z
                # print(xTrans)
                # print(zTrans)
                # time.sleep(5)
                
                # Euler angles from pose quaternion
                # See also https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-549795232
                # and https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-550217609

                w = data.rotation.w
                x = -data.rotation.z
                y = data.rotation.x
                z = -data.rotation.y

                # pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
                # roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
                yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
                
                # print("Frame #{}".format(pose.frame_number))
                # print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))

                # print formatted columns of data for easy consumption
                if i == 0:
                    print("xT    |  zT    |  yaw    |  trackerConfidence")
                    print("------   ------   -------   ----------")
                print(round(xT,2), "    ", round(zT,2),  "    ", round(yaw,2),  "    ", trackerConfidence)

                time.sleep(0.5)

               
        except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
            #finally:
            pipe.stop()

