#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 6/18/2021     Ron King    - created this to visualize live data from the Intel
#                           - RealSense T265 tracking camera
# 7/3/2021      Rong King   - added relocalization notifications
#                           - added code to help discover and then set pose_sensor options
#                           - extracted confidence values and used to color-code plt
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
import pyrealsense2 as rs

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

# Sensor data vars
ODt = np.array([], dtype=np.float32)        # array to index each time/tick (x axis independant var)
xTrans = np.array([], dtype=np.float32)     # array to hold translational 'x' value
zTrans = np.array([], dtype=np.float32)     # array to hold translational 'z' value
rotYaw = np.array([], dtype=np.float32)     # array to hold rotational 'yaw' value

# setup for live graphing
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)


def animate(i):
    # Declare ODt and others as global to force use of global in this function
    global ODt, xTrans, zTrans, rotYaw
    
    # read the sensor - we really only need x, z, and yaw for this demo script
    
    # Wait for the next set of frames from the camera
    frames = pipe.wait_for_frames()

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
        print("pose trackerConfidence is ", trackerConfidence)

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

        pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
        roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
        yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
        
        # print("Frame #{}".format(pose.frame_number))
        # print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))

        yawRadZ = m.radians(yaw + 90)  # translate to radians and add 90 deg to show 'Z' ref
        yawVX = xT + (0.5 * m.cos( yawRadZ))
        yawVZ = zT + (0.5 * m.sin( yawRadZ))
        # print(yawVX, yawVZ)
        time.sleep(1)

    i = i + 1

    #append the arrays
    ODt = np.append(ODt, i)
    xTrans = np.append(xTrans, xT)
    zTrans = np.append(zTrans, zT)
    rotYaw = np.append(rotYaw, yaw)
    # mapperConfidenceColorA = np.append(mapperConfidenceColorA, mapperConfidenceColor)

    ## graph the position and rotational unit vectors
    ax1.clear()
    # ax1.plot(xTrans, zTrans, label='xzPosition', color='mapperConfidenceColorA', marker="o")
    if trackerConfidence == 0:
        ax1.plot(xTrans, zTrans, label='xzPosition', color='k', marker="o")
    if trackerConfidence == 1:
        ax1.plot(xTrans, zTrans, label='xzPosition', color='r', marker="o")
    if trackerConfidence == 2:
        ax1.plot(xTrans, zTrans, label='xzPosition', color='y', marker="o")
    if trackerConfidence == 3:
        ax1.plot(xTrans, zTrans, label='xzPosition', color='g', marker="o")
    ax1.plot([xT, yawVX], [zT, yawVZ], label='yawVZ', lw=0.8, color='g', marker="None")
    ax1.legend()
    plt.axis('equal')

                
    if ODt.size > 100000:
        print("")
        print("")
        print ("Exiting sample mode due to sample size too large...")
        print ("ODt.size = ", ODt.size)
        print("")
        exit


### Main animate Loop

try:
    ani = animation.FuncAnimation(fig, animate, interval=100, repeat=False)
    plt.show()
                
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    #finally:
    pipe.stop()

