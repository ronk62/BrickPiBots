#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 6/18/2021     Ron King    - created this to visualize live data from the Intel
#                           - RealSense T265 tracking camera
#

import time, tty, sys, threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import math as m

# import the pyrealsense library
import pyrealsense2 as rs

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

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
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()
        print("Frame #{}".format(pose.frame_number))
        print("Position: {}".format(data.translation))
        print("Velocity: {}".format(data.velocity))
        print("Acceleration: {}\n".format(data.acceleration))

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
        
        print("Frame #{}".format(pose.frame_number))
        print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))

        yawRadZ = m.radians(yaw + 90)  # translate to radians and add 90 deg to show 'Z' ref
        yawVX = xT + (0.5 * m.cos( yawRadZ))
        yawVZ = zT + (0.5 * m.sin( yawRadZ))
        print(yawVX, yawVZ)
        time.sleep(1)

    i = i + 1

    #append the arrays
    ODt = np.append(ODt, i)
    xTrans = np.append(xTrans, xT)
    zTrans = np.append(zTrans, zT)
    rotYaw = np.append(rotYaw, yaw)

    ## graph the position and rotational unit vectors
    ax1.clear()
    ax1.plot(xTrans, zTrans, label='xzPosition', color='r', marker="o")
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

