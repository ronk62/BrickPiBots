# Note  - no she-bang required (or even used) in win-doze scripts

import paho.mqtt.client as mqtt
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time

# init global vars
imuData = [0,0,0]
spatialLocationData = [0,0,0]

# setup for graphing the IMU/CAM data unit vectors
fig1, ax1 = plt.subplots()

# setup for graphing the spacialCalcROIxyz data
fig2, ax2 = plt.subplots()

# This is the Subscriber

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("topic/oakdspatialLocationCalc")
    client.subscribe("topic/oakdIMU")


def on_message(client, userdata, msg):
    global imuData
    global spatialLocationData

    # for testing, comment or delete as desired
    #print(f"Received message from {msg.topic}: {msg.payload}")

    if (msg.payload.decode() == "Q"):
        print(msg.payload.decode())
        print("quiting...")
        client.disconnect()
    elif (msg.topic == "topic/oakdspatialLocationCalc"):
        spatialLocationDataJsn = msg.payload.decode()
        print("spatialLocationDataJsn = ", spatialLocationDataJsn)
        spatialLocationData = json.loads(spatialLocationDataJsn)
        # print("spatialLocationData = ", spatialLocationData)
        time.sleep(0.2)
    elif (msg.topic == "topic/oakdIMU"):
        imuDataJsn = msg.payload.decode()
        print("imuDataJsn = ", imuDataJsn)
        imuData = json.loads(imuDataJsn)
        # print("imuData = ", imuData)
        time.sleep(0.2)


def animate1(i):
    global imuData
    print("imuData = ", imuData)

    # extract roll, pitch, yaw
    imuRoll, imuPitch, imuYaw = imuData

    # shift imuYaw 90 deg CW to coincide with robot heading
    imuYaw = (imuYaw + 90) %360

    print("---> imuRoll = ", imuRoll)
    print("---> imuPitch = ", imuPitch)
    print("---> imuYaw = ", imuYaw)

    # calculate 3d compass "needle" endpoints
    rollX0 = 3
    rollY0 = 1
    rollX1 = rollX0 + math.cos(math.radians(imuRoll))
    print("rollX1 = ", rollX1)
    rollY1 = rollY0 + math.sin(math.radians(imuRoll))
    print("rollY1 = ", rollY1)

    pitchX0 = 1
    pitchY0 = 1
    pitchX1 = pitchX0 + math.cos(math.radians(imuPitch))
    print("pitchX1 = ", pitchX1)
    pitchY1 = pitchY0 + math.sin(math.radians(imuPitch))
    print("pitchY1 = ", pitchY1)

    headingX0 = 5
    headingY0 = 1
    headingX1 = headingX0 + math.cos(math.radians(imuYaw))
    print("headingX1 = ", headingX1)
    headingY1 = headingY0 + math.sin(math.radians(imuYaw))
    print("headingY1 = ", headingY1)


    # graph the IMU 3d compass/gyro data

    ax1.clear()
    
    ax1.scatter(rollX0,rollY0, label='roll origin', color='b', s=25, marker="o")
    line2, = ax1.plot([rollX0,rollX1], [rollY0,rollY1], label='roll', lw=0.4, color='b', marker="None")

    ax1.scatter(pitchX0,pitchY0, label='pitch origin', color='r', s=25, marker="o")
    line1, = ax1.plot([pitchX0,pitchX1], [pitchY0,pitchY1], label='pitch', lw=0.4, color='r', marker="None")

    ax1.scatter(headingX0,headingY0, label='heading origin', color='g', s=25, marker="o")
    line3, = ax1.plot([headingX0,headingX1], [headingY0,headingY1], label='heading (yaw)', lw=0.4, color='g', marker="None")


    ax1.set_aspect('equal')
    ax1.set_xlabel('3d compass x-position')
    ax1.set_ylabel('3d compass Z-position')
    ax1.set_title('unit vectors')
    #ax1.legend(loc='upper right')
    # fix from chatgpt below to remove obstructing legend covering parts of the plot
    ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.4), ncol=3)


def animate2(i):
    global spatialLocationData
    print("spatialLocationData = ", spatialLocationData)
    
    # extract ROIx, ROIy, ROIz (3D depth 'z' data at x,y coords)
    ROIx, ROIy, ROIz = spatialLocationData

    # graph the spacialLocationCalc data
        
    ax2.clear()  # needed due to animation

    ax2.scatter(ROIz,ROIy, label='ROI z(distance)/y(height) location', color='b', s=25, marker="o")

    #ax2.set_aspect('equal')
    ax2.set_xlabel('ROIz')
    ax2.set_ylabel('ROIy')
    ax2.set_title('spatialLocationData')
    ax2.legend(loc='upper right')
    #ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.4), ncol=3)

    time.sleep(0.5)
    


client = mqtt.Client()
# client.connect("localhost",1883,60)   # connect to "localhost" broker
# client.connect("some_remote_IP_ADDR",1883,60)   # change "some_remote_IP_ADDR" to connect to remote
client.connect("10.0.0.231",1883,60)     # connect to remote broker

client.on_connect = on_connect
client.on_message = on_message

# client.loop_forever()
client.loop_start()


# plot the mqtt msg data (currently shared as globals)
try:
    ani1 = animation.FuncAnimation(fig1, animate1, interval=100, repeat=False)
    ani2 = animation.FuncAnimation(fig2, animate2, interval=100, repeat=False)
    plt.show()
    
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    client.disconnect()
    exit