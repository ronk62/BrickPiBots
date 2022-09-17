#!/usr/bin/env python3

from re import X
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time

# init global vars
imuYaw = 0.0

# setup for graphing the IMU/CAM data unit vectors
fig, ax = plt.subplots()
line1, = ax.plot([],[]) # added for blitting

# This is the Subscriber

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("topic/imuyaw")


def on_message(client, userdata, msg):
    global imuYaw
    if (msg.payload.decode() == "Q"):
        print(msg.payload.decode())
        print("quiting...")
        client.disconnect()
    else:
        imuYaw = float(msg.payload.decode())
        print("imuYaw = ", imuYaw)
        
        time.sleep(0.2)

def init(): # added for blitting
    line1, = ax.plot([], [], label='heading (yaw)', lw=0.4, color='r', marker="None")
    return line1,

def animate(i):
    global imuYaw

    # calculate 3d compass "needle" endpoints
    headingX0 = 1
    headingY0 = 1
    headingX1 = headingX0 + math.cos(math.radians(imuYaw))
    print("headingX1 = ", headingX1)
    headingY1 = headingY0 + math.sin(math.radians(imuYaw))
    print("headingY1 = ", headingY1)

    # ax.clear()  # comment for blitting


    plt.scatter(headingX0,headingY0, label='heading origin', color='r', s=25, marker="o")
    line1, = ax.plot([headingX0,headingX1], [headingY0,headingY1], label='heading (yaw)', lw=0.4, color='r', marker="None")

    # plt.scatter(x1,y1, label='apriltag (v1) origin', color='y', s=25, marker="o")
    # line2, = ax.plot(vxx1,vxy1, label='v1 x', lw=0.4, color='y', marker="None")

    # plt.scatter(x2,y2, label='cam/bot (v2) origin', color='m', s=25, marker="o")
    # line3, = ax.plot(vxx2,vxy2, label='v2 x', lw=0.4, color='m', marker="None")

    plt.axis('equal')
    plt.xlabel('3d compass x-position')
    plt.ylabel('3d compass Z-position')
    plt.title('unit vectors')
    plt.legend()
    time.sleep(0.5)


client = mqtt.Client()
client.connect("localhost",1883,60)     # change "localhost" to some_IP_ADDR to connect to remote

client.on_connect = on_connect
client.on_message = on_message

# client.loop_forever()
client.loop_start()

while True:
    # plot the mqtt msg data (currently shared as globals)
    try:
        # ani = animation.FuncAnimation(fig, animate, interval=100, repeat=False)
        ani = animation.FuncAnimation(fig, animate, init_func=init, interval=100, repeat=False, blit=True)  # adding blit and init
        plt.show()
        
    except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
        client.disconnect()
        exit
