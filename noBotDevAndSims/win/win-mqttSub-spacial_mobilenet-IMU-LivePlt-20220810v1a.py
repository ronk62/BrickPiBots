# Note  - no she-bang required (or even used) in win-doze scripts

import paho.mqtt.client as mqtt
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time

# init global vars
randoAr = [0,0,0]

# setup for graphing the IMU/CAM data unit vectors
fig, ax = plt.subplots()

# This is the Subscriber

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # client.subscribe("topic/imuyaw")
    client.subscribe("topic/oakdIMU")


def on_message(client, userdata, msg):
    global randoAr
    if (msg.payload.decode() == "Q"):
        print(msg.payload.decode())
        print("quiting...")
        client.disconnect()
    else:
        # randoJsn = float(msg.payload.decode())
        randoJsn = msg.payload.decode()
        print("randoJsn = ", randoJsn)
        randoAr = json.loads(randoJsn)
        # print("randoAr = ", randoAr)
        time.sleep(0.2)


def animate(i):
    global randoAr
    print("randoAr = ", randoAr)

    # extract roll, pitch, yaw
    imuPitch, imuRoll, imuYaw = randoAr

    print("---> imuPitch = ", imuPitch)
    print("---> imuRoll = ", imuRoll)
    print("---> imuYaw = ", imuYaw)

    # calculate 3d compass "needle" endpoints
    pitchX0 = 1
    pitchY0 = 1
    pitchX1 = pitchX0 + math.cos(math.radians(imuPitch))
    print("pitchX1 = ", pitchX1)
    pitchY1 = pitchY0 + math.sin(math.radians(imuPitch))
    print("pitchY1 = ", pitchY1)

    rollX0 = 3
    rollY0 = 1
    rollX1 = rollX0 + math.cos(math.radians(imuRoll))
    print("rollX1 = ", rollX1)
    rollY1 = rollY0 + math.sin(math.radians(imuRoll))
    print("rollY1 = ", rollY1)

    headingX0 = 5
    headingY0 = 1
    headingX1 = headingX0 + math.cos(math.radians(imuYaw))
    print("headingX1 = ", headingX1)
    headingY1 = headingY0 + math.sin(math.radians(imuYaw))
    print("headingY1 = ", headingY1)

    ax.clear()


    plt.scatter(pitchX0,pitchY0, label='pitch origin', color='r', s=25, marker="o")
    line1, = ax.plot([pitchX0,pitchX1], [pitchY0,pitchY1], label='pitch', lw=0.4, color='r', marker="None")

    plt.scatter(rollX0,rollY0, label='roll origin', color='b', s=25, marker="o")
    line2, = ax.plot([rollX0,rollX1], [rollY0,rollY1], label='roll', lw=0.4, color='b', marker="None")

    plt.scatter(headingX0,headingY0, label='heading origin', color='g', s=25, marker="o")
    line3, = ax.plot([headingX0,headingX1], [headingY0,headingY1], label='heading (yaw)', lw=0.4, color='g', marker="None")


    plt.axis('equal')
    plt.xlabel('3d compass x-position')
    plt.ylabel('3d compass Z-position')
    plt.title('unit vectors')
    plt.legend()
    time.sleep(0.5)


client = mqtt.Client()
# client.connect("localhost",1883,60)   # connect to "localhost" broker
# client.connect("some_remote_IP_ADDR",1883,60)   # change "some_remote_IP_ADDR" to connect to remote
client.connect("10.0.0.231",1883,60)     # connect to remote broker

client.on_connect = on_connect
client.on_message = on_message

# client.loop_forever()
client.loop_start()


### original code
# while True:
#     # plot the mqtt msg data (currently shared as globals)
#     try:
#         ani = animation.FuncAnimation(fig, animate, interval=100, repeat=False)
#         plt.show()
        
#     except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
#         client.disconnect()
#         exit


### modified from original for experimentation (likely more correct than the orig)
# plot the mqtt msg data (currently shared as globals)
try:
    ani = animation.FuncAnimation(fig, animate, interval=100, repeat=False)
    plt.show()
    
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    client.disconnect()
    exit