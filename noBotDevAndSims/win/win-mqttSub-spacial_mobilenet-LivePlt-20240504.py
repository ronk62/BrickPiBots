# Note  - no she-bang required (or even used) in win-doze scripts

import paho.mqtt.client as mqtt
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time

# init global vars
# List_of_lists with [x1, x2, y1, y2, label, locx, locy, locz, confidence]
spacialMobileNetDataList = []

# setup for graphing the spacialMobileNetData
fig, ax = plt.subplots()

# This is the Subscriber

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("topic/spacialMobileNet")


def on_message(client, userdata, msg):
    global spacialMobileNetDataList
    if (msg.payload.decode() == "Q"):
        print(msg.payload.decode())
        print("quiting...")
        client.disconnect()
    else:
        spacialMobileNetDataJsn = msg.payload.decode()
        
        # for testing
        #print("spacialMobileNetDataJsn = ", spacialMobileNetDataJsn)
        spacialMobileNetDataList = json.loads(spacialMobileNetDataJsn)

        # for testing
        print("spacialMobileNetData = ", spacialMobileNetDataList)
        time.sleep(0.2)


def animate(i):
    global spacialMobileNetDataList
    print("spacialMobileNetData = ", spacialMobileNetDataList)

    locxList = []
    locyList = []
    loczList = []

    ax.clear()

    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
    count = 0

    for spacialMobileNetData in spacialMobileNetDataList:
        # assign a color for each object instance, max object = 8
        color = colors[count]
        count += 1
        # extract x1, x2, y1, y2, label, locx, locy, locz, confidence
        x1, x2, y1, y2, label, locx, locy, locz, confidence = spacialMobileNetData
        locxList.append(locx)
        locyList.append(locy)
        loczList.append(locz)

        plt.scatter(locx,locz, label=label, color=color, s=25, marker="o")

    # ax.clear()


    # plt.scatter(locxList,locyList, label='x/y loc', color='b', s=25, marker="o")


    # plt.scatter(locxList,loczList, label='x/z loc', color='r', s=25, marker="o")


    plt.axis('equal')
    plt.xlabel('Object x-position')
    plt.ylabel('Object z-position')
    plt.title('Object locations')
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


# plot the mqtt msg data (currently shared as globals)
try:
    ani = animation.FuncAnimation(fig, animate, interval=100, repeat=False)
    plt.show()
    
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    client.disconnect()
    exit