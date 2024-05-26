#!/usr/bin/env python3

'''
start mosquitto daemon as follows:

sudo mosquitto -d

'''

import paho.mqtt.client as mqtt
import json


# This is the Publisher

client = mqtt.Client()
client.connect("localhost",1883,60)

def publish_data(roll, pitch, yaw):
    imuData = [roll, pitch, yaw]
    imuDataJsn = json.dumps(imuData)
    client.publish("topic/oakdIMU", imuDataJsn);
