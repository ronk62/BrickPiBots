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

def publish_data(spacialMobileNetDataList):
    # spacialMobileNetData = [x1, x2, y1, y2, label, locx, locy, locz, confidence]
    # spacialMobileNetDataJsn = json.dumps(spacialMobileNetData)
    spacialMobileNetDataJsn = json.dumps(spacialMobileNetDataList)
    client.publish("topic/spacialMobileNet", spacialMobileNetDataJsn);
