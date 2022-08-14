#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
import random
import time

# This is the Publisher

client = mqtt.Client()
client.connect("localhost",1883,60)
# rando = 5

while True:
    imuPitch = random.uniform(-15,15)
    print("---> imuPitch = ", imuPitch)
    imuRoll = random.uniform(-15,15)
    print("---> imuRoll = ", imuRoll)
    imuYaw = random.uniform(0,359)
    print("---> imuYaw = ", imuYaw)
    randoAr = [imuPitch, imuRoll, imuYaw]
    randoJsn = json.dumps(randoAr)
    client.publish("topic/oakdIMU", randoJsn);
    time.sleep(3)


# if rando < 1:
#     client.publish("topic/imuyaw", "Q");
#     client.disconnect();

# if 1 <= rando < 3:
#     client.publish("topic/imuyaw", "Hello world!");

# if 3 <= rando < 6:
#     client.publish("topic/imuyaw", 133);

# if 6 <= rando < 11:
    client.publish("topic/imuyaw", 8.008);
