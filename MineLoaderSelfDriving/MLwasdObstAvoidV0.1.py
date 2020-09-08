#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 4/19/2020     Ron King    used file /home/robot/ev3dev2Projects/wasdNonBlockingv1.0.py as starting point
#                           trimmed out some superfluous comments
#                           modified to add ir sensor (INPUT_1) in proximity mode and imu (INPUT_2) in compass mode


""" 
This program provides wasd keyboard remote control testing with obstacle avoidance (ir proximity sensor).

Intermediate test programs and trial runs

 1) drive straight toward obstacle and stop at {distance from obstacle}

Non-blocking...
- Using a multi-thread approach (see https://realpython.com/intro-to-python-threading/)
- created method to read x and moved `x = ord(sys.stdin.read(1))` there
- instantiated `x = 0` outside of any code blocks to make it global
- declared x as global within keyboardInput() method/thread
- had to set `x = 0` near the end of the main loop to prevent the equiv of a repeat/stuck keyboad key
-- this approach works great!


Motors (if needed)

motor A = Left motor
motor B = Right motor

 """

import time, tty, sys, threading
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import InfraredSensor
from ev3dev2.sensor.lego import Sensor


p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p1.set_device = 'lego-ev3-ir'


# Connect infrared to any sensor port
ir = InfraredSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

irProxVal = 0
prev_irProxVal = 0

p2 = LegoPort(INPUT_2)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p2.mode = 'nxt-i2c'

# allow for some time for mode to setup
time.sleep(0.5)

# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors

""" https://github.com/ev3dev/ev3dev/issues/640

dlech commented on May 15, 2016
    You are forgetting the I2C address for set_device

$ echo mx-pps58-nx 0x0C >/sys/class/lego-port/port0/set_device """

# set device name and i2c address (in hex)
p2.set_device = 'ms-absolute-imu 0x11'

# allow for some time for setup
time.sleep(0.5)

# Connect sensor to sensor port 1
imu = Sensor(INPUT_2)

# allow for some time to load the new drivers
time.sleep(0.5)

imu.mode = 'COMPASS'

compassVal = 0
prev_compassVal = 0


tty.setcbreak(sys.stdin)

mL = LargeMotor(OUTPUT_A)
time.sleep(0.5)

mL.reset()
time.sleep(2)
mLspd = 0
mL.stop_action = 'coast'
mL.polarity = 'inversed'
mL.position = 0
 

mR = LargeMotor(OUTPUT_B)
time.sleep(0.5)
mR.reset()
time.sleep(2)
mRspd = 0
mR.stop_action = 'coast'
mR.polarity = 'inversed'
mR.position = 0

spd = 0        # set this value to -30 to +90; use to set outer wheel/motor speed; default to mL for driving straight
turnRatio = 0.0  # set this value to -0.5 to +0.5; subtract from 1 to set turn ratio; multiply * spd to set inside motor speed

x = 0   # set here to make global

def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")

    while (True):
        irProxVal = ir.proximity
        if irProxVal != prev_irProxVal:
            print("irProxVal = ", irProxVal)
            prev_irProxVal = irProxVal
        
        compassVal = imu.value(0)
        if compassVal != prev_compassVal:
            print("compassVal = ", compassVal)
            prev_compassVal = compassVal

        if irProxVal < 50:
            print("Obstacle Detected! Forward motion stopped.")
            if spd > 0:
                spd = 0
                x = 33 # set x to non-zero so we print spd, etc; 33 = ascii ! 

        if x == 32 or x == 120: # space or x key pushed
            spd = 0
            turnRatio = 0
        if x == 119: # w key pushed
            if spd < 90:  # limit max frwd speed to 90
                spd = spd + 5
        if x == 115: # s key pushed
            if spd > -30:  # limit max rvrs speed to -30
                spd = spd - 5
        if x == 100: # d key pushed (turn more to the Right)
            if turnRatio > -0.5:
                turnRatio = turnRatio - 0.1
        if x == 97: # a key pushed (turn more to the Left)
            if turnRatio < 0.5:
                turnRatio = turnRatio + 0.1

        if turnRatio <= 0:
            mLspd = spd
            mRspd = spd * (1 + turnRatio)
        if turnRatio > 0:
            mRspd = spd
            mLspd = spd * (1 - turnRatio)

        mL.on(mLspd, brake=False)
        mR.on(mRspd, brake=False)
        if x == 120: # x key means exit
            break

        if x != 0:
            print("x, spd, turnRatio, mLspd, mRspd  ", x, spd, turnRatio, mLspd, mRspd)
        
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(0.2)

