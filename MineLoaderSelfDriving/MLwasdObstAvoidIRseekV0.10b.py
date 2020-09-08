#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 4/19/2020     Ron King    used file /home/robot/ev3dev2Projects/wasdNonBlockingv1.0.py as starting point
#                           trimmed out some superfluous comments
#                           modified to add ir sensor (INPUT_1) in proximity mode and imu (INPUT_2) in compass mode
# 4/26/2020     Ron King    Replaced Mindsendors IMU with HiTechnic NXT Compass
#                           Added the scaffolding for auto-pilot; partially implemented
# 5/3/2020      Ron King    Added IR Beacon data (distance, heading) - difficulties with simultaneous Prox and Seek
#                           This iteration has lead me to believe a better approach would be to use US to avoid obstacles
# 5/4/2020      Ron King    Added the US sensor (on port S3) and replaced IR Proximity process with code using the
#                           US distance_centimeters data for obstacle avoidance.
# 5/7/2020      Ron King    Added compass and beacon based driving and an end state/exit
#
# 5/23/2020     Ron King    Moved obstacle avoidance for auto-pilot into the if/elif ladder state machine
#                           Enhanced the obstacle avoidance process to use Near and Med distances as criteria                           
#                           Fixed some issues with the Med distance processing
#


""" 
This program provides wasd keyboard remote control testing with obstacle avoidance.

Intermediate test programs and trial runs

 1) drive straight toward obstacle and stop at {distance from obstacle}

Non-blocking...
- Using a multi-thread approach (see https://realpython.com/intro-to-python-threading/)
- created method to read x and moved `x = ord(sys.stdin.read(1))` there
- instantiated `x = 0` outside of any code blocks to make it global
- declared x as global within keyboardInput() method/thread
- had to set `x = 0` near the end of the main loop to prevent the equiv of a repeat/stuck keyboad key
-- this approach works great!


Motors

motor A = Left motor
motor B = Right motor

 """

import time, tty, sys, threading
from ev3dev2 import list_devices
from ev3dev2.port import LegoPort
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import InfraredSensor, UltrasonicSensor
from ev3dev2.sensor.lego import Sensor


p1 = LegoPort(INPUT_1)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p1.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p1.set_device = 'lego-ev3-ir'


# Connect infrared to sensor port
ir = InfraredSensor(INPUT_1)

# allow for some time to load the new drivers
time.sleep(0.5)

# irProxVal = 100
# prev_irProxVal = 0
irDistVal = 0
prev_irDistVal = 0
irHeadVal = 0
prev_irHeadVal = 0



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
p2.set_device = 'ht-nxt-compass 0x01'

# allow for some time for setup
time.sleep(0.5)

# Connect ht-nxt-compass sensor to sensor port
cmp = Sensor(INPUT_2)

# allow for some time to load the new drivers
time.sleep(0.5)

# cmp.mode = 'COMPASS'  ###  Not req'd for this sensor as it only has one mode

compassVal = 0
prev_compassVal = 0
compassGoal = 220     # with ht-compass mounted sideways, this translates to mostly North



p3 = LegoPort(INPUT_3)
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/brickpi3.html#brickpi3-in-port-modes
p3.mode = 'ev3-uart'
# http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html#supported-sensors
p3.set_device = 'lego-ev3-us'


# Connect lego-ev3-us to any sensor port
us = UltrasonicSensor(INPUT_3)

# allow for some time to load the new drivers
time.sleep(0.5)

usDistCmVal = 0
prev_usDistCmVal = 0



# some keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global
pilot_mode = 1  # start in human mode, '1'; '-1' means auto-pilot

# some Nav control vars
obstacleNear = False    # there is an obstacle nearby - need to stop, turn, backup, etc (and go another way)
obstacleMed = False     # there is an obstacle ahead - veer away
backingUp = False       # use this to control state of obstacle avoidance maneuvers
backingUpTime = 0.0     # use to control how long we backup
evadeLeft = False       # turn hard left for obstacle avoidance
evadeRight = False      # turn hard right for obstacle avoidance
evadeTime = 0.0         # use to control how long we evade left or right
initSearch = False      # have we completed the initial search circle, looking for ir beacon?
beaconLock = False      # has the beacon been found and data read?
cmpGenHeading = False   # do we have a lock on compass heading for North?
beaconSearch = False    # are we searching for the beacon right now?
cmpSearch = False       # are we searching for North with the compass right now?
navPrint = 10           # print the Nav msgs every nth cycle
i = 0                   # iterator for testing

# setup motors
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


def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")


### Main Loop
    while (True):
        usDistCmVal = us.distance_centimeters
        if usDistCmVal != prev_usDistCmVal:
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_usDistCmVal = usDistCmVal

        irDistVal = ir.distance()
        irHeadVal = ir.heading()
        if (irDistVal != prev_irDistVal) or (irHeadVal != prev_irHeadVal):
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_irDistVal = irDistVal
            prev_irHeadVal = irHeadVal
            # if beacon has been found, set beaconLock True
            if irDistVal is not None:
                if irDistVal < 100:
                    beaconLock = True
                if irDistVal == 100:
                    beaconLock = False
            else:
                beaconLock = False

        compassVal = cmp.value(0)
        if compassVal != prev_compassVal:
            # print("usDistCmVal = ", usDistCmVal, "  irDistVal = ", irDistVal, "  irHeadVal = ", irHeadVal, "  compassVal = ", compassVal)  # print all sensor vals, regardless of which changed
            prev_compassVal = compassVal
            # if beaconLock is False and we're outside the compass North range, reset cmpGenHeading to False
            if compassVal < (compassGoal - 40) or compassVal > (compassGoal + 40):
                cmpGenHeading = False

        if usDistCmVal < 30:
            if obstacleNear == False:
                print("Obstacle Detected! Forward motion stopped.")
                if spd > 0:
                    spd = 0
                    if pilot_mode < 0:
                        obstacleNear = True
                    x = 33 # set x to non-zero so we print spd, etc; 33 = ascii !
        elif (usDistCmVal >= 30)  and (usDistCmVal < 100):
            if pilot_mode < 0:
                obstacleMed = True

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

        if x == 112: # p pushed - toggle pilot mode between human and auto
            pilot_mode *= -1
            print("toggled pilot_mode; now set to...  ", pilot_mode)

        if irDistVal is not None:
            if pilot_mode < 0 and irDistVal <= 25:
                # stop motors
                mL.on(0, brake=False)
                mR.on(0, brake=False)
                spd = 0
                turnRatio = 0
                pilot_mode = 1    # destination reached; return to human pilot mode
                print("")
                print("")
                print("destination reached; returned to human pilot mode")
                print("")
                print("")
                time.sleep(3)

        if pilot_mode < 0:
            if  obstacleNear == True:
                obstacleMed = False
                if not (backingUp or evadeLeft or evadeRight):
                    # stop and backup (straight back)
                    mL.on(0, brake=False)
                    mR.on(0, brake=False)
                    backingUp = True
                    backingUpTime = time.time()
                if backingUp == True:
                    print("Auto-pilot collision avoidance - backing up")
                    # backup at speed 5 with equal speed applied to left and right motors
                    spd = -5
                    turnRatio = 0
                    # backup for only 3 seconds - then stop
                    if time.time() - backingUpTime >= 3:
                        print(time.time() - backingUpTime)
                        backingUp = False
                        spd = 0
                        turnRatio = 0
                        mL.on(spd, brake=False)
                        mR.on(spd, brake=False)
                        # determine next action based on compass orientaion
                        if compassVal > 240:
                            evadeLeft = True
                        else:
                            evadeRight = True
                        print("Auto-pilot collision avoidance - turning")
                        evadeTime = time.time()
                        spd = 5
                if evadeLeft == True:
                    turnRatio = 0.5
                elif evadeRight == True:
                    turnRatio = -0.5
                if (evadeLeft or evadeRight) and (time.time() - evadeTime >= 3):
                        spd = 0
                        turnRatio = 0
                        mL.on(spd, brake=False)
                        mR.on(spd, brake=False)
                        obstacleNear = False
                        evadeLeft = False
                        evadeRight = False

            elif obstacleMed == True:
                if not (evadeLeft or evadeRight):
                    print("Auto-pilot collision avoidance - making brief turn")
                    evadeTime = time.time()
                # determine next action based on compass orientaion
                if compassVal > 240:
                    evadeLeft = True
                else:
                    evadeRight = True
                spd = 5
                if evadeLeft == True:
                    turnRatio = 0.4
                elif evadeRight == True:
                    turnRatio = -0.4
                if time.time() - evadeTime >= 2:
                    spd = 5
                    obstacleMed = False
                    evadeLeft = False
                    evadeRight = False

            elif beaconLock:   # if we have a good lock on the ir beacon, use this for beacon Nav
                beaconSearch = False
                # speed and steering controlled by beacon Nav
                # set speed (slow)
                spd = 20
                # use beacon heading to correct steering
                # correct left
                if irHeadVal < -24:
                    turnRatio = 0.5
                elif irHeadVal < -20:
                    turnRatio = 0.4
                elif irHeadVal < -15:
                    turnRatio = 0.3
                elif irHeadVal < -10:
                    turnRatio = 0.2
                elif irHeadVal < -5:
                    turnRatio = 0.1
                # correct right
                elif irHeadVal > 24:
                    turnRatio = -0.5
                elif irHeadVal > 20:
                    turnRatio = -0.4
                elif irHeadVal > 15:
                    turnRatio = -0.3
                elif irHeadVal > 10:
                    turnRatio = -0.2
                elif irHeadVal > 5:
                    turnRatio = -0.1
                else:
                    turnRatio = 0
                
            elif cmpGenHeading:  # if compass Nav has us going mostly North, use compass Nav
                beaconSearch = True
                # speed (slow) and steering controlled by compass Nav
                # set speed (slow)
                spd = 15   #####  set to 0 for testing
                # use compass heading to correct steering
                # correct left
                if compassVal > 275:
                    turnRatio = 0.5
                elif compassVal > 260:
                    turnRatio = 0.3
                elif compassVal > 250:
                    turnRatio = 0.2
                elif compassVal > 240:
                    turnRatio = 0.1
                # correct right
                elif compassVal < 185:
                    turnRatio = -0.5
                elif compassVal < 200:
                    turnRatio = -0.3
                elif compassVal < 205:
                    turnRatio = -0.2
                elif compassVal < 220:
                    turnRatio = -0.1
                else:
                    turnRatio = 0
                
            elif not initSearch:  # if we have not yet circled, looking for ir beacon, do so now
                beaconSearch = True

                ### comment this code block and uncomment the below section for stationary testing
                # set speed (slow)
                spd = 10
                # initiate a CW 360 deg circle while looking for beacon; progress in segments to ensure full circle
                turnRatio = -0.5
                if i == 0:
                    # turn to compass val between 300 and 350
                    if compassVal > (300) and compassVal < (350):
                        i = 1
                elif i == 1:
                    # turn to compass val between 120 and 130
                    if compassVal > (120) and compassVal < (130):
                        i = 2
                elif i == 2:
                    # turn to compass val between 300 and 350
                    if compassVal > (300) and compassVal < (350):
                        i = 0
                        initSearch = True
                        print("Failed to find beacon. Continuing with compass North strategy")
                else:
                    i = 0
                ### uncomment the code below for stationary testing (comment the block above)
                # i += 1
                # if i > 200:
                #     i = 0
                #     initSearch = True
                #     print("Failed to find beacon. Continuing with compass North strategy")
                
                if beaconLock:  # once the beacon is found, set i to 0 and initSearch True
                    i = 0
                    initSearch = True

            else:  # default action is to find North (compassGoal = ~220) and head that way while looking for beacon                
                cmpSearch = True
                # set speed (slow)
                spd = 10
                # circle CW looking for compassGoal +/- 5
                turnRatio = -0.5
                if compassVal > (compassGoal - 5) and compassVal < (compassGoal + 5):
                    cmpGenHeading = True
                    cmpSearch = False
                    print("We found North. Done with cmpSearch")

        # calculate motor speeds for turning right
        if turnRatio <= 0:
            mLspd = spd
            mRspd = spd * (1 + turnRatio)
        
        # calculate motor speeds for turning left
        if turnRatio > 0:
            mRspd = spd
            mLspd = spd * (1 - turnRatio)

        # apply motor speeds
        mL.on(mLspd, brake=False)
        mR.on(mRspd, brake=False)

        if x == 120: # x key means exit
            break

        # if x != 0:
        #     print("x, spd, turnRatio, mLspd, mRspd, pilot_mode  ", x, spd, turnRatio, mLspd, mRspd, pilot_mode)
        
        if navPrint == 10:
            print("NAV:  x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock  ", x, spd, turnRatio, pilot_mode, beaconSearch, cmpSearch, cmpGenHeading, beaconLock)
            navPrint = 0
        navPrint += 1
        if (obstacleNear or obstacleMed):
            print("obstacleNear ", obstacleNear,  "obstacleMed ", obstacleMed )
        x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
        time.sleep(0.2)

