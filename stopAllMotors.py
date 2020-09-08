#!/usr/bin/env python3
#
# uses ev3dev2-based code
#

import time
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print("stopping all motors")

mA = LargeMotor(OUTPUT_A)
mB = LargeMotor(OUTPUT_B)
mC = LargeMotor(OUTPUT_C)
mD = LargeMotor(OUTPUT_D)
time.sleep(0.1)
mA.reset()
mC.reset()
mB.reset()
mD.reset()

mA.off(brake=False)
mC.off(brake=False)
mB.off(brake=False)
mD.off(brake=False)

time.sleep(0.1)

print("all motors should now be stopped, brake = False")
