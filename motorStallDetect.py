#!/usr/bin/env python3
#
# uses ev3dev2-based code
#
#
# Date          Author      Change Notes
# 3/22/2020     Ron King    Initial dev


import time
# from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.motor import LargeMotor, OUTPUT_A

large_motor = LargeMotor(OUTPUT_A)

print("starting motor")

# large_motor.on(speed=30)
# large_motor.wait_until_not_moving()     ### this does not work with BrickPi3, state never changes as needed

# For BrickPi3 (and other non-EV3 platforms) the following may work better as control method for detecting a stall
mAspeed = 15            # speed setting
duty_cycle_margin = 10  # sensitivity buffer/margin; adjust to suite needs
duty_cycle_avg = 0
duty_cycle_min = 101
duty_cycle_max = -101
valChange = 0

large_motor.on(speed=mAspeed)

while large_motor.duty_cycle <= (mAspeed + duty_cycle_margin):
    duty_cycle_avg = (duty_cycle_avg + large_motor.duty_cycle) / 2
    duty_cycle_min = min(duty_cycle_min, large_motor.duty_cycle)
    duty_cycle_max = max(duty_cycle_max, large_motor.duty_cycle)
    if valChange != duty_cycle_min + duty_cycle_max:
        print("duty_cycle avg, min, max =  ", duty_cycle_avg, duty_cycle_min, duty_cycle_max)
    valChange = duty_cycle_min + duty_cycle_max
    time.sleep(0.1)

print("duty_cycle avg, min, max =  ", duty_cycle_avg, duty_cycle_min, duty_cycle_max)

large_motor.off()

time.sleep(0.1)

print("motor should now be stopped")
