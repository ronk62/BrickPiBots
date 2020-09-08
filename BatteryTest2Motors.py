#!/usr/bin/env python3
#
# uses ev3dev2-based code
#

import time
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.power import PowerSupply

b1=PowerSupply()

print ("Please attach two motors - one to BAM1 and another to BAM2")
time.sleep(3)

old_pos = 0
encoder_pos = 0
counter = 0
d_deg = 0
t1 = 0.0
dt = 0.0
tachospeed = 0.0  # degrs Per Sec
ev3dev2_speed = 0
d_deg_avg = 0.0
tachospeed_avg = 0.0
tachospeed_min = 999999.99
tachospeed_max = 0.0
goal_DPS = 0
goal_diff = 99999.99
best_goal_diff = 999999.99
optimal_speed_i = 0
start_pos = 0
end_pos = 0
start_time = 0.0
end_time = 0.0

initial_power = b1.measured_voltage

mA = LargeMotor(OUTPUT_A)
mB = LargeMotor(OUTPUT_B)
time.sleep(0.1)
mA.reset()
time.sleep(0.1)
start_time = time.time()

print("starting motors")

while(True):
    print("ramping motors up")
    for i in range(0, 1001, 5):
        mA.on(SpeedDPS(i))
        mB.on(SpeedDPS(i))
        time.sleep(0.05)
    print("ramping motors down")
    for i in range(1000, 0, -5):
        mA.on(SpeedDPS(i))
        mB.on(SpeedDPS(i))
        time.sleep(0.05)
    print("finished full cycle:     ", time.time() - start_time)
    print("current votage:      ", (b1.measured_voltage / 1000000))
    print("votage drop since start:     ", ((initial_power - b1.measured_voltage) / 1000000))