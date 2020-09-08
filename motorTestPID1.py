#!/usr/bin/env python3
#
# uses ev3dev2-based code
#

import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach a motor to BAM1")
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
goal_DPS = 400
goal_diff = 99999.99
best_goal_diff = 999999.99
optimal_speed_i = 0
start_pos = 0
end_pos = 0
start_time = 0.0
end_time = 0.0

m = LargeMotor(OUTPUT_A)
time.sleep(0.1)
m.reset()
time.sleep(0.1)

# m.ramp_up_sp(5000)     #### unsupported (on PiStorms only?)
# m._ramp_down_sp(5000)  #### unsupported (on PiStorms only?)

m.position_p = 7000
m.position_i = 0
m.position_d = 37500

print("position_p =   ", m.position_p)  # powerup default is 7000
print("position_i =   ", m.position_i)  # powerup default is 0
print("position_d =   ", m.position_d)  # powerup default is 37500

m.speed_p = 15000
m.speed_i = 300
m.speed_d = 7500

print("speed_p =   ", m.speed_p)        # powerup default is 15000
print("speed_i =   ", m.speed_i)        # powerup default is 300
print("speed_d =   ", m.speed_d)        # powerup default is 7500

start_time = time.time()

""" ### quick test
m.position = 0
print(m.position) """


for j in range(250, 351, 5):
    m.speed_p = 15000
    m.speed_i = j
    m.speed_d = 7500
    tachospeed_min = 999999.99
    tachospeed_max = 0.0
    start_pos = 0
    start_time = time.time()
    m.on(SpeedDPS(goal_DPS))
    for i in range(100):
        t1 = time.time()
        old_pos = encoder_pos
        time.sleep(0.05)
        encoder_pos = m.position
        dt = time.time() - t1
        d_deg = encoder_pos - old_pos
        tachospeed = d_deg / dt
        d_deg_avg = ((d_deg_avg + d_deg) / (2))
        tachospeed_avg = ((tachospeed_avg + tachospeed) / (2))
        if tachospeed < tachospeed_min:
            tachospeed_min = tachospeed
        if tachospeed > tachospeed_max:
            tachospeed_max = tachospeed
        # if abs(d_deg) > 100:
        #     print ("d_deg = ", d_deg, "; d_deg_avg", d_deg_avg, "; tachospeed_avg", tachospeed_avg)
    m.off()
    end_time = time.time()
    end_pos = m.position
    print ("running data collection:")
    print ("i = ", i, "; j = ", j)
    print ("d_deg = ", d_deg, "; d_deg_avg", d_deg_avg, "; tachospeed_avg", tachospeed_avg)
    print ("tachospeed_min = ", tachospeed_min, "; tachospeed_max = ", tachospeed_max)
    # print ("")
    # print ("static data collection:")
    # print ("start_time = ", start_time, "; end_time", end_time)
    # print ("start_pos = ", start_pos, "; end_pos", end_pos)
    # print ("time diff  ", end_time - start_time)
    # print ("pos diff  ", end_pos - start_pos)
    print ("d_pos / d_t", (end_pos - start_pos) / (end_time - start_time))
    goal_diff = abs((goal_DPS - tachospeed_max))
    if goal_diff < best_goal_diff:
        best_goal_diff = goal_diff
        optimal_speed_i = j
        print ("current best_goal_diff = ", best_goal_diff, "; current optimal_speed_i", optimal_speed_i)
    m.reset()
print ("Final data:")
print ("best_goal_diff = ", best_goal_diff, "; optimal_speed_i", optimal_speed_i)