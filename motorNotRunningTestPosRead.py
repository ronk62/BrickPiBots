#!/usr/bin/env python3
#
# template for ev3dev2-based code
#

import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach a motor to PORT_A")
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
start_pos = 0
end_pos = 0
start_time = 0.0
end_time = 0.0

m = LargeMotor(OUTPUT_A)
time.sleep(0.1)
#m.reset()
m.stop_action = 'coast'
m.stop()
m.position = 0
time.sleep(0.1)
start_time = time.time()
#m.on(SpeedDPS(800))

while True:
    for i in range(6000):
        t1 = time.time()
        old_pos = encoder_pos
        time.sleep(0.05)
        encoder_pos = m.position
        dt = time.time() - t1
        d_deg = encoder_pos - old_pos
        tachospeed = d_deg / dt
        d_deg_avg = ((d_deg_avg + d_deg) / (2))
        tachospeed_avg = ((tachospeed_avg + tachospeed) / (2))
        if abs(d_deg) > 0:
            print ("d_deg = ", d_deg, "; d_deg_avg", d_deg_avg, "; tachospeed_avg", tachospeed_avg)

    #m.off()
    m.stop_action = 'coast'
    m.stop()
    end_time = time.time()
    end_pos = m.position

    # print ("running data collection:")
    # print ("d_deg = ", d_deg, "; d_deg_avg", d_deg_avg, "; tachospeed_avg", tachospeed_avg)
    # print ("")
    # print ("static data collection:")
    # print ("start_time = ", start_time, "; end_time", end_time)
    # print ("start_pos = ", start_pos, "; end_pos", end_pos)
    # print ("time diff  ", end_time - start_time)
    # print ("pos diff  ", end_pos - start_pos)
    print ("d_pos / d_t", (end_pos - start_pos) / (end_time - start_time))
    time.sleep(2)
