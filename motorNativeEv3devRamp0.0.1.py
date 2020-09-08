#!/usr/bin/env python3
#
# ev3dev2-based code
#

import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach a motor to PORT_A")
time.sleep(1)

start_pos = 0
end_pos = 0
start_time = 0.0
end_time = 0.0

m = LargeMotor(OUTPUT_A)
time.sleep(0.1)
m.reset()
time.sleep(0.1)
start_time = time.time()
m.stop_action ='coast'
m.ramp_up_sp = 2000     #### supported on BrickPi3; ramp_up seems to work but ramp_down...not so well, at least with stop_action='brake'
m.ramp_down_sp = 1000  #### supported on BrickPi3; ramp_up seems to work but ramp_down...not so well, at least with stop_action='brake'

t1=time.time()

for i in range(10, 500, 10):
    print("START --------> m.on...", round((time.time() - t1), 4))
    #m.on_to_position(SpeedDPS(200), (i * 360), brake=False, block=True)  ### this method FAILS TO BLOCK with PiStorms
    m.run_to_rel_pos(position_sp=(i * 360), speed_sp=900, stop_action="coast") ### no claim of blocking
    m.wait_until_not_moving()
    #m.on_for_degrees(SpeedDPS(900), (i * 360), brake=False, block=True)  ### this method FAILS TO BLOCK with PiStorms
    #m.on_for_seconds(SpeedDPS(900), 15, brake=False, block=True)
    print ("target pos = ", (i * 360), ";   actual pos = ", m.position, round((time.time() - t1), 4))
    #print ("target pos = n/a (on_for_seconds);   actual pos = ", m.position, round((time.time() - t1), 4))
    time.sleep(.1)
time.sleep(10)
m.off()

