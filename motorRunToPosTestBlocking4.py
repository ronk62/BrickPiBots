#!/usr/bin/env python3
#
# template for ev3dev2-based code
#

import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach a motor to BAM1")
time.sleep(3)

start_pos = 0
end_pos = 0
start_time = 0.0
end_time = 0.0

m = LargeMotor(OUTPUT_A)
time.sleep(0.1)
m.reset()
time.sleep(0.1)
start_time = time.time()
m.stop_action='brake'


for i in range(1, 11):
    t1=time.time()
    # print("START --------> m.on_to_position ... or on_for_degrees", round((time.time() - t1), 4))
    #m.on_to_position(SpeedDPS(100), (i * 360), brake=False, block=True)  ### this method FAILS TO BLOCK
    #m.run_to_rel_pos(position_sp=(i * 360), speed_sp=600, stop_action="hold") ### no claim of blocking
    m.on_for_degrees(SpeedDPS(900), (i * 360), brake=True, block=True)  ### this method FAILS TO BLOCK
    #print("END --------> m.on_to_position ... or on_for_degrees")
    #time.sleep(2)
    # print("stop_action is  ", m.stop_action, round((time.time() - t1), 4))
    # print("state is  ", m.state, round((time.time() - t1), 4))
    # print("START --------> m.wait_while...blah", round((time.time() - t1), 4))
    m.wait_while('stalled')
    #print("END --------> m.wait_while...blah", round((time.time() - t1), 4))
    # print("stop_action is  ", m.stop_action, round((time.time() - t1), 4))
    # print("state is  ", m.state, round((time.time() - t1), 4))
    # time.sleep(2)
    print ("target pos = ", (i * 360), ";   actual pos = ", m.position, round((time.time() - t1), 4))
    m.off('hold')
    for j in range(10):
        time.sleep(0.5)
        print ("target pos = ", (i * 360), ";   actual pos = ", m.position, round((time.time() - t1), 4))
    m.reset()
    m.stop_action='brake'
    time.sleep(.1)
time.sleep(6)

