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

for i in range(1, 11):
    m.on_to_position(SpeedDPS(600), (i * 360), brake=False, block=True)  ### this method FAILS TO BLOCK
    # m.on_for_degrees(SpeedDPS(600), (i * 360), brake=False, block=True)  ### this method FAILS TO BLOCK
    # time.sleep(.1)
    m.wait_while('running')
    m.off(brake=True)
    print ("target pos = ", (i * 360), ";   actual pos = ", m.position)
    m.reset()
    time.sleep(.1)


