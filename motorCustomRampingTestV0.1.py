#!/usr/bin/env python3
#
# for ev3dev2-based code
#
# Date          Author      Change Notes
# 3/1/2020      Ron King    initial development for BrickPi3 device

"""
This code will test various motor ramping techniques. Among the ideas are...

- apply a low-pass filter (LPF) to the targetSpeed or targetDPS setting
  -- pros: simplicity and predictability for ramping "up"
  -- cons: no mechanism for accurately ramping down to target position w/out sensor data or known odometry (see below)
           this approach requires a loop to iterate the LPF, perhaps a separate thread
           has dependencies on loop timing; complexities if controlling more than a couple motors


- use a simple adder (and negative adder) to incr/decr speed or DPS in a loop
  -- pros: proven technique proposed on Dexter Industries forum my Matt Allen
  -- cons: dependencies on loop timing; complexities if controlling more than a couple motors


-- common "cons" for both ideas above are...
           1) will likely overshoot the target, unless additions are made to prevent that behavior
           2) if the end-position is unknown, it will be hard to use this to ramp "down"

-- possible additions to solve ramp down and accurate positioning problems
           1) if sensor data is providing the target end-position, a PID can be used to ramp down
           2) add some predictive method(s) to estimate distance for motor(S) to move - feed-forward concept
                a) use the "condidence factor" to adjust speed down as confidence in position, orientation, etc diminish
                b) provide a target position " best-guess" and slow based on time since last guess or update

"""
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS, MoveTank
from ev3dev2.sensor import INPUT_1

print ("Please attach a motor to PORT/OUTPUT_A")
time.sleep(3)

start_pos = 0
target_end_pos = 0
start_time = 0.0
end_time = 0.0
target_speedDPS = 900
adjusted_speedDPS = 0

m = LargeMotor(OUTPUT_A)
time.sleep(0.1)
m.reset()
time.sleep(0.1)
start_time = time.time()

m.on(SpeedDPS(0))
t1=time.time()

""" 
Notes:
 - LPF technique needs to be more generalized than it is currently (v0.1, 3/1/2020)
   - needs calculation that takes actual acceleration/decceleration, in DPS, into consideration
   - current implementation ramps up from '0' DPS to target_speedDPS and back down to '0'
   - need to alter to change from current DPS to new DSP (not assume starting at '0')
   - current implementation ramps up/down based on static time duration, rather than constant accel/decel rate


Possible implementation based on  ---> y = (m * x) + b

---- y = new adjusted_speedDPS
---- m (slope) = ramp_rateDPS
---- x delta_t (change in time - measured in seconds)
---- b = initial_speedDPS

-------> new adjusted_speedDPS = (ramp_rateDPS * delta_t) + initial_speedDPS

* this requires a check in the loop to ensure we stop accel/decel process when target is reached

if adjusted_speedDPS > target_speedDPS:
        adjusted_speedDPS = target_speedDPS

initial_speedDPS = {measured speed in DPS}               ### can this be done in ev3dev? - yes --> "m.speed", not "m.speed()"
target_speedDPS = 900                                    ### showing abitrary value; but some value is req'd, of course, to limit accel/decel in loop
ramp_rateDPS = 200                                       ### showing abitrary value; this can be a positive or negative value, or ramp_down loop can invert the sign

 """


for i in range(0, 50, 1):
    adjusted_speedDPS = adjusted_speedDPS + (0.025 * target_speedDPS)    ### Low-pass filter technique
    if adjusted_speedDPS > target_speedDPS:
        adjusted_speedDPS = target_speedDPS
    m.on(SpeedDPS(adjusted_speedDPS))
    time.sleep(.1)
    print("i = ", i, "  ramp_up adjusted_speedDPS = ", adjusted_speedDPS, ",  measured speedDPS = ", m.speed, "  elapsed time = ", round((time.time() - t1), 4))

time.sleep(3)

for i in range(0, 50, 1):
    adjusted_speedDPS = adjusted_speedDPS - (0.025 * target_speedDPS)    ### Low-pass filter technique
    if adjusted_speedDPS < 0:
        adjusted_speedDPS = 0
    m.on(SpeedDPS(adjusted_speedDPS))
    time.sleep(.1)
    print("i = ", i, "  ramp_down adjusted_speedDPS = ", adjusted_speedDPS, ",  measured speedDPS = ", m.speed,"  elapsed time = ", round((time.time() - t1), 4))

m.off()
m.reset()
time.sleep(1)

