#!/usr/bin/env python3
#
# template for ev3dev2-based code
#
# from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

leds = Leds()

# print("Press the touch sensor to change the LED color!")

# while True:
#     if ts.is_pressed:
#         leds.set_color("LEFT", "GREEN")
#         leds.set_color("RIGHT", "GREEN")
#     else:
#         leds.set_color("LEFT", "RED")
#         leds.set_color("RIGHT", "RED")

# m = LargeMotor(OUTPUT_A)
m = MediumMotor(OUTPUT_A)
m.on_for_rotations(SpeedPercent(75), 5)       



# tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)

# # drive in a turn for 5 rotations of the outer motor
# # the first two parameters can be unit classes or percentages.
# tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(75), 10)

# # drive in a different turn for 3 seconds
# tank_drive.on_for_seconds(SpeedPercent(60), SpeedPercent(30), 3)

