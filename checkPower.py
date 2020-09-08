#!/usr/bin/env python3
#
# template for ev3dev2-based code
#
from ev3dev2.power import PowerSupply
import time

b1=PowerSupply()


while(True):
    print(b1.measured_voltage)
    time.sleep(3)


