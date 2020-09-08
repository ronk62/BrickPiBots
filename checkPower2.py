#!/usr/bin/env python3
#
# template for ev3dev2-based code
#
from ev3dev2.power import PowerSupply
import time

b1=PowerSupply()


while(True):
    print("measured_voltage: ", b1.measured_voltage)
    print("measured_volts: ", b1.measured_volts)
    #print("measured_current: ", b1.measured_current)   ### (not supported on Pistorms?)
    #print("measured_amps: ", b1.measured_amps)         ### (not supported on Pistorms?)
    #print("technology: ", b1.technology)               ### (not supported on Pistorms?)
    print("type: ", b1.type)
    time.sleep(15)


