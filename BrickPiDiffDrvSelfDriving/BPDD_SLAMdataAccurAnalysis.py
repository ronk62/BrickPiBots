#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 7/4/2020      Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMdataAccurV6.01b as starting point
#                           - this program will allow analysis of collected data
#


import time, tty, sys, threading
import matplotlib.pyplot as plt
import numpy as np

# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

# Nav control vars
sample_mode = -1        # start in sample mode off, '-1'; '1' means sample mode is enabled
navPrint = 10           # print the Nav msgs every nth cycle
printVerbose = 1        # toggle verbose data/msg printing to terminal (-1 is 'disabled')
i = 0                   # "outer" iterator for dist point sampling
j = 0                   # "inner" iterator for dist point sampling
prev_mRposition = 0     # var to help detect when to save a new OD pair (motor pos, cmpVal) to array or print to screen

# SLAM vars
polarCoordsUSr = np.array([], dtype=np.int32)       # arrary to hold 'r' - US distance r (in cm)
polarCoordscmpDeg = np.array([], dtype=np.int32)    # arrary to hold 'thetaDeg' - compass-angle  in degrees
# added IR
polarCoordsIRr = np.array([], dtype=np.int32)       # arrary to hold 'r' - IR distance r (in arb units)
# added array motor encoder ticks for odometry
ODmRencoderVal = np.array([], dtype=np.int32)       # arrary to hold right motor encoder-count while moving
# added another array for compass odometry
ODcmpDeg = np.array([], dtype=np.int32)             # array to hold associated compass angle (degrees) for each motor tick


def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")



### Main Loop
while (True):        
    if x == 118: # v pushed - toggle verbosity on and off
        printVerbose *= -1
        print("toggled printVerbose mode (v)")

    if x == 112: # p pushed - print sample arrays to screen
        if polarCoordsUSr.size < 1 or polarCoordscmpDeg.size < 1 or polarCoordsIRr.size < 1 or ODmRencoderVal.size < 1:
            print("no data at this time")
        else:
            print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
            print("")
            print ("polarCoordsUSr = ", polarCoordsUSr)
            print("")
            print ("polarCoordsIRr.size = ", polarCoordsIRr.size)
            print("")
            print ("polarCoordsIRr = ", polarCoordsIRr)
            print("")
            print ("ODmRencoderVal.size = ", ODmRencoderVal.size)
            print("")
            print ("ODmRencoderVal = ", ODmRencoderVal)
            print("")
            print ("ODcmpDeg.size = ", ODcmpDeg.size)
            print("")
            print ("ODcmpDeg = ", ODcmpDeg)
            print("")
            print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
            print("")
            print ("polarCoordscmpDeg = ", polarCoordscmpDeg)
            print("")

    # if x == 102: # f pushed - save sample data to files
    #     print("")
    #     if polarCoordsUSr.size < 1 or polarCoordscmpDeg.size < 1 or polarCoordsIRr.size < 1 or ODmRencoderVal.size < 1 or ODcmpDeg.size < 1:
    #         print("no data at this time")
    #     else:
    #         print ("saving sample data to csv files")
    #         # save to csv files
    #         np.savetxt('polarCoordsUSrV601b.csv', polarCoordsUSr, delimiter=',')
    #         np.savetxt('polarCoordsIRrV601b.csv', polarCoordsIRr, delimiter=',')
    #         np.savetxt('ODmRencoderValV601b.csv', ODmRencoderVal, delimiter=',')
    #         #np.savetxt('ODcmpDegV601b.csv', ODcmpDeg, delimiter=',')
    #         np.savetxt('polarCoordscmpDegV601b.csv', polarCoordscmpDeg, delimiter=',')
    #         print ("saves commplete")
    #         print("")
    
    if x == 108: # l pushed - loading sample data from csv files
        print("")
        print ("loading sample data from csv files")
        # load from csv files
        polarCoordsUSr = np.loadtxt('polarCoordsUSrV601b-11.csv', delimiter=',')
        polarCoordsIRr = np.loadtxt('polarCoordsIRrV601b-11.csv', delimiter=',')
        ODmRencoderVal = np.loadtxt('ODmRencoderValV601b-11.csv', delimiter=',')
        #ODcmpDeg = np.loadtxt('ODcmpDegV601b.csv', delimiter=',')
        polarCoordscmpDeg = np.loadtxt('polarCoordscmpDegV601b-11.csv', delimiter=',')
        print ("loads commplete")
        print("")

    if x == 114: # r pushed - reinitializing arrays
        polarCoordsUSr = np.array([], dtype=np.int32)
        polarCoordscmpDeg = np.array([], dtype=np.int32)
        polarCoordsIRr = np.array([], dtype=np.int32)
        ODmRencoderVal = np.array([], dtype=np.int32)
        ODcmpDeg = np.array([], dtype=np.int32)
        print("r pushed - reinitialized polarCoords* arrays")

    # if x == 105: # i pushed - toggle sample mode on and off
    #     sample_mode *= -1
    #     print("toggled sample_mode (i); now set to...  ", sample_mode)
    
    # if sample_mode > 0:
    #     try:
    #         # init the data collection arrays for each sample collection
    #         polarCoordsUSr = np.array([], dtype=np.int32)
    #         polarCoordscmpDeg = np.array([], dtype=np.int32)
    #         polarCoordsIRr = np.array([], dtype=np.int32)
    #         ODmRencoderVal = np.array([], dtype=np.int32)
    #         ODcmpDeg = np.array([], dtype=np.int32)

    #         # set motor encoder to 0
    #         mR.position = 0
    #         prev_mRposition = 0

    #         # take samples while moving
    #         while usDistCmVal > 20:
    #             #mL.on(5, brake=False)
    #             mR.on(5, brake=False)
    #             usDistCmVal = us.distance_centimeters
    #             irDistVal = ir.distance()
    #             if irDistVal == None:
    #                 irDistVal = -1      ### set to -1 instead of None or numpy.savetxt will complain
    #             compassVal = cmp.value(0)
    #             polarCoordsUSr = np.append(polarCoordsUSr, usDistCmVal)
    #             polarCoordsIRr = np.append(polarCoordsIRr, irDistVal)
    #             polarCoordscmpDeg = np.append(polarCoordscmpDeg, compassVal)
    #             #if mR.position != prev_mRposition:
    #             #    prev_mRposition = mR.position
    #             ODmRencoderVal = np.append(ODmRencoderVal, mR.position)
    #             ODcmpDeg = np.append(ODcmpDeg, compassVal)
    #             if polarCoordsUSr.size > 100000:
    #                 sample_mode = -1
    #                 print("")
    #                 print("")
    #                 print ("Exiting sample mode due to sample size too large...")
    #                 print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
    #                 print("")
    #             time.sleep(.05)             
    #         mL.on(0, brake=True)
    #         mR.on(0, brake=True)
    #         print("")
    #         print ("polarCoordsUSr.size = ", polarCoordsUSr.size)
    #         print("")
    #         print ("polarCoordsUSr = ", polarCoordsUSr)
    #         print("")
    #         print ("polarCoordsIRr.size = ", polarCoordsIRr.size)
    #         print("")
    #         print ("polarCoordsIRr = ", polarCoordsIRr)
    #         print("")
    #         print ("ODmRencoderVal.size = ", ODmRencoderVal.size)
    #         print("")
    #         print ("ODmRencoderVal = ", ODmRencoderVal)
    #         print("")
    #         print ("ODcmpDeg.size = ", ODcmpDeg.size)
    #         print("")
    #         print ("ODcmpDeg = ", ODcmpDeg)
    #         print("")
    #         print ("polarCoordscmpDeg.size = ", polarCoordscmpDeg.size)
    #         print("")
    #         print ("polarCoordscmpDeg = ", polarCoordscmpDeg)
    #         print("")
    #         print("")
            
    #     except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    #         mL.on(0, brake=False)
    #         mR.on(0, brake=False)
    #         sample_mode = -1

    #     sample_mode = -1
    #     print("Sampling complete. Returning to keyboard cmd mode.")


    if x == 120: # x key means exit
        break

    x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key
    time.sleep(0.2)
