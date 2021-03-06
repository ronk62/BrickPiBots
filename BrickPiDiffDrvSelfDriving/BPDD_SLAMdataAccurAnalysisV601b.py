#!/usr/bin/env python3
#
# Date          Author      Change Notes
# 7/4/2020      Ron King    - used file /home/robot/ev3dev2Projects/BrickPiDiffDrvSelfDriving/BPDDwasdSLAMdataAccurV6.01b as starting point
#                           - this program will allow analysis of collected data
#
# 7/5/2020      Ron King    - numerous updates and tweaks; expect much more of the same
#                           - from V501b-7.csv -> 80 cm / 1469 ticks
#                             --> 0.054458816 cm / tick
#                             --> 18.3625 ticks / cm
#                           - from V501b-8.csv -> 100 cm / 1842 ticks
#                             --> 0.054288817 cm / tick
#                             --> 18.42 ticks / cm
#                           - similar from V501b-9.csv
#                             --> average of 18.452752525 ticks per cm
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
i = 0                   # "outer" iterator
j = 0                   # "inner" iterator for nested loops or calculations
prev_mRposition = 0     # var to help detect when to save a new OD pair (motor pos, cmpVal) to array or print to screen
deltaDistUS = 0         # var for change in US Distance
deltaMotorTick = 0      # var for change in Motor Ticks

TICKSPERCM = 18.452752525  # constant for motion calculations

# SLAM vars
polarCoordsUSr = np.array([], dtype=np.int32)       # arrary to hold 'r' - US distance r (in cm)
polarCoordscmpDeg = np.array([], dtype=np.int32)    # arrary to hold 'thetaDeg' - compass-angle  in degrees
# added IR
polarCoordsIRr = np.array([], dtype=np.int32)       # arrary to hold 'r' - IR distance r (in arb units)
# added array motor encoder ticks for odometry
ODmRencoderVal = np.array([], dtype=np.int32)       # arrary to hold right motor encoder-count while moving
# added another array for compass odometry
ODcmpDeg = np.array([], dtype=np.int32)             # array to hold associated compass angle (degrees) for each motor tick
ODt = np.array([], dtype=np.int32)                  # array to represent each time/tick - this will be the x axis independant var
USdistPerTick = np.array([], dtype=np.int32)        # array to hold calculated US Distance per Motor encoder tick
IRdistPerTick = np.array([], dtype=np.int32)        # array to hold calculated IR Distance per Motor encoder tick

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
            print ("ODt.size = ", ODt.size)
            print("")
            print ("ODt = ", ODt)
            print("")
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
    #         np.savetxt('polarCoordsUSrV501b.csv', polarCoordsUSr, delimiter=',')
    #         np.savetxt('polarCoordsIRrV501b.csv', polarCoordsIRr, delimiter=',')
    #         np.savetxt('ODmRencoderValV501b.csv', ODmRencoderVal, delimiter=',')
    #         #np.savetxt('ODcmpDegV501b.csv', ODcmpDeg, delimiter=',')
    #         np.savetxt('polarCoordscmpDegV501b.csv', polarCoordscmpDeg, delimiter=',')
    #         print ("saves commplete")
    #         print("")
    
    if x == 108: # l pushed - loading sample data from csv files
        print("")
        print ("loading sample data from csv files")
        # load from csv files
        polarCoordsUSr = np.loadtxt('polarCoordsUSrV601b-14.csv', delimiter=',')
        polarCoordsIRr = np.loadtxt('polarCoordsIRrV601b-14.csv', delimiter=',')
        ODmRencoderVal = np.loadtxt('ODmRencoderValV601b-14.csv', delimiter=',')
        #ODcmpDeg = np.loadtxt('ODcmpDegV501b.csv', delimiter=',')
        polarCoordscmpDeg = np.loadtxt('polarCoordscmpDegV601b-14.csv', delimiter=',')
        print ("loads commplete")
        print("")
        # calculating additional values
        print("creating ODt for use as x axis values (if needed)...")
        print("")
        for i in range(0,len(polarCoordsUSr)):
            ODt = np.append(ODt, i)
            #print("i = ", i)
        #print("ODt = ", ODt)
        print("")
        # calculated delta dist per motor tick
        print("populating USdistPerTick array...")
        print("")
        for i in range(1,(len(polarCoordsUSr) - 1)):
            deltaDistUS = polarCoordsUSr[i] - polarCoordsUSr[i-1]
            deltaMotorTick = ODmRencoderVal[i] - ODmRencoderVal[i-1]
            USdistPerTick = np.append(USdistPerTick, (deltaDistUS/deltaMotorTick))
            print("i, deltaDistUS, deltaMotorTick", i, deltaDistUS, deltaMotorTick)
        USdistPerTick = np.append(USdistPerTick, 0)
        USdistPerTick = np.append(USdistPerTick, 0)
        print("USdistPerTick = ", USdistPerTick)
        print("")
        print("values have been created")
        print("")

        #IRdistPerTick  - add routine here when ready

    if x == 114: # r pushed - reinitializing arrays
        polarCoordsUSr = np.array([], dtype=np.int32)
        polarCoordscmpDeg = np.array([], dtype=np.int32)
        polarCoordsIRr = np.array([], dtype=np.int32)
        ODmRencoderVal = np.array([], dtype=np.int32)
        ODcmpDeg = np.array([], dtype=np.int32)
        print("r pushed - reinitialized polarCoords* arrays")
    
    if x == 103: # g pushed - graph the data
        plt.plot(ODt,polarCoordsUSr, label='polarCoordsUSr')
        plt.plot(ODt,polarCoordscmpDeg, label='polarCoordscmpDeg')
        plt.plot(ODt,polarCoordsIRr, label='polarCoordsIRr')
        plt.plot(ODt,ODmRencoderVal, label='ODmRencoderVal')
        #plt.plot(ODt,ODcmpDeg, label='ODcmpDeg')
        plt.xlabel('ODt')
        #plt.plot(ODmRencoderVal,polarCoordsUSr, label='Distance (US) at motor tick')
        #plt.plot(ODt,USdistPerTick, label='Distance (US) per motor tick')
        #plt.plot(ODt,ODmRencoderVal, label='Motor ticks')
        #plt.xlabel('ODmRencoderVal')
        plt.ylabel('sensor data')
        #plt.ylabel('processed data')
        plt.title('Odometry data')
        plt.legend()
        plt.show()

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
