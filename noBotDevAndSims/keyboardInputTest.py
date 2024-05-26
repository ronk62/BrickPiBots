#!/usr/bin/env python3

import time, tty, sys, threading

# keyboard control setup and vars
tty.setcbreak(sys.stdin)
x = 0   # set here to make global

def keyboardInput(name):
    while (True):
        global x  # Declare x as global to force use of global 'x' in this function/thread
        x = ord(sys.stdin.read(1))
        time.sleep(0.05)


if __name__ == "__main__":
    myThread = threading.Thread(target=keyboardInput, args=(1,), daemon=True)
    myThread.start()
    print ("Ready for keyboard commands...")

while True:
    print(x)

    x = 0   # req'd to prevent the equiv of a repeat/stuck keyboad key

    time.sleep(0.5)
