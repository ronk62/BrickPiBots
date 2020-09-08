#!/usr/bin/env python
#
# Copyright (c) 2016 mindsensors.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#mindsensors.com invests time and resources providing this open source code,
#please support mindsensors.com  by purchasing products from mindsensors.com!
#Learn more product option visit us @  http://www.mindsensors.com/
#
# History:
# Date      Author      Comments
# 04/15/16   Deepak     Initial development.
#

import time, tty, sys
from PiStorms import PiStorms
from PiStormsCom import PiStormsCom
psm = PiStorms()

""" 
This program provides awsd keyboard remote control
over the MineLoader (lego model 42049) with large ev3 motors
driving the front wheels in differential-drive configuration.

MA1 (motor 1, bank A) = Left motor
MA2 (motor 2, bank A) = Right motor

Both motors are mounted such that positive power = forward motion
 """

doExit = False

tty.setcbreak(sys.stdin)

m1spd = 0
m2spd = 0

psm.BAM1.setSpeed(m1spd)
psm.BAM2.setSpeed(m2spd)

while (doExit == False):
    x = ord(sys.stdin.read(1))
    print x, m1spd, m2spd
    if x == 32: # space key pushed
        psm.BAM1.floatSync()
        m1spd = 0
        m2spd = 0
    if x == 119: # w key pushed
        if m1spd < 90:
            m2spd = m1spd = m1spd + 1
            psm.BAM1.setSpeedSync(m1spd)
    if x == 115: # s key pushed
        if m1spd > -90:
            m2spd = m1spd = m1spd - 1
            psm.BAM1.setSpeedSync(m1spd)
    if x == 97: # a key pushed
        if m1spd > 0:
            m1spd = m1spd - 1
            psm.BAM1.setSpeed(m1spd)
    if x == 100: # d key pushed
        if m2spd > 0:
            m2spd = m2spd - 1
            psm.BAM2.setSpeed(m2spd)
    time.sleep(.05)

    if(psm.isKeyPressed() == True): # if the GO button is pressed
        psm.screen.clearScreen()
        psm.screen.termPrintln("")
        psm.screen.termPrintln("Exiting to menu")
        doExit = True
