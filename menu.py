#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
# import subroutine.bridge
# import ./subroutine/follow
# import ./subroutine/move
from subroutine.follow import line_follower
from subroutine.bridge import bridge_main


# Other Imports
import math
import time

ev3 = EV3Brick()

sections = ["FOLLOW", "SEARCH", "MOVE", "BRIDGE"]
selec = 0

def check_abort(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2):
    if Button.LEFT in ev3.buttons.pressed():
        main_menu(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2)

def main_menu(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2):
    bridge_main(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2) #testing
    global selec

    load_screen()
    while True:
        if Button.DOWN in ev3.buttons.pressed():
            if selec == len(sections) - 1:
                selec = 0
            else:
                selec+=1
            load_screen()
        if Button.UP in ev3.buttons.pressed():
            if selec == 0:
                selec = len(sections) - 1
            else:
                selec-=1
            load_screen()

        if Button.RIGHT in ev3.buttons.pressed():
            if sections[selec] is 'FOLLOW':
                ev3.screen.clear()
                ev3.screen.print("Following line")
                line_follower(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2)
            elif sections[selec] is 'BRIDGE':
                ev3.screen.clear()
                ev3.screen.print("Bridge")
                bridge_main(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2)
            elif sections[selec] is 'MOVE':
                pass
            elif sections[selec] is 'SEARCH':
                pass

            else:
               pass
        
        if Button.CENTER in ev3.buttons.pressed():
            break

            

def load_screen():
    ev3.screen.clear()

    for i in range(len(sections)):
           ev3.screen.draw_text(20, 20 * i, sections[i])
    
    ev3.screen.draw_circle(5, 10 + 20 * selec, 5, True)
    #ev3.screen.draw_text(80, 80, selec)
    time.sleep(0.25)