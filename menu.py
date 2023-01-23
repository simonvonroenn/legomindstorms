#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from subroutine.follow import line_follower_controller
from subroutine.move import move_main
from subroutine.search import search_main
from subroutine.bridge import bridge_main

# Other Imports
import math
import time

ev3 = EV3Brick()

sections = ["FOLLOW", "MOVE", "BRIDGE", "SEARCH"]
selec = 0

def check_abort(ev3, mLeft, mRight, sColor):
    if Button.LEFT in ev3.buttons.pressed():
        main_menu(ev3, mLeft, mRight, sColor)

def main_menu(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
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
            resume = False
            if sections[selec] is 'FOLLOW':
                ev3.screen.clear()
                ev3.screen.print("Following line")
                line_follower_controller(ev3, mLeft, mRight, sColor, sTRight, sTLeft)
                resume = True
            if sections[selec] is 'MOVE' or resume:
                ev3.screen.clear()
                ev3.screen.print("Move Box")
                move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft)
                resume = True
            if sections[selec] is 'BRIDGE' or resume:
                ev3.screen.clear()
                ev3.screen.print("Bridge")
                bridge_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft)
                # bridge method
                resume = True
            if sections[selec] is 'SEARCH' or resume:
                ev3.screen.clear()
                ev3.screen.print("Search Red Spot")
                search_main(ev3, mLeft, mRight, sColor)
        
        if Button.CENTER in ev3.buttons.pressed():
            break

            

def load_screen():
    ev3.screen.clear()

    for i in range(len(sections)):
           ev3.screen.draw_text(20, 20 * i, sections[i])
    
    ev3.screen.draw_circle(5, 10 + 20 * selec, 5, True)
    #ev3.screen.draw_text(80, 80, selec)
    time.sleep(0.25)